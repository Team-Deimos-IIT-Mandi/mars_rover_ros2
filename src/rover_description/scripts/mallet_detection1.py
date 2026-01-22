#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import os
from PIL import Image as PILImage
from ament_index_python.packages import get_package_share_directory
from ultralytics import FastSAM
import clip
from datetime import datetime

class MalletPhotoCaptureNode(Node):
    def __init__(self):
        super().__init__('mallet_photo_capture_node')

        self.save_dir = os.path.join(os.path.expanduser('~'), 'captured_mallets')
        os.makedirs(self.save_dir, exist_ok=True)

        # 1. Device Setup
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")
        
        # 2. Path Setup (Dynamic for Jetson/Rpi deployment)
        try:
            pkg_share = get_package_share_directory('rover_description')
            fastsam_weights = os.path.join(pkg_share, 'od_models', 'FastSAM-s.pt')
            clip_weights = os.path.join(pkg_share, 'od_models', 'ViT-B-32.pt')
            
            # Verify files exist before loading
            if not os.path.exists(fastsam_weights) or not os.path.exists(clip_weights):
                self.get_logger().error(f"Weights NOT FOUND in: {pkg_share}/models/")
                return

            # 3. Load Models from LOCAL paths
            self.get_logger().info(f"Loading FastSAM from: {fastsam_weights}")
            self.model = FastSAM(fastsam_weights) 

            self.get_logger().info(f"Loading CLIP from: {clip_weights}")
            # Use jit=False for better compatibility on edge devices
            self.clip_model, self.preprocess = clip.load(clip_weights, device=self.device, jit=False)
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize models: {e}")
            return

        # 4. Configuration
        # Better label set to help CLIP distinguish the mallet from environment features
        self.target_labels = [
            "an orange mallet",
            "an orange tool with handle",
            "shape like a hammer",
            "a mallet with an orange head and may be orange handle"
        ]
        
        self.background_labels = [
            "ground and dirt",
            "a photo of the environment",
            "random debris or clutter",
            "natural vegetation",
            "man-made structure",
            "the shadow of an object"
        ]
        
        self.labels = self.target_labels+self.background_labels

        self.target_indices = list(range(len(self.target_labels)))
        self.bg_indices = list(range(len(self.target_labels), len(self.labels)))

        self.target_text = clip.tokenize(self.labels).to(self.device)
        self.mission_started = False
        self.found_and_stopped = False
        self.bridge = CvBridge()
        self.clip_confidence_threshold = 0.70 
        
        # 5. ROS Communications
        self.image_sub = self.create_subscription(Image, '/camera_1/image_raw', self.image_callback, 10)
        self.start_sub = self.create_subscription(Bool, '/start_search', self.start_cb, 10)
        self.stop_pub = self.create_publisher(Bool, '/stop_search', 10)
        self.stop_sub = self.create_subscription(Bool, '/stop_search',self.stop_node, 10)
        self.labeled_img_pub = self.create_publisher(Image, '/station/mallet_captured_image', 10)

        self.get_logger().info("Mallet Photo Capture Node Initialized and Weights Loaded.")

   
    
    def start_cb(self, msg):
        self.mission_started = msg.data
        if self.mission_started:
            self.get_logger().info("Spiral search active. Scanning...")

    def stop_node(self,msg):
        self.found_and_stopped = msg.data
        if(self.found_and_stopped):
            self.get_logger().info("Image published to station. Silencing this node.")

    def is_orange(self, crop):
        """Helper to check if the crop actually contains orange pixels."""
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([25, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        orange_pct = np.sum(mask > 0) / mask.size
        return orange_pct > 0.2
    

    def image_callback(self, msg):
        if not self.mission_started or self.found_and_stopped:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # --- Step 1: FastSAM Segmentation ---
        results = self.model(cv_image, device=self.device, retina_masks=True, imgsz=640, conf=0.5)
        masks = results[0].masks

        if masks is None:
            return

        for i, mask in enumerate(masks.data):
            mask_np = mask.cpu().numpy().astype(bool)
            coords = np.argwhere(mask_np)
            if len(coords) == 0: continue
            
            y_min, x_min = coords.min(axis=0)
            y_max, x_max = coords.max(axis=0)
            crop = cv_image[y_min:y_max, x_min:x_max]
            
            if crop.size == 0: continue

            if not self.is_orange(crop):
                continue
            
            # --- Step 2: CLIP Verification ---
            crop_pil = PILImage.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
            image_input = self.preprocess(crop_pil).unsqueeze(0).to(self.device)
            
            with torch.no_grad():
                logits_per_image, _ = self.clip_model(image_input, self.target_text)
                probs = logits_per_image.softmax(dim=-1).cpu().numpy()[0]

            target_score = np.max(probs[self.target_indices])
            background_score = np.max(probs[self.bg_indices])

            if target_score > background_score and target_score > self.clip_confidence_threshold:
                self.get_logger().info(f"MATCH FOUND: Confidence {target_score:.2f}")
                
                # --- Step 3: Create Labelled Image ---
                annotated_image = cv_image.copy()
                contour_mask = (mask_np * 255).astype(np.uint8)
                contours, _ = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(annotated_image, contours, -1, (0, 165, 255), 3)
                
                label_str = f"Orange Mallet ({target_score:.2%})"
                cv2.putText(annotated_image, label_str, (x_min, y_min - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(self.save_dir, f"mallet_{timestamp}.jpg")
                cv2.imwrite(filename, annotated_image)
                self.get_logger().info(f"Image saved to: {filename}")
                # --- Step 4: Stop & Send ---
                self.labeled_img_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))
                self.stop_pub.publish(Bool(data=True))
                self.found_and_stopped = True
                break

def main(args=None):
    rclpy.init(args=args)
    node = MalletPhotoCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()