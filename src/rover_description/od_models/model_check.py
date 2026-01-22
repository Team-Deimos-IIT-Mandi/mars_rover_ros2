import cv2
import torch
import numpy as np
import clip
import os
from PIL import Image as PILImage
from ultralytics import FastSAM
from datetime import datetime

# --- CONFIGURATION ---
WEBCAM_INDEX = 0 
FASTSAM_WEIGHTS = "FastSAM-s.pt"
CLIP_WEIGHTS = "ViT-B-32.pt"  # Path to local .pt or use "ViT-B-32"
CONFIDENCE_THRESHOLD = 0.65   # CLIP matching threshold
SAVE_DIR = "captured_bottles"
os.makedirs(SAVE_DIR, exist_ok=True)

class BottleCaptureFastSAM:
    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = FastSAM(FASTSAM_WEIGHTS)
        self.clip_model, self.preprocess = clip.load(CLIP_WEIGHTS, device=self.device, jit=False)

        # Labels to help CLIP verify the "Bottle"
        self.target_labels = [

"a wide-mouthed plastic bottle",

"a 1 liter water bottle in person's hand", # Change color if you changed the URDF color

"a large container with a grey cap",

"a tall plastic jar for liquids",

"a translucent blue bottle"

]
        
        self.background_labels = [

"a bunked bed",
"a table",
"room walls",
"a person on a chair",
"a face of a person",
"lamp on the walls",
"a dark room"
]


        self.bg_labels = ["rocks", "dirt", "background"]
        self.labels = self.target_labels + self.bg_labels
        self.target_idx = list(range(len(self.target_labels)))
        self.text_tokens = clip.tokenize(self.labels).to(self.device)

    def is_bottle_shape(self, h, w):
        if w == 0: return False
        aspect_ratio = h / w
        return 1.8 < aspect_ratio < 3.5

    def run(self):
        cap = cv2.VideoCapture(WEBCAM_INDEX, cv2.CAP_V4L2)
        print("FastSAM + CLIP Active. Waiting for bottle...")

        while True:
            ret, frame = cap.read()
            if not ret: break

            # 1. FastSAM Segmentation
            results = self.model(frame, device=self.device, retina_masks=True, imgsz=640, conf=0.5, verbose=False)
            masks = results[0].masks
            
            if masks is not None:
                for mask in masks.data:
                    mask_np = mask.cpu().numpy().astype(bool)
                    coords = np.argwhere(mask_np)
                    if len(coords) == 0: continue
                    
                    y_min, x_min = coords.min(axis=0)
                    y_max, x_max = coords.max(axis=0)
                    h, w = y_max - y_min, x_max - x_min
                    crop = frame[y_min:y_max, x_min:x_max]
                    
                    if crop.size == 0 or not self.is_bottle_shape(h, w):
                        continue

                    # 2. CLIP Verification
                    crop_pil = PILImage.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
                    image_input = self.preprocess(crop_pil).unsqueeze(0).to(self.device)
                    
                    with torch.no_grad():
                        logits_per_image, _ = self.clip_model(image_input, self.text_tokens)
                        probs = logits_per_image.softmax(dim=-1).cpu().numpy()[0]

                    target_score = np.max(probs[self.target_idx])

                    # 3. Save if match found
                    if target_score > CONFIDENCE_THRESHOLD:
                        annotated = frame.copy()
                        # Draw Contour
                        contour_mask = (mask_np * 255).astype(np.uint8)
                        contours, _ = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(annotated, contours, -1, (0, 165, 255), 3)
                        
                        # Add Label
                        label_str = f"Bottle: {target_score:.2%}"
                        cv2.putText(annotated, label_str, (x_min, y_min - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        filename = os.path.join(SAVE_DIR, f"bottle_labeled_{timestamp}.jpg")
                        cv2.imwrite(filename, annotated)
                        
                        print(f"SUCCESS: Match found ({target_score:.2f}). Image saved to {filename}")
                        cap.release()
                        cv2.destroyAllWindows()
                        return

            cv2.imshow("FastSAM+CLIP Search", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = BottleCaptureFastSAM()
    detector.run()