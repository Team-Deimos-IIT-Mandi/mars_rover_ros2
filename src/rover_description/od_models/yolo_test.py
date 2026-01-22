import cv2
import os
from ultralytics import YOLO
from datetime import datetime

# --- CONFIGURATION ---
WEBCAM_INDEX = 2 
MODEL_PATH = "yolov8n-seg.pt" 
CONF_THRESHOLD = 0.6
SAVE_DIR = "captured_bottles"
os.makedirs(SAVE_DIR, exist_ok=True)
# ---------------------

def main():
    model = YOLO(MODEL_PATH)
    
    cap = cv2.VideoCapture(WEBCAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("ERROR: Could not open webcam.")
        return

    print(f"Detecting... Script will save the Labeled Image and exit on first match.")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 1. Run detection for Bottles (Class 39)
        results = model.predict(frame, conf=CONF_THRESHOLD, classes=[39], verbose=False)

        # 2. Check if a bottle was found
        if len(results[0].boxes) > 0:
            # Generate the labeled image (bounding boxes + labels + confidence)
            # Use results[0].plot() to get the frame with all annotations
            annotated_frame = results[0].plot()

            # Generate Filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(SAVE_DIR, f"bottle_labeled_{timestamp}.jpg")
            
            # 3. SAVE THE FULL LABELED IMAGE
            cv2.imwrite(filename, annotated_frame)
            print(f"SUCCESS: Labeled image saved to {filename}. Exiting...")
            break 

        # Display live feed
        cv2.imshow("Bottle Hunter - Labeled Save Mode", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("Program terminated.")

if __name__ == "__main__":
    main()