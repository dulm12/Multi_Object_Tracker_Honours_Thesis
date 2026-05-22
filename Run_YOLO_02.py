import cv2 
import csv
import os 
from ultralytics import YOLO 

# Configuration 
Session_folder = r"C:\Users\Dulmith Pitigalage\Thesis_C\Data_Sessions\Session_2026-04-01_16-35-45"
YOLO_MODEL_PATH = r"C:\Users\Dulmith Pitigalage\Thesis_C\Camera_Data_Collection\Custom_Dataset_Two_Sessions_Combined\runs\detect\train\weights\YOLO11m_custom_two_sessions.pt"
COCO_Bird_Index = 0 # Custom Model has 0 as the bird index (only class it knows)
Confidence_Threshold = 0.40

def run_YOLO_on_video(video_path, timestamps_dict, output_csv_path, camera_ID): 
    model = YOLO(YOLO_MODEL_PATH) 
    cap = cv2.VideoCapture(video_path)

    with open(output_csv_path, 'w', newline = '') as out_f: 
        writer = csv.writer(out_f)
        writer.writerow(['Timestamp', 'Camera ID', 'U (pixel)', 'V (pixel)'])

        frame_number = 0
        print(f"Processing {os.path.basename(video_path)}.")

        # Read the video frame by frame. Ask dict. what real-world time was. Feed frame to YOLO.
        while True:

            ret, frame = cap.read()

            if not ret:
                break # End of Video 
            
            # Get exact computer timestamp for the specific frame 
            current_timestamp = timestamps_dict.get(frame_number) # .get() returns None if frame_no. missing from CSV file. 

            if current_timestamp is not None: 
                # Enhance contrast for birds against dark backgrounds
                # lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
                # l, a, b = cv2.split(lab)
                # clahe = cv2.createCLAHE(clipLimit=6.0, tileGridSize=(8, 8))
                # l = clahe.apply(l)
                # frame_enhanced = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)

                # Run YOLO 
                results = model(frame, verbose=False)

                # Add preview 
                preview = frame.copy() 
                for r in results: 
                    for box in r.boxes: 
                        if int(box.cls.item()) == 0: 
                            x1, y1, x2, y2 = box.xyxy[0].tolist() 
                            confidence = box.conf.item() 
                            cv2.rectangle(preview, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(preview, f"bird {confidence:.2f}", (int(x1), int(y1) - 5), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.imshow(f"Camera {camera_ID}", cv2.resize(preview, (640, 360))) 
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    break

                # 'results' just contain the 1 image
                for r in results: 
                    boxes = r.boxes # bounding boxes in image
                    for box in boxes: 
                        # in each box what has the model identified with what confidence 
                        class_ID = int(box.cls.item())
                        confidence = box.conf.item()

                        if class_ID == COCO_Bird_Index and confidence > Confidence_Threshold:
                            xywh = box.xywh[0] # IMM-EKF wants teh centroid to find birds trajectory 
                            x_center = xywh[0].item()
                            y_center = xywh[1].item() 

                            u = x_center 
                            v = y_center 

                            # Log to CSV 
                            writer.writerow([current_timestamp, camera_ID, u, v])

            frame_number += 1 # Outside the box loop, inside the while loop, advance counter once per picture

    cap.release()
    cv2.destroyAllWindows()
    print(f"Ran YOLO on Camera {camera_ID}. Saved to {os.path.basename(output_csv_path)}")


if __name__ == "__main__": 

    # Load the timestamps into a dictionary of frame number : timestamp
    # Dictionary entries can be looked up automatically, O(1) time.
    timestamps_csv = os.path.join(Session_folder, "Timestamps_interpolated.csv")
    timestamps_dict = {}
    
    # 'r' for read mode 
    with open(timestamps_csv, 'r') as f: 
        reader = csv.reader(f) # reader formats 'f' nicely 
        next(reader) # Skip header row 
        for row in reader: 
            # Example row: ['142', '1772834315.12']
            timestamps_dict[int(row[0])] = float(row[1])
    
    # Run YOLO on Camera 1
    run_YOLO_on_video (
        video_path = os.path.join(Session_folder, "cam1_raw.mp4"), 
        timestamps_dict = timestamps_dict, 
        output_csv_path = os.path.join(Session_folder, "Camera1_detections_custom_model_fourth_run.csv"),
        camera_ID = 1
    )

    # Run YOLO on Camera 2 
    run_YOLO_on_video (
        video_path = os.path.join(Session_folder, "cam2_raw.mp4"), 
        timestamps_dict = timestamps_dict, 
        output_csv_path = os.path.join(Session_folder, "Camera2_detections_custom_model_fourth_run.csv"),
        camera_ID = 2
    )

    print("YOLO model has run on both cameras. Cam1 and Cam2 detection files complete")

                    
