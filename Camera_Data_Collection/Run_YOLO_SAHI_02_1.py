import cv2 
import csv
import os 
import torch 
from sahi import AutoDetectionModel
from sahi.predict import get_sliced_prediction
from ultralytics import YOLO 

# is gpu being used? 
print(f"CUDA Available: {torch.cuda.is_available()}")
if not torch.cuda.is_available():
    print("WARNING: PyTorch is using your CPU! SAHI will be incredibly slow.")

# Configuration 
Session_folder =   r"C:\Users\z5406189\Thesis_C\Data_Sessions\Session_2026-04-01_16-35-45" 
YOLO_MODEL_PATH = r"C:\Users\z5406189\Thesis_C\Camera_Data_Collection\runs\detect\train\weights\yolo11m_custom.pt"
COCO_Bird_Index = 0 # When using custom mode 
Confidence_Threshold = 0.6

Show_PREVIEW = False 
def run_YOLO_on_video(video_path, timestamps_dict, output_csv_path, camera_ID, detection_model): 
    global Show_PREVIEW
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
                
                # Convert OpenCV's BGR format to standad RGB 
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 

                # SAHI Inference 
                # chop the video frame into multiple 640 x 640 patches 
                # after YOLO looks at every slice in the frame, merge the results 
                # results -> object containing list of every bird found across all slices 
                results = get_sliced_prediction (
                    frame_rgb, 
                    detection_model, 
                    slice_height = 640, 
                    slice_width  = 640, 
                    overlap_height_ratio = 0.15, # make the patches overlap 20%
                    overlap_width_ratio  = 0.15,  # if a bird is sitting on the edge of a 'cut', it is not missed
                    perform_standard_pred = False, # look at the entire image at once to not miss detections 
                    verbose = 0
                )
                
                # Add preview 
                preview = frame.copy()
                for box in results.object_prediction_list:
                    if box.category.id == COCO_Bird_Index and box.score.value > Confidence_Threshold:
                        
                        bounding_box = box.bbox.to_xyxy()
                        u = (bounding_box[0] + bounding_box[2]) / 2.0 
                        v = (bounding_box[1] + bounding_box[3]) / 2.0 

                        writer.writerow([current_timestamp, camera_ID, u, v])
                        
                        if Show_PREVIEW:
                            confidence = box.score.value
                            cv2.rectangle(preview, 
                                        (int(bounding_box[0]), int(bounding_box[1])), 
                                        (int(bounding_box[2]), int(bounding_box[3])), 
                                        (0, 255, 0), 2)
                            cv2.putText(preview, f"bird {confidence:.2f}", 
                                        (int(bounding_box[0]), int(bounding_box[1]) - 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                if Show_PREVIEW: 
                    cv2.imshow(f"Camera {camera_ID}", cv2.resize(preview, (640, 360)))
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        cv2.destroyAllWindows()
                        Show_PREVIEW = False
                # end preview 

            frame_number += 1 # Outside the box loop, inside the while loop, advance counter once per picture

            # Print a progress heartbeat so you know it hasn't crashed
            if frame_number % 100 == 0:
                print(f"Processed {frame_number} frames.", end="\r")

    cap.release()
    print(f"Ran YOLO on Camera {camera_ID}. Saved to {os.path.basename(output_csv_path)}")


if __name__ == "__main__": 

    # Load model once 
    detection_model = AutoDetectionModel.from_pretrained(
        model_type           = 'ultralytics',
        model_path           = YOLO_MODEL_PATH,
        confidence_threshold = Confidence_Threshold,
        device               ="cuda:0", 
    )

    # Load the timestamps into a dictionary of frame number : timestamp
    # Dictionary entries can be looked up automatically, O(1) time.
    timestamps_csv = os.path.join(Session_folder, "Timestamps.csv")
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
        video_path      = os.path.join(Session_folder, "cam1_raw.mp4"), 
        timestamps_dict = timestamps_dict, 
        output_csv_path = os.path.join(Session_folder, "Camera1_detections_custom_model_SAHI.csv"),
        camera_ID       = 1, 
        detection_model = detection_model
    )

    # Run YOLO on Camera 2 
    run_YOLO_on_video (
        video_path      = os.path.join(Session_folder, "cam2_raw.mp4"), 
        timestamps_dict = timestamps_dict, 
        output_csv_path = os.path.join(Session_folder, "Camera2_detections_custom_model_SAHI.csv"),
        camera_ID       = 2, 
        detection_model = detection_model
    )

    print("YOLO model has run on both cameras. Cam1 and Cam2 detection files complete")