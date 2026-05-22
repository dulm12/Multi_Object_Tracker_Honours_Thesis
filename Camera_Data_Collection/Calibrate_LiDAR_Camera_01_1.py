from turtle import end_fill
import cv2, csv, os
import numpy as np
import open3d as o3d

# configuration 
SESSION_FOLDER = r"C:\Users\hp\Downloads\Data_Sessions\Session_2026-03-26_16-32-06"
image_dir = os.path.join(SESSION_FOLDER, "calib_images")
pcd_dir = os.path.join(SESSION_FOLDER, "calib_pointclouds")
os.makedirs(image_dir, exist_ok=True)
os.makedirs(pcd_dir, exist_ok=True)

# Helper Function: save and preview the video 
def save_pair(frame, frame_number, timestamps_dict, lidar_data, pair_count): 
    target_unix_time = timestamps_dict.get(frame_number) 

    if target_unix_time is None: 
        print(f"Error: Frame {frame_number} missing from CSV.")
        return pair_count # return without incrementing 

    # 1. save the image 
    image_name = f"image_{pair_count:02d}.png"
    cv2.imwrite(os.path.join(image_dir, image_name), frame) 

    # 2. slice 1-second of lidar data around this microsecond 
    time_mask = (lidar_data['timestamp'] >= target_unix_time - 0.5) & (lidar_data['timestamp'] <= target_unix_time + 0.5)
    lidar_pt_slice = lidar_data[time_mask] 

    # 3. apply ROI filter 
    # ROI filter (metres), the flight corridor 
    ROI_X_MIN, ROI_X_MAX = 0.0, 10.0   
    ROI_Y_MIN, ROI_Y_MAX = -2.0, 2.0  
    ROI_Z_MIN, ROI_Z_MAX = 0, 7.0   

    ROI_mask = (
        (lidar_pt_slice['x'] >= ROI_X_MIN) & (lidar_pt_slice['x'] <= ROI_X_MAX) & 
        (lidar_pt_slice['y'] >= ROI_Y_MIN) & (lidar_pt_slice['y'] <= ROI_Y_MAX) & 
        (lidar_pt_slice['z'] >= ROI_Z_MIN) & (lidar_pt_slice['z'] <= ROI_Z_MAX)
    )

    filtered_slice = lidar_pt_slice[ROI_mask]

    # 4. check if anything survived the ROI filter 
    if len(filtered_slice) < 10: 
        print(f"Warning: Only {len(filtered_slice)} points after applying ROI filter.")

    # 5. Save filtered point cloud to pcd
    xyz = np.column_stack((filtered_slice['x'], filtered_slice['y'], filtered_slice['z']))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    o3d.io.write_point_cloud(os.path.join(pcd_dir, f"cloud_{pair_count:02d}.pcd"), pcd)

    print(f"Success: Saved pair {pair_count:02d} at Frame {frame_number}")

    # 6. Preview the point cloud 
    print(f"Previewing point cloud for pair {pair_count}. Close the 3D window to continue.")
    # paint it dark so it's easy to see  
    pcd.paint_uniform_color([0.3, 0.3, 0.3])
    o3d.visualization.draw_geometries([pcd], window_name = f"Pair {pair_count}, verify checkerboard")

    return pair_count + 1 # increment and return 

def seek_to_frame(cap, frame_number): 
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_number = max(0, min(frame_number, total_frames - 1))
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)

# 1. load the master clock 
print("Loading Timestamps.")
timestamps_dict = {}
with open(os.path.join(SESSION_FOLDER, "Timestamps.csv"), 'r') as f:
    reader = csv.reader(f)
    next(reader)
    for row in reader:
        timestamps_dict[int(row[0])] = float(row[1])

# 2. load lidar data 
print("Loading LiDAR binary data.")
point_format = np.dtype([('timestamp', '<f8'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
lidar_data = np.fromfile(os.path.join(SESSION_FOLDER, "lidar_raw_2026_03_26--16_32_18.bin"), dtype=point_format)

# 3. video player
cap = cv2.VideoCapture(os.path.join(SESSION_FOLDER, "cam1_raw.mp4"))
FPS = cap.get(cv2.CAP_PROP_FPS)
SKIP_seconds = 5
SKIP_frames = int(FPS * SKIP_seconds) 

frame_number = 0
pair_count = 0
speed_multiplier = 1.5 
paused = False

print("\n" + "="*50)
print("CONTROLS:")
print(" [s] : Save calibration pair")
print(" [p] : Pause / Unpause")
print(" [f] : Toggle speed (0.5x / 1.0x / 1.5x)")
print(" [a] : Rewind 5 seconds")
print(" [d] : Skip forward 5 seconds")
print(" [q] : Quit")
print("="*50 + "\n")

while True:

    if not paused:
        ret, frame = cap.read()
        if not ret: 
            print("End of video reached.")
            break 

        frame_number = int(cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1
        current_time = frame_number / FPS 

        display_frame = frame.copy() 
        cv2.putText(display_frame, 
                    f"Time: {current_time:.1f}s | Frame: {frame_number} | Speed: {speed_multiplier} x | Saved: {pair_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Calibration Extractor", display_frame)

        # delay depends on speed 
        delay = int(33 / speed_multiplier)
        key = cv2.waitKey(delay) & 0xFF 
    else: 
        # Paused 
        current_time = frame_number / FPS 
        display_frame = frame.copy()
        cv2.putText(display_frame, 
                    f"Time: {current_time:.1f}s | Frame: {frame_number} | Speed: {speed_multiplier} x | Saved: {pair_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Calibration Extractor", display_frame)
        key = cv2.waitKey(0) & 0xFF 

    # Key Handling 

    if key == ord('q'):
        break

    elif key == ord('s'):
        # Save while video is actively playing
        pair_count = save_pair(frame, frame_number, timestamps_dict, lidar_data, pair_count)

    elif key == ord('p'):
        paused = not paused 
        if paused: 
            print(f"Paused at frame {frame_number}, time {frame_number / FPS:.1f}s")
        else: 
            print("Resuming")
        print("Paused. Press 's' to save, 'p' to resume, or 'q' to quit.")

    elif key == ord("f"):

        if speed_multiplier == 1.5: 
            speed_multiplier = 0.25
            print("Speed: 0.25x, slow.")

        elif speed_multiplier == 0.25: 
            speed_multiplier = 1.5 
            print("Speed: 1.5x, fast.")
    
    elif key == ord("a"): 
        # go back 5s 
        current_accumulated_frames = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        seek_to_frame(cap, current_accumulated_frames - SKIP_frames)
        ret, frame = cap.read()
        frame_number = int(cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1 
        print(f"Went back 5s to frame {frame_number}, time {frame_number/FPS:.1f}s")

    elif key == ord('d'): 
         # go forward 5s 
        current_accumulated_frames = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        seek_to_frame(cap, current_accumulated_frames + SKIP_frames)
        ret, frame = cap.read()
        frame_number = int(cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1 
        print(f"Skipped forward 5s to frame {frame_number}, time {frame_number/FPS:.1f}s")

cap.release()
cv2.destroyAllWindows()
print(f"Extraction complete. Saved {pair_count} calibration pairs.")