import cv2
import os

# Configuration 
SESSION_FOLDER = r"C:\Users\z5406189\Thesis_C\Data_Sessions\Session_2026-04-08_17-21-25"

vid1_path = os.path.join(SESSION_FOLDER, "cam1_raw.mp4")
vid2_path = os.path.join(SESSION_FOLDER, "cam2_raw.mp4")

# subfolders for extrinsic images for each cam 
out1_dir = os.path.join(SESSION_FOLDER, "extrinsic_images_cam1") 
out2_dir = os.path.join(SESSION_FOLDER, "extrinsic_images_cam2") 
os.makedirs(out1_dir, exist_ok=True)
os.makedirs(out2_dir, exist_ok=True)

# video players 
cap1 = cv2.VideoCapture(vid1_path) 
cap2 = cv2.VideoCapture(vid2_path) 

# Assume both cameras were recorded at the same FPS
FPS = cap1.get(cv2.CAP_PROP_FPS)
SKIP_seconds = 5 
SKIP_frames = int(FPS * SKIP_seconds) 

pair_count = 0
speed_multiplier = 1.5 

# save images from cam1 and cam2
def save_pair(f1, f2, count):
    img1_name = f"extrinsic_cam1_{count:02d}.png"
    img2_name = f"extrinsic_cam2_{count:02d}.png"
    
    cv2.imwrite(os.path.join(out1_dir, img1_name), f1)
    cv2.imwrite(os.path.join(out2_dir, img2_name), f2)
    
    print(f"SUCCESS: Saved pair {count:02d}")
    return count + 1

def get_current_frame(cap): 
    return int(cap.get(cv2.CAP_PROP_POS_FRAMES))

def seek_to_frame(cap1, cap2, frame_number): 
    total_frames = int(cap1.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_number = max(0, min(frame_number, total_frames - 1))
    
    # Set both cameras to the exact same frame
    cap1.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
    cap2.set(cv2.CAP_PROP_POS_FRAMES, frame_number)


print("\n" + "="*50)
print("Getting Extrinsic Stereo Pairs")
print("Controls:")
print(" [p] : Pause / Unpause video")
print("[s] : Save calibration pair")
print(" [f] : Toggle 0.25x / 1.5x speed")
print(" [a] : Go back 5 seconds")
print(" [d] : Skip forward 5 seconds")
print(" [q] : Quit the video player windows")
print("="*50 + "\n")

paused = False

while True:
    if not paused: 
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        
        if not ret1 or not ret2:
            print("End of one or both videos reached.")
            break

        # Display current frame number and speed
        current_frame = get_current_frame(cap1)
        current_time = current_frame / FPS
        
        display1 = frame1.copy()
        display2 = frame2.copy()
        
        info_text = f"Time: {current_time:.1f}s | Frame: {current_frame} | Speed: {speed_multiplier}x | Pairs: {pair_count}"
        
        cv2.putText(display1, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display2, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # show displays 
        cv2.imshow("Camera 1 (Extrinsic)", display1)
        cv2.imshow("Camera 2 (Extrinsic)", display2)

        # Delay depends on speed: normal=33ms, slow speed=132ms
        delay = int(33 / speed_multiplier)
        key = cv2.waitKey(delay) & 0xFF

    else:
        # Paused, show frozen frame with overlay
        display1 = frame1.copy()
        display2 = frame2.copy()
        
        current_frame = get_current_frame(cap1)
        current_time = current_frame / FPS
        pause_text = f"PAUSED | Time: {current_time:.1f}s | Speed: {speed_multiplier}x | Pairs: {pair_count}"
        
        cv2.putText(display1, pause_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(display2, pause_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.imshow("Camera 1 (Extrinsic)", display1)
        cv2.imshow("Camera 2 (Extrinsic)", display2)
        
        key = cv2.waitKey(0) & 0xFF

    # key handling
    if key == ord('q'):
        break
        
    elif key == ord('s'):
        # Save while video is actively playing or paused
        pair_count = save_pair(frame1, frame2, pair_count)
        print(f"Saved at time: {current_frame / FPS:.1f}s")

    elif key == ord('p'):
        paused = not paused
        if paused:
            print(f"Paused at frame {get_current_frame(cap1)}, time {get_current_frame(cap1)/FPS:.1f}s")
        else:
            print("Resuming.")

    elif key == ord('f'):
        # Toggle speed
        if speed_multiplier == 1.5:
            speed_multiplier = 0.25
            print("Speed: 0.25x (quarter speed)")
        else:
            speed_multiplier = 1.5
            print("Speed: 1.5x (fast speed)")

    elif key == 97 or key == ord('a'):  # 'A'  to go back 5s
        current = get_current_frame(cap1)
        seek_to_frame(cap1, cap2, current - SKIP_frames)
        # Read the new frames immediately so the paused screen updates
        ret1, frame1 = cap1.read()  
        ret2, frame2 = cap2.read()  
        print(f"Went back 5s to frame {get_current_frame(cap1)}, time {get_current_frame(cap1)/FPS:.1f}s")

    elif key == 100 or key == ord('d'):  # 'D'  to go foward 5s
        current = get_current_frame(cap1)
        seek_to_frame(cap1, cap2, current + SKIP_frames)
        # Read the new frames immediately so the paused screen updates
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        print(f"Skipped forward 5s to frame {get_current_frame(cap1)}, time {get_current_frame(cap1)/FPS:.1f}s")

cap1.release()
cap2.release()
cv2.destroyAllWindows()
print(f"Extraction complete. Saved {pair_count} stereo pairs.")