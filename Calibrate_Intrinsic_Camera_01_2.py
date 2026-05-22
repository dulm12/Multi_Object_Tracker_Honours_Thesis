import cv2
import os

# Configuration
SESSION_FOLDER = r"C:\Users\hp\Downloads\Data_Sessions\Session_2026-03-27_11-21-30"
VIDEO_FILENAME = "cam2_raw.mp4" 

video_path = os.path.join(SESSION_FOLDER, VIDEO_FILENAME)
# Create subfolder, "intrinsic_images_cam1"
output_dir = os.path.join(SESSION_FOLDER, f"intrinsic_images_{VIDEO_FILENAME[:4]}_SECOND_TRY") 
os.makedirs(output_dir, exist_ok=True)

# video player (get FPS) 
cap = cv2.VideoCapture(video_path) 
FPS = cap.get(cv2.CAP_PROP_FPS)
SKIP_seconds = 5 
SKIP_frames = int(FPS * SKIP_seconds) 

img_count = 0
speed_multiplier = 1.5 

# helper function 
def save_image(frame, count):

    img_name = f"intrinsic_{count:02d}.png"
    save_path = os.path.join(output_dir, img_name)
    cv2.imwrite(save_path, frame)
    print(f"SUCCESS: Saved {img_name}")

    return count + 1

def get_current_frame(cap): 
    return int(cap.get(cv2.CAP_PROP_POS_FRAMES))

def seek_to_frame(cap, frame_number): 
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_number = max(0, min(frame_number, total_frames - 1))
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)


print("\n" + "="*50)
print(f"Getting intrinsics for {VIDEO_FILENAME}")
print("Controls:")
print(" [p] : Pause / Unpause video")
print(" [s] : Save calibration frame")
print(" [f] : Toggle 0.5x / 1.0x speed")
print(" [->]: Go back 5 seconds")
print(" [<-]: Skip forward 5 seconds")
print(" [q] : Quit the video player window")
print("="*50 + "\n")

paused = False

while True:
    if not paused: 
        ret, frame = cap.read()
        if not ret:
            print("End of video reached.")
            break

        # Display current frame number and speed
        current_frame = get_current_frame(cap)
        current_time = current_frame / FPS
        display_frame = frame.copy()
        cv2.putText(display_frame, 
                    f"Time: {current_time:.1f}s | Frame: {current_frame} | Speed: {speed_multiplier}x | Saved: {img_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Intrinsic Calibration Extractor", display_frame)

        # Delay depends on speed: normal=33ms, half speed=66ms
        delay = int(33 / speed_multiplier)
        key = cv2.waitKey(delay) & 0xFF

    else:
        # Paused, show frozen frame with overlay
        display_frame = frame.copy()
        current_frame = get_current_frame(cap)
        current_time = current_frame / FPS
        cv2.putText(display_frame,
                    f"PAUSED | Time: {current_time:.1f}s | Speed: {speed_multiplier}x | Saved: {img_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow("Intrinsic Calibration Extractor", display_frame)
        key = cv2.waitKey(0) & 0xFF

    # Key Handling 
    if key == ord('q'):
        break
        
    elif key == ord('s'):
        # Save while video is actively playing
        img_count = save_image(frame, img_count)
        print(f"Saved at time: {current_frame / FPS:.1f}s")

    elif key == ord('p'):
        paused = not paused
        if paused:
            print(f"Paused at frame {get_current_frame(cap)}, time {get_current_frame(cap)/FPS:.1f}s")
        else:
            print("Resuming.")

    elif key == ord('f'):
        # Toggle speed
        if speed_multiplier == 1.5:
            speed_multiplier = 0.25
            print("Speed: 0.25x (quarter speed)")
        else:
            speed_multiplier = 1.5
            print("Speed: 1.5x (run through)")

    elif key == 97:  # 'A' key to go back 5s
        current = get_current_frame(cap)
        seek_to_frame(cap, current - SKIP_frames)
        ret, frame = cap.read()  # Read  frame at new position
        print(f"Went back 5s to frame {get_current_frame(cap)}, time {get_current_frame(cap)/FPS:.1f}s")

    elif key == 100:  # 'D' key to go foward 5s
        current = get_current_frame(cap)
        seek_to_frame(cap, current + SKIP_frames)
        ret, frame = cap.read()
        print(f"Skipped forward 5s to frame {get_current_frame(cap)}, time {get_current_frame(cap)/FPS:.1f}s")


cap.release()
cv2.destroyAllWindows()
print(f"Extraction complete. Saved {img_count} images to {output_dir}")