import cv2 
import time 
import csv 
import os
import threading 

# Configurations 
cam1_ID = 2
cam2_ID = 0
FPS = 15
Resolution = (1280, 720)

# Create Folder Structure 
BASE_DIR = r"C:\Users\hp\Downloads\Data_Sessions"
session_name = time.strftime("Session_%Y-%m-%d_%H-%M-%S")
output_dir = os.path.join(BASE_DIR, session_name)
os.makedirs(output_dir, exist_ok=True)

# Initialise cameras 
cap1 = cv2.VideoCapture(cam1_ID, cv2.CAP_DSHOW)
cap2 = cv2.VideoCapture(cam2_ID, cv2.CAP_DSHOW)

# MJPG compression (to prevent USB bandwidth crashing)
cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

cap1.set(cv2.CAP_PROP_FRAME_WIDTH, Resolution[0])
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, Resolution[1])
cap1.set(cv2.CAP_PROP_FPS, FPS)

cap2.set(cv2.CAP_PROP_FRAME_WIDTH, Resolution[0])
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, Resolution[1])
cap2.set(cv2.CAP_PROP_FPS, FPS) 

# Initialise Video Writers 
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
writer1 = cv2.VideoWriter(os.path.join(output_dir, "cam1_raw.mp4"), fourcc, FPS, Resolution)
writer2 = cv2.VideoWriter(os.path.join(output_dir, "cam2_raw.mp4"), fourcc, FPS, Resolution)

# Initialise CSV 
timestamp_path = os.path.join(output_dir, "Timestamps.csv")
csv_file = open(timestamp_path, 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Frame No.", "Timestamp (Unix)"])

# Threading Setup 
frame1, frame2 = None, None  # Hold the frames pulled by the threads 

def grab_cam1(): 
    global frame1
    _, frame1 = cap1.read() 

def grab_cam2(): 
    global frame2
    _, frame2 = cap2.read()

print(f"Recording started. Saving to {output_dir}. Press 'ctrl + C' to stop.")

frame_number = 0

try: 
    while True: 
        # Start both threads simultaneously 
        t1 = threading.Thread(target = grab_cam1) 
        t2 = threading.Thread(target = grab_cam2) 

        t1.start()
        t2.start()

        timestamp = time.time() # Log the EXACT real-world time when both cameras grab frame  

        # Grab frame 
        t1.join()
        t2.join()

        if frame1 is None or frame2 is None: 
            print("Dropped frame detected. Terminating recording.")
            break 

        # Save to files 
        csv_writer.writerow([frame_number, timestamp])
        writer1.write(frame1)
        writer2.write(frame2)

        # Show preview window 
        preview1 = cv2.resize(frame1, (640, 360))
        preview2 = cv2.resize(frame2, (640, 360))
        cv2.imshow("Camera 1", preview1)
        cv2.imshow("Camera 2", preview2)

        if cv2.waitKey(1) & 0xFF == ord('q'): 
            print("Recording stopped by user. (q pressed).")
            break 
        
        frame_number += 1

except KeyboardInterrupt: 
    print("Recording session ended by user.")

# Cleanup 
cap1.release() 
cap2.release() 
writer1.release()
writer2.release()
csv_file.close() 
cv2.destroyAllWindows() 

print(f"Successfully saved {frame_number} frames to {output_dir}")