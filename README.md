# Multi_Object_Tracker_Honours_Thesis
Real-time Multi-Object Tracking system for bird tracking, fuses stereo Logitech web-cameras (using custom YOLOv11m) and Livox Mid-360 LiDAR (using DBSCAN clustering) with an IMM-EKF tracker in MATLAB. Data acquisition in Python/C++. Undergraduate Honours Thesis, UNSW Mechatronics 2026.

## System Overview

### Phase 1: Sensor Rig
A physically constructed field-deployable rig consisting of two 
stereo Logitech C922 HD Pro cameras and a Livox Mid-360 LiDAR, deployed outdoors for 
live data collection sessions.

### Phase 2: Live Data Acquisition
Asynchronous raw data capture split across two languages running 
in parallel:
- **Python** : Opens stereo cameras and captures raw video streams.
- **C++** : Interfaces with the Livox Mid-360 LiDAR to capture 
  raw point cloud data.

### Phase 3: Offline Detection
Processing of raw data from Phase 2:
- **Camera:** Custom YOLOv11m model (trained on field-collected 
  images) detects birds in each camera frame, outputting 
  detections with timestamps to Excel. (File 1)
- **LiDAR:** DBSCAN clustering applied to a filtered point cloud 
  (ROI coordinates identified via raw point cloud visualisation) 
  to detect bird clusters, outputting detections with timestamps 
  and x, y, z coordinates to Excel.(File 2)

### Phase 4: Prediction & Update (MATLAB Object Tracker)
An Interacting Multiple Model Extended Kalman Filter (IMM-EKF) 
running two motion models in parallel:
- **Constant Velocity (CV)**
- **Constant Acceleration (CA)**

Each model is weighted dynamically based on which better explains 
incoming camera measurements. Full track management logic 
handles target birth, confirmation (M-of-N) and death.

## Repository Structure
```text
Multi_Object_Tracker_Honours_Thesis/
│
├── Run_LiDAR/
│   └── main.cpp
│       Live LiDAR data acquisition, interfaces with Livox
│       Mid-360 to capture and save the raw point cloud file from the data session.
│
├── Camera_Data_Collection (Python)/
│   ├── Stereo Video capture script.
│   ├── YOLOv11m detection pipeline: processes raw video,
│   │   outputs per-camera detection Excel files with timestamps.
│   ├── Raw LiDAR Point Cloud Plot: used to identify
│   │   ROI filter coordinates (x, y, z) of the 'flight corridor' of the data sessions.
│   ├── LiDAR detection pipeline: Applies the above ROI filter to ONLY get the corridor/area where birds traversed and
│   │   outputs detection Excel files with timestamps. 
│   └── Camera/LiDAR calibration scripts: intrinsic and
│       extrinsic calibration
│
├── MOT_Deployment_Primary/ ← Primary tracker implementation
│   Modularised MATLAB IMM-EKF object tracker.
│   Inputs: camera detection Excel files.
│   Uses LiDAR detections for track birth only (not updates)
│   Outputs: track plots of predicted trajectories.
│
├── MOT_Deployment_Secondary/
│   Same IMM-EKF tracker (earlier version) but ALSO incorporates
│   LiDAR detections in the update step as well as track birth.
│   Experimental, not used in final thesis results.
│
│── MOT_Deployment_Simulation/
│   Simulation environment developed during Thesis A & B.
│   Two bird trajectories (manouevering & constant velocity) simulated and used to validate the │   │   tracker design before field deployment.
│
└── README.md
```

---

## Tracker Pipeline Architecture

The primary tracker implementation is  modularised into
sequential processing stages for maintainability, debugging and
scalability.

```text
main_hardware.m
│
├── 1. Load Hardware Data
│   └── Load timestamped camera and LiDAR detections.
│
├── 2. Setup Real Cameras
│   └── Construct stereo/camera projection matrices using
│       intrinsic/extrinsic calibration.
│
├── 3. Run Tracker
│   └── IMM-EKF prediction/update, data association,
│       track birth, confirmation and coasting.
│
├── 4. Track Scoring
│   └── Confidence-based validation of generated tracks.
│
├── 5. Trajectory Visualisation
│   └── Plot predicted trajectories against LiDAR ground truth.
│
├── 6. Quantitative Analysis
│   └── RMSE/error analysis of tracked objects.
│
├── 7. Detection Distribution Analysis
│   └── Statistical analysis of measurement detections.
│
└── 8. Mode Probability Visualisation
    └── IMM mode probability evolution across time.
```

---

## Tech Stack

| Component          | Language | Libraries                     |
|--------------------|----------|-------------------------------|
| LiDAR Acquisition  | C++      | Livox SDK                     |
| Camera Acquisition | Python   | OpenCV                        |
| Object Detection   | Python   | Custom YOLOv11m (Ultralytics) |
| LiDAR Clustering   | Python   | DBSCAN (scikit-learn)         |
| Camera Calibration | Python   | OpenCV                        |
| MOT Tracker        | MATLAB   | Custom IMM-EKF                |

---

## Features

- **IMM-EKF:** Interacting Multiple Model Extended Kalman Filter 
  — maintains parallel CV and CA motion models, weighted by 
  measurement likelihood
- **Track Management:** M-of-N confirmation, Mahalanobis Distance calculation, Hungarian Assignment logic for track birth,
  coasting and death. 
- **Sensor Fusion:** LiDAR detections used to initialise new tracks 
  (birth), camera detections used for EKF updates.
- **Asynchronous Streams:** Python and C++ pipelines run 
  independently, timestamps used to align detections in the 
  MATLAB tracker

---

## Author

**Dulmith Pitigalage**  
BEng (Honours) Mechatronics - UNSW Sydney, 2026  
LinkedIn: linkedin.com/in/dulmith-pitigalage-046baa264
