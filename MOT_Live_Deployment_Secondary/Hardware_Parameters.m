function params = Hardware_Parameters()

    %% Real Data File Paths 
    params.data_folder = 'C:\Users\Dulmith Pitigalage\Thesis_C\Data_Sessions\Session_2026-03-17_17-11-01';
    params.cam1_csv    = fullfile(params.data_folder, 'Camera1_detections_custom_model_second_run.csv'); 
    params.cam2_csv    = fullfile(params.data_folder, 'Camera2_detections_custom_model_second_run.csv');
    params.LIDAR_csv   = fullfile(params.data_folder, 'LiDAR_detections.csv');
    
    %% Camera Setup 
  
    % Camera intrinsics (Calibration) 
    cam1_intrinsics = load('Camera_Calibration\intrinsic_calib_cam1_29_03_2026.mat');
    cam2_intrinsics = load('Camera_Calibration\intrinsic_calib_cam2_29_03_2026.mat'); 

    params.cam1.K = cam1_intrinsics.calibrationSession.CameraParameters.Intrinsics.K; 
    params.cam2.K = cam2_intrinsics.calibrationSession.CameraParameters.Intrinsics.K; 

    % Camera extrinsics (Calibration) 
    cam1_cam2_extrinsics = load('Camera_Calibration\extrinsic_calib_session_08_04_2026.mat'); 

    % LiDAR & Cam1 Calibration 
    LIDAR_cam1_calib = load('Camera_Calibration\lidar_camera1_calibration.mat'); 

    % 1. Map LiDAR (GCF) frame (X=Fwd, Y=Left, Z=Up) to Cam 1 (Z=Fwd, X=Right, Y=Down)
    % Need this in EKF (3D GCF pt -> Multiply R -> Camera 1 optical frame
    % -> Perspective Division -> Multiply by K -> (u, v) 
    params.cam1.R = [  0, -1,  0; 
                       0,  0, -1; 
                       1,  0,  0 ]; 

    params.cam1.t = [0; 0; 0]; 

    R_GCF_to_Cam1 = params.cam1.R; 
    R_Cam1_to_GCF = R_GCF_to_Cam1';

    t_GCF_to_Cam1 = params.cam1.t; 

    % 2. Extract rotation and translation from cam1 to cam2 
    % Translation vector is 1 x 3 natively and in mm (Cam 2's position as
    % seen from Cam1, in cam1's coordinates)
    t_Cam1_to_Cam2 = cam1_cam2_extrinsics.calibrationSession.CameraParameters.PoseCamera2.Translation' / 1000;
    % Since translation is cam1->cam2, rotation is also cam1->cam2
    R_Cam1_to_Cam2 = cam1_cam2_extrinsics.calibrationSession.CameraParameters.PoseCamera2.R;

    % 3. Calculate Cam 2's relationship to the Global Frame
    % To project GCF into Cam2, two steps are needed: 
    % a. GCF to Cam1 AND b. Cam1 to Cam2 

    % P_cam2 = R_cam1_to_cam2 * (R_GCF_to_Cam1 * P_GCF) + t_cam1_to_cam2
    params.cam2.R = R_Cam1_to_Cam2 * R_GCF_to_Cam1;
    % Take first offset, GCF to cam1, re-express it in cam2's language
    % using R_cam1_to_cam2. Then, add the second offset, t_cam1_to_cam2.
    params.cam2.t = R_Cam1_to_Cam2 * t_GCF_to_Cam1 + t_Cam1_to_Cam2; 

    % Camera Positions in GCF 
    % P_cam2_LCF = R_GCF_to_Cam2 * Cam2_Point_GCF + t_GCF_to_Cam2
    % [0; 0; 0]  = R_GCF_to_Cam2 * Cam2_Point_GCF + t_GCF_to_Cam2
    % -t_GCF_to_Cam2 = R_GCF_to_Cam2 * Cam2_Point_GCF 
    % -R_GCF_to_Cam2^(-1) * t_GCF_to_Cam2 = Cam2_Point_GCF 
    params.cam1.pos_GCF = [0; 0; 0];
    params.cam2.pos_GCF = -params.cam2.R' * params.cam2.t; 
    
    % Resolution 
    params.image_width = 1280;
    params.image_height = 720;

    %% LiDAR detections to GCF
    R_lidar_to_Cam1 = LIDAR_cam1_calib.tform.R; 
    t_lidar_to_Cam1 = LIDAR_cam1_calib.tform.Translation'; 
    
    params.R_lidar_to_GCF = R_Cam1_to_GCF * R_lidar_to_Cam1;
    % t_lidar_to_cam1 is the offset from lidar to cam1, expressed in cam1's coord frame.
    % R_cam1_to_gcf translates cam1's coord frame to GCF's coord frame. 
    params.t_lidar_to_GCF = R_Cam1_to_GCF * t_lidar_to_Cam1; 

    %% Tracker Parameters 

    % IMM Transition Probability Matrix (2 Models: CV and CA) 
    % Row1: CV -> [Stay CV, Switch to CA] 
    % Row2: CA -> [Switch to CV, Stay CA]
    params.tracker.IMM_TPM = [0.97, 0.03; 
                              0.10, 0.90]; 
    params.tracker.tentative_track_timeout         = 2.0;
    params.tracker.maximum_allowed_hits            = 4;
    params.tracker.LIDAR_match_threshold           = 1.5;
    params.tracker.association_threshold           = 80;
    params.tracker.max_missed_frames               = 50;
    params.tracker.prediction_rate                 = 20;
    params.tracker.max_time_diff                   = 0.2;
    params.tracker.max_dist_from_origin            = 40;
    params.tracker.max_possible_velocity           = 20;
    params.tracker.chi2_probability                = 0.99;
    params.tracker.measurement_noise_std           = 2.5;
    params.tracker.measurement_noise_LIDAR_std     = 0.5; 

    % params.tracker.tentative_track_timeout = 3;      % seconds
    % params.tracker.maximum_allowed_hits    = 3;      % tracks 
    % params.tracker.LIDAR_match_threshold   = 1.5;    % metres 
    % params.tracker.association_threshold   = 150;    % pixels
    % params.tracker.max_missed_frames       = 60;     % frames
    % params.tracker.prediction_rate         = 20;     % Hz
    % params.tracker.max_time_diff           = 0.2;    % seconds
    % params.tracker.max_dist_from_origin    = 70;     % meters
    % params.tracker.max_possible_velocity   = 40;     % m/s
    % params.tracker.chi2_probability        = 0.99; 
    % params.measurement_noise_std = 2.5;              % pixels, tune to YOLO spread 

end
