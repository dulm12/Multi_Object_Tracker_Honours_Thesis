% EKF Tracking with Real Camera Data -> Week 01 T3 2025 (One Camera) 

%% Section 01: Real Measurement Data

% 1. Load full measurement set from camera + YOLO model 
measurements = readmatrix('../ThesisB_Object_Classification/bird_centroids.csv'); 

% 2. Extract data for the filter (only using one bird so far)
timestamps = measurements_bird_1(:, 1);
% u,v measurements: 
measurements_2D = measurements_bird_1(:, 2:3)'; % transpose to get 2xN format

%% Section 02: Define REAL Camera Parameters

focal_length    = 500;
principal_point = [1920/2; 1080/2]; 
K_cam = [focal_length,      0      , principal_point(1);
              0      , focal_length, principal_point(2);
              0      ,      0      ,          1         ];

% extrinsic parameters, camera pose when recording 
R_GCF_to_Cam = eye(3); 
t_GCF_to_Cam = [0; 0; 0];

%% Section 03: EKF Initialisation (First Measurement) 

num_steps = length(timestamps);
X_est = zeros(6, num_steps);

% true 3D position not known
% back-project the first measurement to an assumed initial depth.
initial_measurement = measurements_2D(:, 1);
initial_depth_guess = 10.0; % change THIS

% Back-projection math equations to get initial 3D position guess
u0 = initial_measurement(1);
v0 = initial_measurement(2);
x0 = (u0 - K_cam(1,3)) * (initial_depth_guess / K_cam(1,1));
y0 = (v0 - K_cam(2,3)) * (initial_depth_guess / K_cam(2,2));
z0 = initial_depth_guess;

% Initial state estimate: guessed position, zero velocity
X_est(:, 1) = [x0; y0; z0; 0; 0; 0];

% Initial Covariance: high uncertainty in depth (Z) and all velocities
P_est = diag([2^2, 2^2, 10^2, 5^2, 5^2, 5^2]); 

% Measurement Noise (how much to trust YOLO detections)
measurement_noise_std = 2.0; % pixels
R_ekf = diag([measurement_noise_std^2, measurement_noise_std^2]);

%% Section 04: EKF Main Loop 

for k = 2:num_steps
    % Calculate dt from the timestamps
    dt = timestamps(k) - timestamps(k-1);
    
    % the process model matrix for the current dt
    F_ekf = [1 0 0 dt 0 0; 
             0 1 0 0 dt 0; 
             0 0 1 0 0 dt;
             0 0 0 1 0  0; 
             0 0 0 0 1  0; 
             0 0 0 0 0  1];
    
    % process noise matrix for the current dt
    q_pos = 0.5 * dt^2; 
    q_vel = 2.0 * dt;   
    Q_ekf = diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel]);

    % Prediction Step 
    X_pred = F_ekf * X_est(:, k-1);
    P_pred = F_ekf * P_est * F_ekf' + Q_ekf;

    % Update Step 
    % Get measurement for current time step
    Z_measured = measurements_2D(:, k);
    
    % Perform update using the single camera's data
    [X_updated, P_updated] = EKF_Update_Step(X_pred, P_pred, Z_measured, R_ekf, K_cam, R_GCF_to_Cam, t_GCF_to_Cam);
    
    % Store the final estimate for this time step
    X_est(:, k) = X_updated;
    P_est       = P_updated; 
end

%% Section 05: Plot Estimated Trajectory

figure(1); clf;
% Plot the estimated 3D path of the bird
plot3(X_est(1,:), X_est(2,:), X_est(3,:), 'r.-', 'DisplayName', 'EKF Estimated Path');
hold on;

% Plot the starting point
plot3(X_est(1,1), X_est(2,1), X_est(3,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Point');

% Plot the camera's position
plot3(0, 0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Camera');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('EKF Estimated 3D Trajectory from Real Video');
legend show; grid on; axis equal; view(30,20);
set(gca, 'ZDir','reverse', 'YDir','reverse'); % Adjust view to match camera perspective
hold off;

%% Helper function: EKF update step 
function [X_updated, P_updated] = EKF_Update_Step(X_pred, P_pred, Z_measured, R_noise, K_cam, R_GCF_to_Cam, t_GCF_to_Cam)
    % X_pred: predicted state for the current time step (x_hat(k|k-1)). EKF's best guess before looking at the Z_measured.
    % P_pred: predicted state covariance (P(k|k-1)). the uncertainty associated with X_pred.
    % Z_measured: actual noisy 2D pixel measurement [u;v] obtained from the camera for the current time step.
    % R_noise: The 2x2 measurement noise covariance matrix R(k), tells EKF how noisy Z_measured is.
    % K_cam: The camera's 3x3 intrinsic matrix.
    % R_GCF_to_Cam: The 3x3 rotation matrix to transform points GCF to camera coords.
    % t_GCF_to_Cam: The 3x1 translation vector for GCF to camera transformation.

    % Calculate expected measurement h(X_pred)

    % get 3D position from 6D predicted state X_pred since camera
    % measurement only depends on object's 3D position
    P_GCF_pred = X_pred(1:3);

    % transform 3D GCF position into camera LCF
    % P_camera_pred = [Xc_predicted; Yc_predicted; Zc_predicted]
    P_camera_pred = R_GCF_to_Cam * P_GCF_pred + t_GCF_to_Cam;

    % check if EKF predicted point is behind or exactly on camera plane
    if P_camera_pred(3) <= 0.01 
        % No update possible
        X_updated = X_pred; 
        P_updated = P_pred;
        return;
    end

    % Perspective division 
    Xc_val_pred = P_camera_pred(1);
    Yc_val_pred = P_camera_pred(2);
    Zc_val_pred = P_camera_pred(3);
    
    Xc_norm_pred = Xc_val_pred / Zc_val_pred; 
    Yc_norm_pred = Yc_val_pred / Zc_val_pred;

    % expected 2D pixel measurement h(x_hat(k|k-1))
    Z_expected = K_cam * [Xc_norm_pred; Yc_norm_pred; 1];
    % extract u_expected and v_expected 
    Z_expected = Z_expected(1:2);

    % Calculate Jacobian H
    fx = K_cam(1,1);
    fy = K_cam(2,2);

    % Jacobian of pixel projection w.r.t 3D point in camera coords 
    % u = fx * Xc/Zc + cx
    % v = fy * Yc/Zc + cy
    % 2 x 3 matrix 
    dZexpected_dPcamera = [(fx/Zc_val_pred),      0          , (-fx * Xc_val_pred)/(Zc_val_pred^2);
                             0             , (fy/Zc_val_pred), (-fy * Yc_val_pred)/(Zc_val_pred^2)];

    % Jacobian of GCF to camera coord transformation (P_camera_pred w.r.t P_GCF_pred)
    % 3 x 3 matrix
    dPcamera_dPgcf = R_GCF_to_Cam;

    % Jacobain of full camera projection (3D GCF  to 2D pixels):
    % 2 x 3 matrix 
    dZexpected_dPgcf = dZexpected_dPcamera * dPcamera_dPgcf;

    % Full H, since u,v do not depend on vx, vy, vz in 'h' model, their
    % partial derivatives are zero 
    % H = [dZexpected_dPgcf(1,1), dZexpected_dPgcf(1,2), dZexpected_dPgcf(1,3), 0, 0, 0; 
    %      dZexpected_dPgcf(2,1), dZexpected_dPgcf(2,2), dZexpected_dPgcf(2,3), 0, 0, 0 ]; 

    H = [dZexpected_dPgcf, zeros(2, 3)];

    % EKF Update Equations
    Z = Z_measured - Z_expected;
    S = H * P_pred * H' + R_noise;
    K_gain = (P_pred * H') * (S)^(-1);

    X_updated = X_pred + K_gain * Z;
    P_updated = (eye(6) - K_gain * H) * P_pred;
end