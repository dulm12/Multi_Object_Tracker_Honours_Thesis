% Brute Force Method 
% initialise EKF by triangulating two asynchronous measurements. (event
% driven filter) 

%% Section 01: Simulation Setup (Ground Truth and Cameras)
clear; clc; close all;

% Time 
total_time = 20;
dt_sim = 0.01; 
time_vector = 0:dt_sim:total_time;
num_steps = length(time_vector);

% Ground Truth Trajectory 
true_trajectory_3D = zeros(3, num_steps);
initial_true_position = [0; 0; 5];
% (multi-maneuver velocity generation code would go here to create the trajectory)
% For simplicity, we'll use a linear trajectory for this validation script.
velocity = [2; 1; -0.5];

for i = 2:num_steps
    true_trajectory_3D(:, i) = true_trajectory_3D(:, i-1) + velocity * dt_sim;
end

% Camera Parameters 
focal_length = 500;
principal_point = [1920/2; 1080/2];

K_cam = [focal_length,      0      , principal_point(1); 
              0      , focal_length, principal_point(2); 
              0      ,      0      ,          1         ];

image_width = 1920; 
image_height = 1080;

cam1_pos_GCF = [-5; 0; 1];
cam2_pos_GCF = [0; -5; 1.5];
look_at_point = true_trajectory_3D(:, floor(num_steps / 2));

% Extrinsic Camera Calculation 
% z -> forward, x -> right, y -> up 
% camera pose in GCF
Zc1 = (look_at_point - cam1_pos_GCF) / norm(look_at_point - cam1_pos_GCF);
Xc1 = cross([0;0;1], Zc1) / norm(cross([0;0;1], Zc1)); 
Yc1 = cross(Zc1, Xc1);

% camera pose in LCF
R_GCF_to_Cam1 = [Xc1'; Yc1'; Zc1']; 
t_GCF_to_Cam1 = -R_GCF_to_Cam1 * cam1_pos_GCF;

Zc2 = (look_at_point - cam2_pos_GCF) / norm(look_at_point - cam2_pos_GCF);
Xc2 = cross([0;0;1], Zc2) / norm(cross([0;0;1], Zc2)); 
Yc2 = cross(Zc2, Xc2);

R_GCF_to_Cam2 = [Xc2'; Yc2'; Zc2']; 
t_GCF_to_Cam2 = -R_GCF_to_Cam2 * cam2_pos_GCF;

%% Section 02: Generate Asynchronous Measurements

cam1_frame_rate = 30; % Hz
cam2_frame_rate = 25; % Hz 

cam1_dt = 1/cam1_frame_rate;
cam2_dt = 1/cam2_frame_rate;

measurement_log = []; 
% used to add random noise
measurement_noise_std = 1.0;

% Generate measurements for Camera 1
% simulate camera's shutter clicking at a specific frame rate 
for t = 0: cam1_dt :total_time

    % find closest index in true trajectory array correspondinig to when
    % camera shutter was clicked 
    sim_idx = round(t / dt_sim) + 1;

    if sim_idx > num_steps
        continue; 
    end

    pt_3D = true_trajectory_3D(:, sim_idx);

    % project 3d point onto camera's 2d image plane 
    [uv, in_fov] = project_point(pt_3D, R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam, image_width, image_height);
    
    if in_fov
        % add random noise 
        noisy_uv = uv + randn(2,1) * measurement_noise_std;
        % record the measurement as a new row
        measurement_log = [measurement_log; t, 1, noisy_uv']; % [timestamp, cam_id, u, v]
    end
end

% Generate measurements for Camera 2
for t = 0 : cam2_dt : total_time
    sim_idx = round(t / dt_sim) + 1;
    if sim_idx > num_steps, continue; end
    
    pt_3D = true_trajectory_3D(:, sim_idx);
    [uv, in_fov] = project_point(pt_3D, R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam, image_width, image_height);
    if in_fov
        noisy_uv = uv + randn(2,1) * measurement_noise_std;
        measurement_log = [measurement_log; t, 2, noisy_uv']; % [timestamp, cam_id, u, v]
    end
end

% Sort all measurements by timestamp
measurement_log = sortrows(measurement_log, 1);

%% Section 03: Brute-Force Triangulation initialisation

% Create logical map of cam_ID == 1 rows, seach from first row downwards.
% once first value found, it get row number and return it
first_cam1_idx = find(measurement_log(:, 2) == 1, 1, 'first');
first_cam2_idx = find(measurement_log(:, 2) == 2, 1, 'first');

if isempty(first_cam1_idx) || isempty(first_cam2_idx)
    error('Could not find initial measurements from both/either cameras.');
end

meas1 = measurement_log(first_cam1_idx, :);
meas2 = measurement_log(first_cam2_idx, :);

% CHECK: are measurements close enough in time?
max_time_diff = 0.2; % seconds
if abs(meas1(1) - meas2(1)) > max_time_diff
    error('Initial measurements are too far apart in time for triangulation.');
end

% Triangulate to get initial 3D position guess
% P1, P2: any 3d point in GCF -> 2d image plane of cam 
P1 = K_cam * [R_GCF_to_Cam1, t_GCF_to_Cam1]; % Camera 1 projection matrix
P2 = K_cam * [R_GCF_to_Cam2, t_GCF_to_Cam2]; % Camera 2 projection matrix

% meas(3,4) -> 2d u, v point of each camera (row)
% takes in transpose of the projection matrices 
% finds the intersection point of the two rays 
initial_pos_guess = triangulate(meas1(3:4), meas2(3:4), P1', P2')';

% To get an initial velocity, triangulate the next pair of points
% assume zero initial velocity for now 
initial_vel_guess = [0; 0; 0];

% EKF initialisation 
num_measurements = size(measurement_log, 1);
% 6 x N matrix for storing filter state estimate at each measurement step
X_est = zeros(6, num_measurements);
X_est(:, 1) = [initial_pos_guess; initial_vel_guess]; % Use triangulated guess

% filters inital confidence in the pos + velocity guess 
P_est = diag([1^2, 1^2, 1^2, 5^2, 5^2, 5^2]); % Moderate pos uncertainty, high vel uncertainty
% measurement noise -> how much to trust the incoming measurement from
% sensor 
R_ekf = diag([measurement_noise_std^2, measurement_noise_std^2]);
disp('EKF initialised via triangulation.');

%% Section 04: Event-Driven EKF Loop
% iterates through measurement log
for k = 2:num_measurements

    dt = measurement_log(k, 1) - measurement_log(k - 1, 1);
    
    % process matrix: the motion model (Constant velocity here)
    F_ekf = [1 0 0 dt 0 0; 
             0 1 0 0 dt 0; 
             0 0 1 0 0 dt; 
             0 0 0 1 0  0; 
             0 0 0 0 1  0; 
             0 0 0 0 0  1];

    % process noise covariance matrix: uncertainty in motion model 
    q_pos = 0.5 * dt^2; q_vel = 2.0 * dt;   
    Q_ekf = diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel]);

    % Prediction Step (fast-forward old state through time)
    % always take corrected state estimate from previous measurement 
    X_pred = F_ekf * X_est(:, k-1);
    P_pred = F_ekf * P_est * F_ekf' + Q_ekf;

    % Update Step 
    cam_id = measurement_log(k, 2);
    Z_measured = measurement_log(k, 3:4)';
    
    if cam_id == 1
        [X_updated, P_updated] = EKF_Update_Step(X_pred, P_pred, Z_measured, R_ekf, K_cam, R_GCF_to_Cam1, t_GCF_to_Cam1);
    else % cam_id == 2
        [X_updated, P_updated] = EKF_Update_Step(X_pred, P_pred, Z_measured, R_ekf, K_cam, R_GCF_to_Cam2, t_GCF_to_Cam2);
    end
    
    X_est(:, k) = X_updated;
    P_est       = P_updated; 

end

%% Section 05: Visualisation
true_pts_for_plot = zeros(3, num_measurements);
for k = 1:num_measurements
    % get timestamp of when measurement occured, convert it into closest
    % index of the true trajectory array (+1 is because of MATLAB indexing)
    sim_idx = round(measurement_log(k,1) / dt_sim) + 1;
    true_pts_for_plot(:, k) = true_trajectory_3D(:, sim_idx);
end

figure(1); clf;
plot3(true_pts_for_plot(1,:), true_pts_for_plot(2,:), true_pts_for_plot(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'True Path');
hold on;
plot3(X_est(1,:), X_est(2,:), X_est(3,:), 'r.-', 'DisplayName', 'EKF Estimated Path');
legend; grid on; axis equal; title('EKF Tracking with Triangulation Initialisation');

figure(2); clf;
error = X_est(1:3, :) - true_pts_for_plot;
plot(measurement_log(:,1), error(1,:), 'r', 'DisplayName', 'X Error'); hold on;
plot(measurement_log(:,1), error(2,:), 'g', 'DisplayName', 'Y Error');
plot(measurement_log(:,1), error(3,:), 'b', 'DisplayName', 'Z Error');
legend; grid on; title('Position Estimation Error'); xlabel('Time (s)'); ylabel('Error (m)');

%% Helper Functions

function [uv, in_fov] = project_point(point_GCF, R_GCF2Cam, t_GCF2Cam, K_cam, img_width, img_height)
    uv = [NaN; NaN];
    in_fov = false;

    % 1. Get point in camera CF.
    point_camera = R_GCF2Cam * point_GCF + t_GCF2Cam;

    % 2. Check if object is in front of cam
    if point_camera(3) > 0.01

        % 3. Get x and y coords of curr. camera point 
        x_norm = point_camera(1) / point_camera(3);
        y_norm = point_camera(2) / point_camera(3);

        % 4. Find the corresponding homogeneous pixel points (u, v) of this
        % camera point 
        point_homo_norm = [x_norm; y_norm; 1];
        point_pixels_homo = K_cam * point_homo_norm;

        u_value = point_pixels_homo(1);
        v_value = point_pixels_homo(2);

        % 5. Check if these points are within the camera viewing
        % boundaries. 
        if ((u_value >= 0 && u_value < img_width) && (v_value >= 0 && v_value < img_height))
            uv = [u_value; v_value];
            in_fov = true;
        end

    end
end

function [X_updated, P_updated] = EKF_Update_Step(X_pred, P_pred, Z_measured, R_noise, K_cam, R_GCF_to_Cam, t_GCF_to_Cam)
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