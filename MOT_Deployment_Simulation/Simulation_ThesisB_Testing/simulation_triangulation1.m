% Simulation Validation of Thesis B Pipeline
% Prove triangulation and event-driven EKF tracks a known ground truth 
% using simulated asynchronous data that mimics the output of the real 
% Python script.

%% Section 1: Define Ground Truth & Camera Rig
clear; clc; 

% Simulation Time 
total_time = 20;
dt_sim = 0.01; % High-resolution ground truth
time_vector = 0:dt_sim:total_time;
num_time_steps = length(time_vector);

% Multi Manoeuvre Ground Truth 
true_trajectory_3D = zeros(3, num_time_steps);

initial_true_position = [0; 0; 5];

manoeuvre_time_1 = 7.5; 
manoeuvre_time_2 = 12.0; 
manoeuvre_time_3 = 15.0; 

manoeuvre_step_1 = round(manoeuvre_time_1 / dt_sim);
manoeuvre_step_2 = round(manoeuvre_time_2 / dt_sim);
manoeuvre_step_3 = round(manoeuvre_time_3 / dt_sim);

velocity_segment_1 = [2; 1; -0.5]; 
velocity_segment_2 = [-1; 2; -0.5];
velocity_segment_3 = [-1; 2; 2.0]; 
velocity_segment_4 = [-1; 2; -2.0];

% set initial state 
current_velocity = velocity_segment_1;
true_trajectory_3D(:, 1) = initial_true_position;

for i = 2:num_time_steps
    if i == manoeuvre_step_1 
        current_velocity = velocity_segment_2;

    elseif i == manoeuvre_step_2 
        current_velocity = velocity_segment_3;

    elseif i == manoeuvre_step_3 
        current_velocity = velocity_segment_4;  
    end

    % Update position using the CURRENT velocity for this segment
    true_trajectory_3D(:, i) = true_trajectory_3D(:, i-1) + current_velocity * dt_sim;
end

% Virtual camera parameters 
K_cam1 = [500, 0, 960; 0, 500, 540; 0, 0, 1];
K_cam2 = [510, 0, 955; 0, 510, 545; 0, 0, 1]; % slightly different 
image_width = 1920; image_height = 1080;

% find the center of the entire trajectory 
% 2 -> find the minimum across the rows, a row reprsents all x, y, z at
% every timestamp. 
% min_coords -> [x_min, y_min, z_min]
% max_coords -> [x_max, y_max, z_max]
% the corners of the imaginary 3D bounding box containing every single
% trajectory point 
min_traj_coordinates = min(true_trajectory_3D, [], 2); 
max_traj_coordinates = max(true_trajectory_3D, [], 2); 
look_at_point = (min_traj_coordinates + max_traj_coordinates) / 2; 

% Camera 1
cam1_pos_GCF     = [-15; -15; 1];
cam1_up_GCF      = [ 0; 0; 1];

Zc1 = (look_at_point - cam1_pos_GCF) / norm(look_at_point - cam1_pos_GCF);
Xc1 = cross(cam1_up_GCF, Zc1) / norm(cross(cam1_up_GCF, Zc1));
Yc1 = cross(Zc1, Xc1);

R_GCF_to_Cam1 = [Xc1'; Yc1'; Zc1'];
t_GCF_to_Cam1 = -R_GCF_to_Cam1 * cam1_pos_GCF;

% Camera 2
cam2_pos_GCF     = [-10; -20; 1.5]; 
cam2_up_GCF      = [0;  0;  1];

Zc2 = (look_at_point - cam2_pos_GCF) / norm(look_at_point - cam2_pos_GCF);
Xc2 = cross(cam2_up_GCF, Zc2) / norm(cross(cam2_up_GCF, Zc2));
Yc2 = cross(Zc2, Xc2);

R_GCF_to_Cam2 = [Xc2'; Yc2'; Zc2'];
t_GCF_to_Cam2 = -R_GCF_to_Cam2 * cam2_pos_GCF;

% % Define Camera 1 as the origin of rig Coordinate system 
% R_GCF_to_Cam1 = eye(3);
% t_GCF_to_Cam1 = [0; 0; 0];
% 
% % Define Camera 2 as being 1.5 meters to the right of Camera 1
% R_GCF_to_Cam2 = eye(3);
% t_GCF_to_Cam2 = [-1.5; 0; 0]; % Translation of GCF origin in Cam2 frame

%% Section 02: Simulate the Asynchronous Data Capture
% create a simulated measurement log for the EKF loop to run 
cam1_frame_rate = 30; % Hz
cam2_frame_rate = 25; % Hz 

cam1_dt = 1/cam1_frame_rate;
cam2_dt = 1/cam2_frame_rate;

measurement_log = [];
measurement_noise_std = 1.0;

% Generate measurements for Camera 1
% simulate camera's shutter clicking at a specific frame rate 
for t = 0: cam1_dt: total_time

    % find closest index in true trajectory array correspondinig to when
    % camera shutter was clicked 
    sim_idx = round(t / dt_sim) + 1;

    if sim_idx > num_time_steps
        continue; 
    end

    pt_3D = true_trajectory_3D(:, sim_idx);

    % project 3d point onto camera's 2d image plane 
    [uv, in_fov] = project_point(pt_3D, R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, image_width, image_height);
    
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
    if sim_idx > num_time_steps, continue; end
    
    pt_3D = true_trajectory_3D(:, sim_idx);
    [uv, in_fov] = project_point(pt_3D, R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, image_width, image_height);
    if in_fov
        noisy_uv = uv + randn(2,1) * measurement_noise_std;
        measurement_log = [measurement_log; t, 2, noisy_uv']; % [timestamp, cam_id, u, v]
    end
end

% Sort all measurements by timestamp
measurement_log = sortrows(measurement_log, 1);

%% Section 03: EKF Pipeline (Initialisation and Tracking)

first_cam1_idx = find(measurement_log(:, 2) == 1, 1, 'first');
first_cam2_idx = find(measurement_log(:, 2) == 2, 1, 'first');

if isempty(first_cam1_idx) || isempty(first_cam2_idx)
    error('Could not find initial measurements.'); 
end

meas1 = measurement_log(first_cam1_idx, :);
meas2 = measurement_log(first_cam2_idx, :);

max_time_diff = 0.2; % seconds
if (abs(meas1(1) - meas2(1)) > max_time_diff)
    error('Initial measurements too far apart in time.'); 
end

% Triangulate using the KNOWN camera parameters from virtual rig
P1 = K_cam1 * [R_GCF_to_Cam1, t_GCF_to_Cam1];
P2 = K_cam2 * [R_GCF_to_Cam2, t_GCF_to_Cam2];

initial_pos_guess = triangulate(meas1(3:4), meas2(3:4), P1', P2')';
initial_vel_guess = [0; 0; 0];

% EKF Setup
num_measurements = size(measurement_log, 1);

% history of the filters best guess when a specific measurement came 
X_est = zeros(6, num_measurements);
X_est(:, 1) = [initial_pos_guess; initial_vel_guess];

P_est = diag([1^2, 1^2, 1^2, 5^2, 5^2, 5^2]);
R_ekf = diag([measurement_noise_std^2, measurement_noise_std^2]);
disp('EKF initialised via simulated triangulation.');

% Event-driven EKF loop 
for k = 2:num_measurements

    dt = measurement_log(k, 1) - measurement_log(k-1, 1);
    
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
    % take last known best estimate and predict where object is now 
    X_pred = F_ekf * X_est(:, k-1);
    P_pred = F_ekf * P_est * F_ekf' + Q_ekf;

    % Update Step 
    cam_id = measurement_log(k, 2);
    Z_measured = measurement_log(k, 3:4)';
    
    if cam_id == 1
        [X_updated, P_updated] = EKF_Update_Step(X_pred, P_pred, Z_measured, R_ekf, K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1);
    else % cam_id == 2
        [X_updated, P_updated] = EKF_Update_Step(X_pred, P_pred, Z_measured, R_ekf, K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2);
    end
    
    X_est(:, k) = X_updated;
    P_est       = P_updated; 

end

%% Section 04: Validation and Visualization
% Get the ground truth points that align with our measurement timestamps
true_pts_for_plot = zeros(3, num_measurements);
for k = 1:num_measurements
    sim_idx = round(measurement_log(k,1) / dt_sim) + 1;
    true_pts_for_plot(:, k) = true_trajectory_3D(:, sim_idx);
end

% Plot 3D results
figure(1); clf; 
plot3(true_pts_for_plot(1,:), true_pts_for_plot(2,:), true_pts_for_plot(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'True Path');
hold on;
plot3(X_est(1,:), X_est(2,:), X_est(3,:), 'r.-', 'DisplayName', 'EKF Estimated Path');
legend; grid on; axis equal; title('Validation of Triangulation & Event-Driven EKF');

% Plot error over time
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