% Simulation for Week 04 - Object moving in linear trajectory, Two Cameras,
% EKF Implementation 

%% Section 01: Time 

total_time = 20;
dt = 0.1;
time_vector = 0:dt:total_time;
num_steps = length(time_vector);

%% Section 02: Object Initial State (True, moving object)

initial_true_position = [0; 0; 5]; % (metres) 
true_const_velocity = [2; 1; -0.5]; % object moving in const. velocity

%% Section 03: Calculating TRUE 3D Trajectory
% calculate position at each time step

true_trajectory_3D = zeros(3, num_steps); % Store true [x; y; z] in each column
true_6D_state      = zeros(6, num_steps); % Store true [x;y;z;vx;vy;vz] in each column

% Accessing First Column 
true_trajectory_3D(:, 1) = initial_true_position;
true_6D_state(1:3, 1)    = initial_true_position;
true_6D_state(4:6, 1)    = true_const_velocity;

for i = 2:num_steps
    % Constant Velocity Model 
    true_trajectory_3D(:, i) = true_trajectory_3D(:, i-1) + true_const_velocity * dt;
    true_6D_state(1:3, i)    = true_trajectory_3D(:, i);
    true_6D_state(4:6, i)    = true_const_velocity; % Velocity remains constant
end

%% Section 04: Virtual Cameras, K Matrix
focal_length = 500;
% assuming 640 width and 480 height image
principal_point = [320; 240]; 
K_cam = [focal_length,     0,       principal_point(1);
              0,      focal_length, principal_point(2);
              0,           0,                1         ];

image_width  = 640;
image_height = 480;

%% Section 05: Two Virtual Cameras (Extrinsic Parameters)
% Xc, Yc, Zc -> Defines Orientation of Cam in GCF 
% R -> Rotation from GCF to Cam CF. 
% t -> where the GCF origin is relative to Cam origin expressed in cam CF

% Calc. midpoint of true traj to use as the look-at pt.
midpoint_index = floor(num_steps / 2); 
traj_midpt = true_trajectory_3D(:, midpoint_index);

% Camera 1
cam1_pos_GCF     = [-5; 0; 1];
cam1_look_at_GCF = traj_midpt; 
cam1_up_GCF      = [ 0; 0; 1];

Zc1 = (cam1_look_at_GCF - cam1_pos_GCF) / norm(cam1_look_at_GCF - cam1_pos_GCF);
Xc1 = cross(cam1_up_GCF, Zc1) / norm(cross(cam1_up_GCF, Zc1));
Yc1 = cross(Zc1, Xc1);

R_GCF_to_Cam1 = [Xc1'; Yc1'; Zc1'];
t_GCF_to_Cam1 = -R_GCF_to_Cam1 * cam1_pos_GCF;

% camera 2
cam2_pos_GCF     = [0; -5; 1.5]; % different position and height 
cam2_look_at_GCF = traj_midpt;  
cam2_up_GCF      = [0;  0;  1];

Zc2 = (cam2_look_at_GCF - cam2_pos_GCF) / norm(cam2_look_at_GCF - cam2_pos_GCF);
Xc2 = cross(cam2_up_GCF, Zc2) / norm(cross(cam2_up_GCF, Zc2));
Yc2 = cross(Zc2, Xc2);

R_GCF_to_Cam2 = [Xc2'; Yc2'; Zc2'];
t_GCF_to_Cam2 = -R_GCF_to_Cam2 * cam2_pos_GCF;

%% Section 06: Generate 2D Measurements (no noise initially)

% measurements_cam1_2D will store [u1; v1]
% measurements_cam2_2D will store [u2; v2]
measurements_cam1_2D = nan(2, num_steps); % initialising with nan
measurements_cam2_2D = nan(2, num_steps);

% for use in EKF update step, to see if a measurement was taken 
is_in_FoV_cam1 = false(1, num_steps);
is_in_FoV_cam2 = false(1, num_steps);

% measurement noise std -> set close to 0 because perfect measurements
measurement_noise_std_u = 1;
measurement_noise_std_v = 1; 
R_ekf = diag([measurement_noise_std_u^2, measurement_noise_std_v^2]); 

for i = 1:num_steps
    P_true_GCF = true_trajectory_3D(:, i);

    % Camera 1 measurements

    % projection perspective 
    [uv_1, in_FoV1] = project_point(P_true_GCF, R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam, image_width, image_height);
    if in_FoV1
        is_in_FoV_cam1(i) = true;
        measurements_cam1_2D(:, i) = uv_1 + randn(2,1) .* [measurement_noise_std_u; measurement_noise_std_v]; 
    end

    % Camera 2 measurements 

    [uv_2, in_FoV2] = project_point(P_true_GCF, R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam, image_width, image_height);
    if in_FoV2
        is_in_FoV_cam2(i) = true;
        measurements_cam2_2D(:, i) = uv_2 + randn(2,1) .* [measurement_noise_std_u; measurement_noise_std_v]; 
    end
end

%% Section 07: EKF Implementation for Moving Object

X_est = zeros(6, num_steps);

% Initial EKF guess is different to true stationary position 
% Initial velocity is assumed to be ZERO 
X_est(:, 1) = [initial_true_position + [1; -1; 0.5]; 0; 0; 0];                               

% Current initial EKF Covariance -> high unct. for position, high unct. for velocity
% uncertain about vel. since initial guess is 0. 
P_est = diag([2^2, 2^2, 2^2, 4^2, 4^2, 4^2]);

% EKF Process Model, const. velocity 
% x_k = F_ekf * x_{k-1} + w_k, ignoring w_k for now 
% predicts state at current time step assuming constant velocity 
F_ekf = [1  0  0  dt 0  0 ;
         0  1  0  0  dt 0 ;
         0  0  1  0  0  dt;
         0  0  0  1  0  0 ;
         0  0  0  0  1  0 ;
         0  0  0  0  0  1 ];

% Process Noise Covariance Q_ekf 
q_val_pos = (3 * dt^2)^2; 
q_val_vel = (3 * dt)^2;   
Q_ekf = diag([q_val_pos, q_val_pos, q_val_pos, q_val_vel, q_val_vel, q_val_vel]);

% acc_max = 2; 
% acc_var = acc_max^2;
% Q_ekf_block = [dt^4/4, 0, 0, dt^3/2, 0, 0;
%                0, dt^4/4, 0, 0, dt^3/2, 0;
%                0, 0, dt^4/4, 0, 0, dt^3/2;
%                dt^3/2, 0, 0, dt^2, 0, 0;
%                0, dt^3/2, 0, 0, dt^2, 0;
%                0, 0, dt^3/2, 0, 0, dt^2];
% Q_ekf = Q_ekf_block * acc_var;

% EKF Loop
for k = 2:num_steps
    % X_hat(k | k - 1) & P(k | k - 1)
    % Pred. state for curr. time step using estimated state from previous time step. 
    X_pred = F_ekf * X_est(:, k-1);
    P_pred = F_ekf * P_est * F_ekf' + Q_ekf;

    % hold the progressively refined estimate within time step k  
    X_curr_update_step = X_pred; 
    P_curr_update_step = P_pred;

    % Camera1 Update Step 
    % if is_in_FoV_cam1(k) && ~any(isnan(measurements_cam1_2D(:,k))):
    if is_in_FoV_cam1(k)
        Z_measured1 = measurements_cam1_2D(:, k);
        % pass X_pred and P_pred to the EKF update function 
        [X_curr_update_step, P_curr_update_step] = EKF_Update_Step(X_pred, P_pred, Z_measured1, R_ekf, K_cam, R_GCF_to_Cam1, t_GCF_to_Cam1);
        X_pred = X_curr_update_step; 
        P_pred = P_curr_update_step; 
    end

    % Camera2 Update Step 
    if is_in_FoV_cam2(k)
        Z_measured2 = measurements_cam2_2D(:, k);
        % if cam1 had a measurement, we are passing in the updated X_pred
        % and P_pred from that cam1 measurement 
        [X_curr_update_step, P_curr_update_step] = EKF_Update_Step(X_pred, P_pred, Z_measured2, R_ekf, K_cam, R_GCF_to_Cam2, t_GCF_to_Cam2);
        X_pred = X_curr_update_step; 
        P_pred = P_curr_update_step; 
    end
    
    % Store final best STATE, x(k|k), and COVARAINCE, P(k|k), estimates for 
    % curr. time step k 
    X_est(:, k) = X_pred;
    P_est       = P_pred; 
end

%% Section 08: Visualisation


% Subplot1: 3D Trajectory 
figure(1); clf;
plot3(true_trajectory_3D(1,:), true_trajectory_3D(2,:), true_trajectory_3D(3,:), ...
    'bo', 'MarkerSize', 10, 'DisplayName', 'Trajectory');
hold on;

% Plot the path EKF estimates 
plot3(X_est(1,:), X_est(2,:), X_est(3,:), 'r.-', 'DisplayName', 'EKF Estimated Path');
% Plot user given EKF initial guess
plot3(X_est(1,1), X_est(2,1), X_est(3,1), 'mx', 'MarkerSize',10, 'DisplayName', 'EKF Initial Position Guess');
% Plot Cam1 and Cam2 Positions
plot3(cam1_pos_GCF(1), cam1_pos_GCF(2), cam1_pos_GCF(3), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Cam1');
plot3(cam2_pos_GCF(1), cam2_pos_GCF(2), cam2_pos_GCF(3), 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'DisplayName', 'Cam2');
xlabel('X_(GCF) (m)'); ylabel('Y_(GCF) (m)'); zlabel('Z_(GCF) (m)');
title('Moving Object: True vs EKF Estimate');
legend show; grid on; axis equal; view(30,20);
hold off;

% Subplot2: X, Y, Z Estimation Errors 
figure(2); clf;
plot(time_vector, X_est(1,:) - true_trajectory_3D(1,:), 'r', 'DisplayName', 'X Error'); 
hold on;
plot(time_vector, X_est(2,:) - true_trajectory_3D(2,:), 'g', 'DisplayName', 'Y Error');
plot(time_vector, X_est(3,:) - true_trajectory_3D(3,:), 'b', 'DisplayName', 'Z Error');
title('Position Estimation Error (EKF)');
xlabel('Time (s)'); ylabel('Error (m)');
legend show; grid on;
hold off;

% Print these values to command window 
disp('True Initial Position:'); disp(initial_true_position');
disp('EKF Initial Guess (Position):'); disp(X_est(1:3,1)');
disp('True Final Position:'); disp(true_trajectory_3D(:, end)');
disp('EKF Final Estimated Position:'); disp(X_est(1:3,end)');

%% Helper function 01: perspective projection
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

%% Helper function 02: EKF update step
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