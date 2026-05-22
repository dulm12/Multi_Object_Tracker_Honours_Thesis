% Simulation

% Simulates the 3D trajectory of a single object and then projects this trajectory
% onto the 2D image plane of a virtual pinhole camera.
% It then uses an EKF to estimate the 3D trajectory from the noisy 2D measurements.

%% Section 01: Time Vector

total_time = 10;
dt = 0.1;
time_vector = 0:dt:total_time;

%% Section 02: Object Initial State and Velocity Vector (True Trajectory)

initial_position = [0; 0; 5]; % [x; y; z] (metres)
const_velocity = [1; 0.5; -0.2];   % [vx; vy; vz] (m/s)

%% Section 03: Calculating True 3D Trajectory

num_steps          = length(time_vector);
true_trajectory_3D = zeros(3, num_steps); % Store true [x; y; z] in each column
true_state         = zeros(6, num_steps); % Store true [x;y;z;vx;vy;vz] in each column

% Accessing First Column 
true_trajectory_3D(:, 1) = initial_position;
true_state(1:3, 1)       = initial_position;
true_state(4:6, 1)       = const_velocity;

for i = 2:num_steps
    % Constant Velocity Model 
    true_trajectory_3D(:, i) = true_trajectory_3D(:, i-1) + const_velocity * dt;
    true_state(1:3, i) = true_trajectory_3D(:, i);
    true_state(4:6, i) = const_velocity; % Velocity remains constant
end

%% Section 04: Virtual Camera, K Matrix (Intrinsic Parameters)

focal_length = 500;
principal_point = [320; 240];

% Transform 3D points from the camera's own coordinate system 
% into 2D pixel coordinates on the image plane.
K_cam = [focal_length,     0,       principal_point(1); 
              0,      focal_length, principal_point(2);
              0,           0,                1         ];

%% Section 05: Virtual Camera Position Definition (Extrinsic Parameters)

camera_position_GCF = [-5; -5; 1]; % Camera Position in GCF 
look_at_point_GCF   = [0; 0; 0];   % Point the camera is looking at
camera_up_GCF       = [0; 0; 1];   % 'Up' direction of camera 

% Camera's local coord Axes (Orientation in GCF)
% X,Y,Z axes if standing at the camera and looking through it

% Cam Viewing Direction
Zc = (look_at_point_GCF - camera_position_GCF) / norm(look_at_point_GCF - camera_position_GCF);

% Cam Local X-axis 
Xc = cross(camera_up_GCF, Zc) / norm(cross(camera_up_GCF, Zc));

% Cam Local Y-axis 
Yc = cross(Zc, Xc);

R_GCF_to_Cam = [Xc'; Yc'; Zc'];
t_GCF_to_Cam = -R_GCF_to_Cam * camera_position_GCF; % Translation of GCF origin in cam CF

%% Section 06: Generate Noisy 2D Measurements from True 3D Trajectory
measurements_2D_noisy = zeros(2, num_steps); % Store noisy [u; v]
is_in_FoV = false(1, num_steps);

image_width  = 640;
image_height = 480;

% Measurement Noise STD (in pixels)
measurement_noise_std_u = 2.0; % 2 pixels noise in u
measurement_noise_std_v = 2.0; % 2 pixels noise in v
R_ekf = diag([measurement_noise_std_u^2, measurement_noise_std_v^2]); % Measurement noise covariance for EKF

for i = 1:num_steps
    % True 3D position of object at current time step 
    P_true_GCF = true_trajectory_3D(:, i);
    % Transform this to Camera CF
    P_camera_true = R_GCF_to_Cam * P_true_GCF + t_GCF_to_Cam;

    % if object in front of camera 
    if P_camera_true(3) > 0.01
        % perform perspective projection
        
        % scales down to project its looks on a 1 unit away image plane. 
        % x_n, y_n, coords on the plane centered around optical axis 
        x_norm = P_camera_true(1) / P_camera_true(3);
        y_norm = P_camera_true(2) / P_camera_true(3);

        % add extra dimension so single transformation matrix obtained
        % understand these two lines better 
        p_homo_norm = [x_norm; y_norm; 1];

        % focal_length scales x_n, y_n to distances on the image sensor.
        % shifts origin from princip. pt. to actual org. of image sensor
        p_pixels_homo = K_cam * p_homo_norm;

        u_true = p_pixels_homo(1);
        v_true = p_pixels_homo(2);

        % check if the u,v are within the boundaries of the camera's image
        if u_true >= 0 && u_true < image_width && v_true >= 0 && v_true < image_height
            is_in_FoV(i) = true;
            % Add Gaussian noise to the true pixel measurements
            % randn -> rand. no. from std distribution 
            % scale this rand. noise to match error of std_u and std_v. 
            measurements_2D_noisy(:, i) = [u_true; v_true] + randn(2,1) .* [measurement_noise_std_u; measurement_noise_std_v];
        else
            measurements_2D_noisy(:, i) = [NaN; NaN];
        end
    else
        measurements_2D_noisy(:, i) = [NaN; NaN];
    end
end

%% Section 07: EKF Implementation
% EKF State: X_est = [x_est, y_est, z_est, vx_est, vy_est, vz_est]' (6x1)
% EKF Covariance: P_est (6x6), plots uncertainty. 
X_est = zeros(6, num_steps);
P_est_diag_history = zeros(6, num_steps); 

% initial EKF guess (true + noise)
X_est(:, 1) = true_state(:,1) + randn(6,1).* [0.5;0.5;0.5;0.1;0.1;0.1]; 

% initial EKF covariance (uncertainty in initial guess)
P_est = diag([1^2, 1^2, 1^2, 0.5^2, 0.5^2, 0.5^2]); 
P_est_diag_history(:,1) = diag(P_est);

% EKF constant veloc. process model 
% x_k = F * x_{k-1} + w_k
% w_k -> process noise, i assume random accelerations average to zero over
% time, so 0 mean and Q_ekf, process noise covariance matrix. 
F_ekf = [1  0  0  dt 0  0 ;
         0  1  0  0  dt 0 ;
         0  0  1  0  0  dt;
         0  0  0  1  0  0 ;
         0  0  0  0  1  0 ;
         0  0  0  0  0  1 ];

% Process Noise Covariance Q_ekf (reflects unmodeled accelerations)
a_max_ekf = 0.5; % m/s^2
Q_vel_unc = (a_max_ekf * dt)^2; % Simplified: variance of velocity change in one step
Q_pos_unc_from_vel = (0.5 * a_max_ekf * dt^2)^2; % Simplified: variance of position change due to vel change

Q_ekf = diag([Q_pos_unc_from_vel, Q_pos_unc_from_vel, Q_pos_unc_from_vel, ...
              Q_vel_unc, Q_vel_unc, Q_vel_unc]);
% A common form for Q in CV involves dt^4, dt^3, dt^2 terms.
% For simplicity here, we use a diagonal Q. Refer to lecture page 6, (xi(k) ~ N(0,Q(k)))
% The Q matrix from your lecture (page 9) for the pendulum is a specific example.
% For CV, let's use a simpler form or you can derive a more complex one.
% For our 6D state:
accel_variance = a_max_ekf^2; % (m/s^2)^2
Q_ekf_block = [dt^4/4 * accel_variance, 0, 0, dt^3/2 * accel_variance, 0, 0;
               0, dt^4/4 * accel_variance, 0, 0, dt^3/2 * accel_variance, 0;
               0, 0, dt^4/4 * accel_variance, 0, 0, dt^3/2 * accel_variance;
               dt^3/2 * accel_variance, 0, 0, dt^2 * accel_variance, 0, 0;
               0, dt^3/2 * accel_variance, 0, 0, dt^2 * accel_variance, 0;
               0, 0, dt^3/2 * accel_variance, 0, 0, dt^2 * accel_variance];
% Q_ekf = Q_ekf_block; % Using a more standard continuous Wiener process acceleration model derived Q

% EKF Loop
for k = 2:num_steps
    % --- Prediction Step ---
    % x_pred = f(X_est_prev, u_prev) -> for CV, x_pred = F_ekf * X_est_prev
    X_pred = F_ekf * X_est(:, k-1);
    % P_pred = J * P_est_prev * J' + Q_ekf -> for CV, J is F_ekf
    P_pred = F_ekf * P_est * F_ekf' + Q_ekf;

    % --- Update Step (if measurement is available) ---
    if is_in_FoV(k) && ~any(isnan(measurements_2D_noisy(:, k)))
        % Current noisy measurement
        Z_measured = measurements_2D_noisy(:, k);

        % Calculate expected measurement h(X_pred)
        % This is your camera projection function using X_pred(1:3)
        P_world_pred = X_pred(1:3); % Predicted 3D position
        P_camera_pred = R_GCF_to_Cam * P_world_pred + t_GCF_to_Cam;

        % Check if predicted point is in front of camera for h(x)
        if P_camera_pred(3) > 0.01
            x_norm_pred = P_camera_pred(1) / P_camera_pred(3);
            y_norm_pred = P_camera_pred(2) / P_camera_pred(3);
            p_homo_norm_pred = [x_norm_pred; y_norm_pred; 1];
            p_pixels_homo_pred = K_cam * p_homo_norm_pred;
            Z_expected = p_pixels_homo_pred(1:2); % h(X_pred)

            % Calculate Jacobian H of observation function h(X) w.r.t X
            % H = dh/dX | evaluated at X_pred
            % For h(x) = [ K(1,1)*x_cam/z_cam + K(1,3);
            %              K(2,2)*y_cam/z_cam + K(2,3) ]
            % where x_cam, y_cam, z_cam are functions of X_pred(1:3) via R_GCF_to_Cam, t_GCF_to_Cam
            % This is the most complex part to derive analytically.
            % Let P_cam = R*P_world + t = [Xc; Yc; Zc]
            % u = fx * Xc/Zc + cx
            % v = fy * Yc/Zc + cy
            % H will be 2x6. Derivatives w.r.t vx,vy,vz are 0.
            % du/dx = fx * ( (dXc/dx)*Zc - Xc*(dZc/dx) ) / Zc^2
            % dXc/dx = R_GCF_to_Cam(1,1), dYc/dx = R_GCF_to_Cam(2,1), dZc/dx = R_GCF_to_Cam(3,1)
            % etc. for dy, dz.

            Xc_val = P_camera_pred(1);
            Yc_val = P_camera_pred(2);
            Zc_val = P_camera_pred(3);
            fx = K_cam(1,1);
            fy = K_cam(2,2);

            % Jacobian of camera projection w.r.t P_camera = [Xc, Yc, Zc]'
            dH_dPc = [fx/Zc_val, 0,        -fx*Xc_val/(Zc_val^2);
                      0,         fy/Zc_val, -fy*Yc_val/(Zc_val^2)];

            % Jacobian of P_camera w.r.t P_world (which is X_pred(1:3)) is R_GCF_to_Cam
            dPc_dPworld = R_GCF_to_Cam; % This is 3x3

            % Chain rule: dH_dPworld = dH_dPc * dPc_dPworld
            H_jacobian_pos = dH_dPc * dPc_dPworld; % This is 2x3

            H_ekf = [H_jacobian_pos, zeros(2,3)]; % Full 2x6 Jacobian for EKF state

            % Innovation (Residual)
            Y_residual = Z_measured - Z_expected;

            % Innovation Covariance
            S_ekf = H_ekf * P_pred * H_ekf' + R_ekf;

            % Kalman Gain
            K_gain = P_pred * H_ekf' * inv(S_ekf); % inv(S_ekf) or S_ekf\eye(size(S_ekf))

            % Update State Estimate
            X_est(:, k) = X_pred + K_gain * Y_residual;

            % Update Covariance Estimate
            P_est = (eye(6) - K_gain * H_ekf) * P_pred;
        else
            % Predicted point is behind camera, cannot form expected measurement
            % Only do prediction
            X_est(:, k) = X_pred;
            P_est = P_pred;
        end
    else
        % No valid measurement (out of FoV or NaN)
        % Only do prediction step
        X_est(:, k) = X_pred;
        P_est = P_pred;
    end
    P_est_diag_history(:,k) = diag(P_est); % Store diagonal for plotting uncertainty
end


%% Section 08: Visualization with EKF results
figure (2);
% Plot True 3D Trajectory vs EKF Estimated 3D Trajectory
subplot(2,1,1);
plot3(true_trajectory_3D(1,:), true_trajectory_3D(2,:), true_trajectory_3D(3,:), 'b-', 'DisplayName', 'True Trajectory');
hold on;
plot3(X_est(1,:), X_est(2,:), X_est(3,:), 'r--', 'DisplayName', 'EKF Estimate');
plot3(camera_position_GCF(1), camera_position_GCF(2), camera_position_GCF(3), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Camera');
xlabel('X_w (m)'); ylabel('Y_w (m)'); zlabel('Z_w (m)');
title('True vs EKF Estimated 3D Trajectory');
legend show; grid on; axis equal; view(45,20);
hold off;

% Plot 2D measurements vs EKF projected measurements
subplot(2,1,2);
plot(measurements_2D_noisy(1,is_in_FoV), measurements_2D_noisy(2,is_in_FoV), 'bx', 'DisplayName', 'Noisy Measurements');
hold on;
% Project EKF estimated trajectory back to 2D for comparison
ekf_projected_2D = zeros(2, num_steps);
for k_vis = 1:num_steps
    if is_in_FoV(k_vis) % Only project if original measurement was in FoV
        P_world_ekf_est = X_est(1:3, k_vis);
        P_camera_ekf_est = R_GCF_to_Cam * P_world_ekf_est + t_GCF_to_Cam;
        if P_camera_ekf_est(3) > 0.01
            x_norm_ekf = P_camera_ekf_est(1) / P_camera_ekf_est(3);
            y_norm_ekf = P_camera_ekf_est(2) / P_camera_ekf_est(3);
            p_homo_norm_ekf = [x_norm_ekf; y_norm_ekf; 1];
            p_pixels_homo_ekf = K_cam * p_homo_norm_ekf;
            ekf_projected_2D(:, k_vis) = p_pixels_homo_ekf(1:2);
        else
            ekf_projected_2D(:, k_vis) = [NaN; NaN];
        end
    else
         ekf_projected_2D(:, k_vis) = [NaN; NaN];
    end
end
plot(ekf_projected_2D(1,is_in_FoV), ekf_projected_2D(2,is_in_FoV), 'r-o', 'DisplayName', 'EKF Projected');
xlabel('u (pixels)'); ylabel('v (pixels)');
title('2D Measurements vs EKF Projected Estimate');
axis([0 image_width 0 image_height]);
set(gca, 'YDir','reverse'); grid on; axis equal; legend show;
hold off;

% Optional: Plot estimation errors
figure(3);
error_pos = X_est(1:3,:) - true_trajectory_3D;
subplot(3,1,1); plot(time_vector, error_pos(1,:)); title('X Position Error (EKF - True)'); ylabel('Error (m)'); grid on;
subplot(3,1,2); plot(time_vector, error_pos(2,:)); title('Y Position Error'); ylabel('Error (m)'); grid on;
subplot(3,1,3); plot(time_vector, error_pos(3,:)); title('Z Position Error'); ylabel('Error (m)'); xlabel('Time (s)'); grid on;

% Optional: Plot uncertainty (standard deviation from P_est diagonal)
figure(4);
std_dev_pos = sqrt(P_est_diag_history(1:3,:));
subplot(3,1,1); plot(time_vector, std_dev_pos(1,:)); title('EKF X Pos Std Dev'); ylabel('Std Dev (m)'); grid on;
subplot(3,1,2); plot(time_vector, std_dev_pos(2,:)); title('EKF Y Pos Std Dev'); ylabel('Std Dev (m)'); grid on;
subplot(3,1,3); plot(time_vector, std_dev_pos(3,:)); title('EKF Z Pos Std Dev'); ylabel('Std Dev (m)'); xlabel('Time (s)'); grid on;


%% Section 09: Output EKF results (Example)
disp('EKF Estimated Final 3D Position:');
disp(X_est(1:3, end));
disp('True Final 3D Position:');
disp(true_trajectory_3D(:, end));