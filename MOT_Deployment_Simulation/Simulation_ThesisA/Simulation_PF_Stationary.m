% Simulation for Week 08 - Object moving in linear trajectory, Two Cameras,
% PF Implementation for STATIONARY OBJECT 

%% Section 01: Time 

total_time = 20;
dt = 0.1;
time_vector = 0:dt:total_time;
num_steps = length(time_vector);

%% Section 02: Object Initial State (stationary object)

true_stationary_position = [0; 0; 5]; % (metres) 
true_stationary_velocity = [0; 0; 0]; % object 0 velocity

%% Section 03: Calculating TRUE 3D Trajectory
% calculate position at each time step

true_trajectory_3D = zeros(3, num_steps); % Store true [x; y; z] in each column
true_6D_state      = zeros(6, num_steps); % Store true [x;y;z;vx;vy;vz] in each column

% Accessing First Column 
true_trajectory_3D(:, 1) = true_stationary_position;
true_6D_state(1:3, 1)    = true_stationary_position;
true_6D_state(4:6, 1)    = true_stationary_velocity;

for i = 2:num_steps
    % Constant Velocity Model 
    true_trajectory_3D(:, i) = true_trajectory_3D(:, i-1) + true_stationary_velocity * dt;
    true_6D_state(1:3, i)    = true_trajectory_3D(:, i);
    true_6D_state(4:6, i)    = true_stationary_velocity; % Velocity remains 0
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

is_in_FoV_cam1 = false(1, num_steps);
is_in_FoV_cam2 = false(1, num_steps);

% measurement noise std of u and v pixels
measurement_noise_std_u = 1;
measurement_noise_std_v = 1; 

% measuremennt noise for PF's weighting step 
R_pf = diag([measurement_noise_std_u^2, measurement_noise_std_v^2]); 

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

%% Section 07: Particle Filter Implementation for Stationary Object

% PF Parameters 
% no. of particles, MORE is accurate, LESS is computationally efficient
N_particles = 5000; 

% 1. Initialise particles 

% Cloud of 'N' particles created around an initial guess 
% Each column is one 6D state hypothesis 
particles = zeros(6, N_particles); 

% Initial guesses of pos and vel (same as EKF) 
initial_guess_pos = true_stationary_position + [1; -1; 0.5]; 
initial_guess_vel = [0; 0; 0]; 
initial_state_guess = [initial_guess_pos; initial_guess_vel]; 

% Particles spread around initial guess with some uncertainty 
pos_uncertainty = 2; % HIGH INITIAL UNC. for pos
vel_uncertainty = 0.1; % LOW INITIAL UNC. for velocity 

% randn -> diffuse INITIAL set of particles (INITIAL Unc.)
for i = 1:N_particles
    particles(:, i) = initial_state_guess + ...
        diag([pos_uncertainty, pos_uncertainty, pos_uncertainty, vel_uncertainty, vel_uncertainty, vel_uncertainty]) * randn(6,1);
end

% Initialise particle weights (all are equally likely initially)
% to be updated when a measurement comes in 
weights = ones(1, N_particles) / N_particles;

% Store final estimated state, the weighted average of particles (each of the 6 columns stores a hypothesis for each state) 
% at each time step 
X_est_pf = zeros(6, num_steps); 
X_est_pf(:, 1) = mean(particles, 2);                                

% PF Process Model (same as EKF) 
F_pf = [ 1  0  0  dt 0  0 ;
         0  1  0  0  dt 0 ;
         0  0  1  0  0  dt;
         0  0  0  1  0  0 ;
         0  0  0  0  1  0 ;
         0  0  0  0  0  1  ];

% Process Noise Covariance Q_ekf 
% high confidence that object is stationary, filter
% must not expect much movement (low diffusion).
q_val_pos = 0.5; % metres 
q_val_vel = 0.001; % m/s   

% 6 x 1 vector, used for particle diffusion. 
% used to scale a 6 x 1 randn vector, that is added to particles matrix 
Q_pf_std = [q_val_pos, q_val_pos, q_val_pos, q_val_vel, q_val_vel, q_val_vel]';

% 2. pre-calculate inverse of R for efficiency 
inv_R_pf = inv(R_pf); 

% PF Loop
for k = 2:num_steps

    % 3. PF Prediction (move and diffuse particles matrix)
    for i = 1:N_particles
        % move each particle according to  motion model
        particles(:, i) = F_pf * particles(:, i);
        % randn -> diffuse EACH time step set of particles (new Unc. in each time step)
        particles(:, i) = particles(:, i) + Q_pf_std .* randn(6,1);
    end

    % measurement from Cam 1
    if is_in_FoV_cam1(k)
        Z_measured1 = measurements_cam1_2D(:, k);
        for i = 1:N_particles
            % weight -> 'score', how good each particle's hypothesis is.
            % compare what  particle PREDICTS with what camera saw. 
            % If close, particle is good hypothesis, high score (weight).

            particle_pos = particles(1:3, i);
            % What particle predicts -> z_expected 
            [Z_expected1, ~] = project_point(particle_pos, R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam, image_width, image_height);
            
            % if pixel point was in the camera field of view, update
            % weights according to comparison 
            if ~any(isnan(Z_expected1))
                error = Z_measured1 - Z_expected1;
                % weighting formula:
                likelihood = exp(-0.5 * error' * inv_R_pf * error);
                
                % multiply old weight with new likelihood, good predictors
                % will have weights increased
                weights(i) = weights(i) * likelihood;
            else
                %  if any particle projects outside FoV provide only a
                %  small weight 
                weights(i) = weights(i) * 1e-10; 
            end
        end
        % normalise weights to sum up to 1
        % Avoid division by zero
        if sum(weights) > 1e-9 
            weights = weights / sum(weights);
        else
            % if all particles are bad, reset filter
            weights = ones(1, N_particles) / N_particles;
        end
    end
    
    % measurement from Cam 2
    if is_in_FoV_cam2(k)
        Z_measured2 = measurements_cam2_2D(:, k);
        for i = 1:N_particles
            % weight -> 'score', how good each particle's hypothesis is.
            % compare what  particle PREDICTS with what camera saw. 
            % If close, particle is good hypothesis, high score (weight).

            particle_pos = particles(1:3, i);
            % What particle predicts -> z_expected 
            [Z_expected2, ~] = project_point(particle_pos, R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam, image_width, image_height);
            
            % if pixel point was in the camera field of view, update
            % weights according to comparison 
            if ~any(isnan(Z_expected2))
                error = Z_measured2 - Z_expected2;
                % weighting formula:
                likelihood = exp(-0.5 * error' * inv_R_pf * error);
                
                % multiply old weight with new likelihood, good predictors
                % will have weights increased
                weights(i) = weights(i) * likelihood;
            else
                %  if any particle projects outside FoV provide only a
                %  small weight 
                weights(i) = weights(i) * 1e-10; 
            end
        end
        % normalise weights after the second measurement
        % Avoid division by zero
        if sum(weights) > 1e-9 
            weights = weights / sum(weights);
  
        else % All particles are bad, re-initialize weights
            weights = ones(1, N_particles) / N_particles;
        end
    end

    % 4. low variance resampling
    N_eff_particles = 1 / sum(weights.^2);
    if N_eff_particles < N_particles / 2
        % need resampling, generate a new set of particles
        new_particles = zeros(6, N_particles);

        % Get a single random starting point (new point for each run, prevents systematic bias)
        r = rand() / N_particles;

        % Get N evenly spaced pointers (low variance) 
        % allows large weighted particles to be selected many times 
        % vector of N points: [0, 1/N, 2/N, 3/N, ..., (N-1)/N] + r
        pointers = r + (0: N_particles - 1) / N_particles;

        %  e.g. weights [0.1, 0.4, 0.2, 0.3], cumsum [0.1, 0.5, 0.7, 1.0]. 
        % represents boundaries of particle's slice on a number line from 0 to 1. 
        % Particle 1 occupies the space [0, 0.1], Particle 2 occupies [0.1, 0.5] etc. 
        cumulative_weights = cumsum(weights);

        i = 1; % index for new_particles
        j = 1; % index for old particles

        while (i <= N_particles)
            if (pointers(i) < cumulative_weights(j))
                new_particles(:, i) = particles(:, j);
                i = i + 1;
            else
                j = j + 1;
            end
        end

        particles = new_particles;
        % After resampling, all particles have equal weight again to be
        % ready for next batch of resampling 
        weights = ones(1, N_particles) / N_particles;
    end

    % 5. Final State Estimation for curr. time step
    % weighted average of all particles for curr. time step
    % particles -> 6 x N, weights -> 1 x N, weights' -> N x 1
    % X_est_pf(k) -> 6 x 1 vector 
    X_est_pf(:, k) = particles * weights';
    
end

%% Section 08: Visualisation

% Subplot1: 3D Trajectory 
figure(1); clf;
plot3(true_stationary_position(1), true_stationary_position(2), true_stationary_position(3), ...
    'bo', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'True Stationary Pos');
hold on;
% PF estimated path 
plot3(X_est_pf(1,:), X_est_pf(2,:), X_est_pf(3,:), 'r.-', 'DisplayName', 'PF Estimated Path');
% Initial PF guessed position 
plot3(X_est_pf(1,1), X_est_pf(2,1), X_est_pf(3,1), 'mx', 'MarkerSize',10, 'DisplayName', 'PF Initial Position Guess');
% Cam1 position 
plot3(cam1_pos_GCF(1), cam1_pos_GCF(2), cam1_pos_GCF(3), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Cam1');
% Cam2 position 
plot3(cam2_pos_GCF(1), cam2_pos_GCF(2), cam2_pos_GCF(3), 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'DisplayName', 'Cam2');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Stationary Object: True vs. Particle Filter Estimate');
legend show; grid on; axis equal; view(30,20);
hold off;

% Subplot2: X, Y, Z Estimation Errors 
figure(2); clf;
plot(time_vector, X_est_pf(1,:) - true_stationary_position(1), 'r', 'DisplayName', 'X Error'); 
hold on;
plot(time_vector, X_est_pf(2,:) - true_stationary_position(2), 'g', 'DisplayName', 'Y Error');
plot(time_vector, X_est_pf(3,:) - true_stationary_position(3), 'b', 'DisplayName', 'Z Error');
title('Position Estimation Error (Particle Filter)');
xlabel('Time (s)'); ylabel('Error (m)');
legend show; grid on;
hold off;

% Print these values to command window 
disp('True Initial Position:'); disp(true_stationary_position');
disp('PF Initial Guess (Position):'); disp(X_est_pf(1:3,1)');
disp('True Final Position:'); disp(true_stationary_position');
disp('PF Final Estimated Position:'); disp(X_est_pf(1:3,end)');

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