% Simulate two objects, use data association and track management, use a
% continuous EKF with nearest-neighbour data association. 

%% Section 1: Define Ground Truth & Camera Rig
clear; clc; close all;

% Simulation time 
total_time = 20;
dt_sim = 0.01;
time_vector = 0: dt_sim: total_time;
num_steps = length(time_vector);

% Ground Truth for Object 1 (manouevring)
true_trajectory_3D_1 = zeros(3, num_steps);
initial_pos_1 = [0; 0; 5];
% multi-manouevre 
v1_seg1=[2;1;-0.5]; v1_seg2=[-1;2;-0.5]; v1_seg3=[-1;2;2]; v1_seg4=[-1;2;-2];

manouevre_obj1_time1 = 7.5; manouevre_obj1_step1 = round(manouevre_obj1_time1/dt_sim); 
manouevre_obj1_time2 = 12;  manouevre_obj1_step2 = round(manouevre_obj1_time2/dt_sim); 
manouevre_obj1_time3 = 15;  manouevre_obj1_step3 = round(manouevre_obj1_time3/dt_sim);

current_vel = v1_seg1; 
true_trajectory_3D_1(:, 1) = initial_pos_1;

for i = 2 : num_steps

    if i == manouevre_obj1_step1, current_vel = v1_seg2; 
    elseif i == manouevre_obj1_step2, current_vel = v1_seg3; 
    elseif i == manouevre_obj1_step3, current_vel = v1_seg4; 
    end

    true_trajectory_3D_1(:, i) = true_trajectory_3D_1(:, i-1) + current_vel * dt_sim;
end

% Ground truth for Object 2 going at a constant velocity 
true_trajectory_3D_2 = zeros(3, num_steps);
initial_pos_2 = [5; -5; 8];

obj_2_const_velocity = [-2; 1; -0.8];

true_trajectory_3D_2(:, 1) = initial_pos_2;

for i = 2:num_steps
    true_trajectory_3D_2(:, i) = true_trajectory_3D_2(:, i-1) + obj_2_const_velocity * dt_sim;
end

% Virtual Camera Rig using look-at method 
K_cam1=[500, 0 , 960;
         0 ,500, 540;
         0 , 0 ,  1  ]; 

K_cam2=[510, 0 , 955;
         0 ,510, 545;
         0 , 0 ,  1  ];

image_width = 1920; image_height = 1080;

combined_traj = [true_trajectory_3D_1, true_trajectory_3D_2];
min_coords = min(combined_traj,[],2); 
max_coords = max(combined_traj,[],2);

look_at_point = ((min_coords + max_coords) / 2);

% Camera 1 
cam1_pos_GCF = [-15; -15; 1]; 
cam1_up_GCF  = [ 0; 0; 1];

Zc1 = (look_at_point - cam1_pos_GCF) / norm(look_at_point - cam1_pos_GCF); 
Xc1 = cross(cam1_up_GCF, Zc1) / norm(cross(cam1_up_GCF, Zc1)); 
Yc1 = cross(Zc1, Xc1);

R_GCF_to_Cam1 = [Xc1'; Yc1'; Zc1']; 
t_GCF_to_Cam1 = -R_GCF_to_Cam1 * cam1_pos_GCF;

% Camera 2 
cam2_pos_GCF = [-10; -20; 1.5];
cam2_up_GCF  = [ 0; 0; 1];

Zc2 = (look_at_point - cam2_pos_GCF) / norm(look_at_point - cam2_pos_GCF); 
Xc2 = cross(cam2_up_GCF, Zc2) / norm(cross(cam2_up_GCF, Zc2)); 
Yc2 = cross(Zc2, Xc2);

R_GCF_to_Cam2 = [Xc2';Yc2';Zc2']; 
t_GCF_to_Cam2 = -R_GCF_to_Cam2 * cam2_pos_GCF;


P_cam1 = K_cam1 * [R_GCF_to_Cam1, t_GCF_to_Cam1]; 
P_cam2 = K_cam2 * [R_GCF_to_Cam2, t_GCF_to_Cam2];
P0_default = diag([1,1,1,5,5,5]);
P_birth_default = diag([25, 25, 25, 100, 100, 100]); % High uncertainty for tracks being birthed new 

%% Section 2: Simulate Asynchronous Multi-Object Measurements
cam1_fr = 30; 
cam2_fr = 25; 

cam1_dt = 1 / cam1_fr; 
cam2_dt = 1 / cam2_fr;

measurement_log = []; m_noise_std = 1.0;

% For loop -> Simulate what Camera 1 sees at its unique frame rate 
for t = 0: cam1_dt: total_time

    % synchronise frame of true trajectory with camera frame rate 
    idx = min(max(round(t / dt_sim) + 1, 1), num_steps); 

    if (idx>num_steps) 
        continue; 
    end

    % true_trajectory_3D -> Each row contains x, y, z. Column is time
    % index, which is whats provided
    [uv1, obj1_in_cam1_fov] = project_point(true_trajectory_3D_1(:,idx), R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, ...
        image_width, image_height);

    % if object 1 visible for cam1 then add to measurement log 
    if obj1_in_cam1_fov
        % add new row: 
        % timestamp, camera id, add noise to the u,v coords, object id
        measurement_log = [measurement_log; t, 1, (uv1 + randn(2,1) * m_noise_std)', 1]; 
    end

    [uv2, obj2_in_cam1_fov] = project_point(true_trajectory_3D_2(:,idx), R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, ...
        image_width,image_height);

    % if object 2 visible for cam1 then add to measurement log 
    if obj2_in_cam1_fov 
        % add new row: 
        % timestamp, camera id, add noise to the u,v coords, object id
        measurement_log = [measurement_log; t, 1, (uv2 + randn(2,1) * m_noise_std)', 2]; 
    end

end

% Camera 2 
for t = 0: cam2_dt: total_time
    idx = min(max(round(t / dt_sim) + 1, 1), num_steps); 

    if (idx>num_steps) 
        continue; 
    end

    % if object 1 visible for cam2 then add to measurement log 
    [uv1, obj1_in_cam2_fov] = project_point(true_trajectory_3D_1(:,idx), R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, ...
        image_width, image_height);

    if obj1_in_cam2_fov 
        % add new row: 
        % timestamp, camera id, add noise to the u,v coords, object id
        measurement_log = [measurement_log; t, 2, (uv1 + randn(2,1) * m_noise_std)', 1]; 
    end

    % if object 2 visible for cam2 then add to measurement log 
    [uv2, obj2_in_cam2_fov] = project_point(true_trajectory_3D_2(:,idx), R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, ...
        image_width,image_height);

    if obj2_in_cam2_fov
        % add new row: 
        % timestamp, camera id, add noise to the u,v coords, object id
        measurement_log = [measurement_log; t, 2, (uv2 + randn(2,1) * m_noise_std)', 2]; 
    end

end

measurement_log = sortrows(measurement_log, 1);

if isempty(measurement_log)
    error('No measurements generated, check camera FOV or projection.');
end

%% Section 3: Multi-Object EKF Pipeline
% Track Management Setup 

% tracks -> Array that will hold each bird struct (state X, covariance P, etc.)
tracks = struct('id', {}, 'X', {}, 'P', {}, 'age', {}, 'missed_frames', {});

% this list will hold the first detection new object
% hits -> how many times tentative object was re-observed
% P_latest -> most recent 3D position observed. 
% T_latest -> timestamp for P_latest 
tentative_tracks = struct('P1', {}, 'T1', {}, 'object_id', {}, 'hits', {}, 'P_latest', {}, 'T_latest', {});
tentative_track_timeout = 1.0; % tentative track deleted if not confirmed within 0.5s

next_track_id = 1;

% threshold (pixel)
association_threshold = 100; 
max_missed_frames = 5;

R_ekf = diag([m_noise_std^2, m_noise_std^2]);

% Continuous tracker 
prediction_rate = 20; 
dt_pred = 1 / prediction_rate;

total_sim_time = measurement_log(end, 1);
num_pred_steps = floor(total_sim_time / dt_pred);

time_history = (0 : num_pred_steps - 1)' * dt_pred;
track_histories = {}; 

track_colours = containers.Map('KeyType', 'double', 'ValueType', 'any'); 

base_colours = lines(100); 
track_colours(1) = base_colours(1, :); 
track_colours(2) = base_colours(2, :); 

% maximum allowed delay between cam1 & cam2 when birthing new tracks
max_time_diff = 0.2;

max_dist_from_origin = 70; % 70 metres

% Velocity Check: 
max_possible_velocity = 40; % m/s; 

% Main Continuous Loop
for k = 1:num_pred_steps
    current_time = time_history(k);

    % 1. Predict all active tracks 
    for i = 1 : length(tracks)
        F_ekf = [1, 0, 0, dt_pred,  0    ,  0    ; 
                 0, 1, 0,   0    ,dt_pred,  0    ; 
                 0, 0, 1,   0    ,  0    ,dt_pred; 
                 0, 0, 0,   1    ,  0    ,  0    ; 
                 0, 0, 0,   0    ,  1    ,  0    ; 
                 0, 0, 0,   0    ,  0    ,  1     ];

        unmodeled_accel_std = 5.0; % m/s^2 
        q_pos = (0.5 * unmodeled_accel_std * dt_pred^2)^2;
        q_vel = (unmodeled_accel_std * dt_pred)^2;
        Q_ekf = diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel]); % How much the tracker trusts its own prediction 

        % each bird's new state and covaraince (stored in a unique track)
        % X = F * X 
        % P = F * P * F' + Q
        tracks(i).X = F_ekf * tracks(i).X;
        tracks(i).P = F_ekf * tracks(i).P * F_ekf' + Q_ekf;
    end

    % 2. Find measurements in this time slice
    meas_indices_in_slice = find(measurement_log(:,1) <= current_time & ...
        measurement_log(:,1) > (current_time - dt_pred));

    current_measurements = measurement_log(meas_indices_in_slice, :);
    num_current_measurements = size(current_measurements, 1);

    % 3. Data Association (Nearest Neighbour) 

    % associations -> will indicate which measurement will go on with which
    % track. for each track with a valid measurement, do the EKF update
    % with that measurement 
    associations = zeros(length(tracks), 1); 
    if ~isempty(tracks) && num_current_measurements > 0

        % cost matrix shows how well each detection matches each track 
        % row -> each track 
        % column -> each measurement 
        cost_matrix = inf(length(tracks), num_current_measurements); 

        % for loops -> for every track-measurement pair, project the tracks
        % 3D predicted position into the u,v image plane. 
        % find the distance between the predicted pixel and the measured pixel  
        for i = 1:length(tracks)
            for j = 1:num_current_measurements
                cam_id = current_measurements(j, 2);
                if cam_id == 1 
                    [pred_uv, ~] = project_point(tracks(i).X(1:3), R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, image_width, image_height);
                else 
                    % if cam_id == 2
                    [pred_uv, ~] = project_point(tracks(i).X(1:3), R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, image_width, image_height); 
                end

                if ~any(isnan(pred_uv)) 
                    cost_matrix(i,j) = norm(pred_uv - current_measurements(j, 3:4)'); 
                end

            end
        end

        % Assign Measurements to Tracks 
        % (match each predicted track with closest measurement, WITHOUT
        % re-using a measurement). 

        % length(tracks) -> no. of currently active birds being tracked
        % num_current_meas -> no. of new birds detected at the time step
        % find the min. because some tracks cannot be paired with
        % measurements or some measurements cannot be paired with tracks
        for i = 1:min(length(tracks), num_current_measurements)

            % cost_matrix, each entry -> distance betw. track i's
            % predicted position and measurement j 
            % find smallest distance between current track and all detected
            % measurements.
            [min_val, min_idx] = min(cost_matrix(:));
            if min_val > association_threshold 
                break; 
            end

            % min_idx -> single linear index 
            % ind2sub converts it to matrix row and column indices
            % row index -> which track the best match belongs to.
            % column index -> which measurement it corresponds to.
            [track_idx, measurement_idx] = ind2sub(size(cost_matrix), min_idx);

            % track no. track_idx has been associated with measurement no.
            % measurement_idx 
            associations(track_idx) = measurement_idx;

            % make the track row inf and measurement idx column inf to not
            % re-use the measurement 
            cost_matrix(track_idx, :) = inf; 
            cost_matrix(:, measurement_idx) = inf; 
        end
    end 
        
    % 4. Update Associated Tracks
    % for each track, this mask will track whether each measurement was
    % assigned a track in the below for loop 
    unassigned_meas_mask = true(num_current_measurements, 1);
    tracks_to_keep = true(length(tracks), 1); 

    % Update associated tracks 
    for i = 1:length(tracks)
        measurement_idx = associations(i);
        if measurement_idx > 0 % Track was associated
            cam_id = current_measurements(measurement_idx, 2);

            % Measured u, v
            Z_m = current_measurements(measurement_idx, 3:4)';

            if cam_id == 1 
                [X_update,P_update] = EKF_Update_Step(tracks(i).X, tracks(i).P, Z_m, R_ekf, K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1);
            else 
                [X_update,P_update] = EKF_Update_Step(tracks(i).X, tracks(i).P, Z_m, R_ekf, K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2); 
            end

            tracks(i).X = X_update; 
            tracks(i).P = P_update;
            tracks(i).missed_frames = 0;
            unassigned_meas_mask(measurement_idx) = false;

        else % Track was not associated
            tracks(i).missed_frames = tracks(i).missed_frames + 1;
        end

        % Mark Tracks for deletion (death)
        if tracks(i).missed_frames >= max_missed_frames 
            tracks_to_keep(i) = false; 
        end 

     end

    % Perform Deletion
    tracks = tracks(tracks_to_keep);
    
    % 5. Track Birth: 
    % Two-point initialization strategy.

    % gets list of all measurements from curr. time slice not matched to
    % any existing tracks. (Orphans)
    unassigned_meas_indices = find(unassigned_meas_mask);

    % A. Find all valid triangulated pairs from the unassigned measurements

    % create a list of false values, one for each orphan. 
    % When orphan successfully used to create a point (either tentative or full),
    % make entry true. No same measurement used twice.
    processed_indices = false(length(unassigned_meas_indices), 1);

     
    % The outer loop (i) picks an orphan measurement. 
    % The inner loop (j = i+1) searches through all remaining orphans to find a suitable partner. 
    % The i+1 means no pairs already considered are checked. 
    for i = 1:length(unassigned_meas_indices)
        % Skip if already paired
        if processed_indices(i) 
            continue; 
        end 

        % Get orphan measurement index and its details
        primary_idx = unassigned_meas_indices(i);
        primary_meas = current_measurements(primary_idx, :);
        
        % Look for a partner orphan measurement
        for j = (i + 1):length(unassigned_meas_indices)

            if processed_indices(j) 
                continue; 
            end

            % Gete POTENTIAL partner orphan measurment and its details
            partner_idx = unassigned_meas_indices(j);
            partner_meas = current_measurements(partner_idx, :);

            % Check for a valid pair 
            % (First check is confirming it is different cameras 
            % and second check is absolute time difference)
            if partner_meas(2) ~= primary_meas(2) && abs(partner_meas(1) - primary_meas(1)) < max_time_diff
                
                % Valid pair found, now triangulate

                % if orphan was from camera 1,
                % cam1 measurement -> Orphan 
                % cam2 measurement -> Partner Orphan
                if primary_meas(2) == 1
                    meas_cam1 = primary_meas; meas_cam2 = partner_meas;
                else
                    meas_cam1 = partner_meas; meas_cam2 = primary_meas;
                end
                
                P_3d = triangulate(meas_cam1(3:4), meas_cam2(3:4), P_cam1', P_cam2')';

                if norm(P_3d) > max_dist_from_origin
                    % point too far away, discard. 
                    continue; % Skip to next loop iteration
                end

                % We now have a VALID 3D point. What to do with it?  

                % Cheating for simulation: use true object ID to match tentative points
                object_id = meas_cam1(5); 
                
                % B. Check if this point can complete a tentative track

                % was_handled flag -> A way to remember if the new 3D point
                % was used to confirm an existing tentative track 
                was_handled = false;

                for tt = 1:length(tentative_tracks)
                    if tentative_tracks(tt).object_id == object_id
                        tentative_tracks(tt).hits = tentative_tracks(tt).hits + 1;
                        tentative_tracks(tt).P_latest = P_3d;
                        tentative_tracks(tt).T_latest = meas_cam1(1);
                
                        % Promote after 4 hits
                        if tentative_tracks(tt).hits >= 4
                            P1 = tentative_tracks(tt).P1; T1 = tentative_tracks(tt).T1;
                            P2 = tentative_tracks(tt).P_latest; T2 = tentative_tracks(tt).T_latest;
                            dt_vel = T2 - T1;
                            if dt_vel > 1e-3
                                V0 = (P2 - P1) / dt_vel;
                                if norm(V0) > max_possible_velocity
                                    % 1. Reason for Deletion of TT
                                    % track: Impossible Velocity 
                                    tentative_tracks(tt) = []; 
                                    was_handled = true; 
                                    break;   
                                end

                                new_track.id = next_track_id;
                                new_track.X = [P2; V0];
                                new_track.P = P_birth_default;
                                new_track.age = 0;
                                new_track.missed_frames = 0;
                                tracks(end + 1) = new_track;
                                next_track_id = next_track_id + 1;
                                if ~isKey(track_colours, new_track.id)
                                    track_colours(new_track.id) = base_colours(mod(new_track.id - 1, 100) + 1, :);
                                end
                                % 2. Reason for Deletion of TT track:
                                % Succesful FULL track birth
                                tentative_tracks(tt) = [];
                            end
                        end
                
                        was_handled = true;
                        break;
                    end
                end
                
                % C. If the 3D point was not used to update an existing tentative track, 
                % it's the FIRST point for a potentially newly detected object
                if ~was_handled
                    new_tentative.P1 = P_3d;
                    % get timestamp of detected 3d pt. from either camera
                    new_tentative.T1 = meas_cam1(1);
                    new_tentative.object_id = object_id;
                    new_tentative.hits = 1;
                    new_tentative.P_latest = P_3d;
                    new_tentative.T_latest = meas_cam1(1);
                    tentative_tracks(end + 1) = new_tentative;
                    % This track is put in probation. 
                end

                % Mark both measurements as processed
                % Successfully used (i & j) pair of measurements to promote a track or 
                % create a new tentative one. 
                % Mark them as "processed" so as not to use them again in this time slice. 
                processed_indices(i) = true;
                processed_indices(j) = true;

                break; % Stop searching for a partner for primary_meas (already found or list exhausted)
            end
        end
    end

    % D. Clean up old, unconfirmed tentative tracks
    stale_tracks_mask = false(length(tentative_tracks), 1);
    for tt = 1:length(tentative_tracks)
        if current_time - tentative_tracks(tt).T1 > tentative_track_timeout
            stale_tracks_mask(tt) = true;
        end
    end

    % If tentative tracks is NOT empty
    if ~isempty(tentative_tracks)
        % perform deletion of the expired tentative tracks (noise)
        tentative_tracks(stale_tracks_mask) = [];
    end

    % 6. Store history of all active tracks 
    for i = 1:length(tracks)
        track_id = tracks(i).id;
        if track_id > numel(track_histories) || isempty(track_histories{track_id})
            track_histories{track_id} = [time_history(k), tracks(i).X'];
        else 
            track_histories{track_id} = [track_histories{track_id}; time_history(k), tracks(i).X'];
        end
    end
end

%% Section 4: Visualisation

figure(1); clf; hold on;
plot3(true_trajectory_3D_1(1,:), true_trajectory_3D_1(2,:), true_trajectory_3D_1(3,:), 'b-', 'LineWidth', 2, ...
    'DisplayName', 'True Path 1');
plot3(true_trajectory_3D_2(1,:), true_trajectory_3D_2(2,:), true_trajectory_3D_2(3,:), 'g-', 'LineWidth', 2, ...
    'DisplayName', 'True Path 2');

valid_ids = find(~cellfun(@isempty, track_histories)); 
num_valid = numel(valid_ids); 

for idx = 1:num_valid
    % actual track_id (e.g. 1, 3, 8, 12)
    track_id = valid_ids(idx); 
    history = track_histories{track_id};

    % return true if track_colours contain an entry for current track_id
    if isKey(track_colours, track_id) 
        % retrieve the value (RGB triplet) stored in the map for this track_id
        colour = track_colours(track_id); 
    else 
        % if track id wasn't assigned a colour (rare case)
        colour = [0.5 0.5 0.5];
    end
    % history columns -> [time, x, y, z, vx, vy, vz] 
    plot3(history(:,2), history(:,3), history(:,4), '.-', 'Color', colour, ...
        'DisplayName', ['Track ' num2str(track_id)]);
end

legend('show', 'Location', 'best'); grid on; axis equal; 
title('Multi-Object EKF Tracking'); 
view(30,20);

%% Helper Functions

% project_point acts as a virtual camera, takes a 3D traj point and
% projects it into (u,v) coordinates. 
% Since using real data this function is unused here. 
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