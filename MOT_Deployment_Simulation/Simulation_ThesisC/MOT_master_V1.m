% December 19th 2025. 
% Gain thorough understanding of each section of code before IMM. 

%% Section 1: Define Ground Truth & Camera Rig
clear; clc; close all;

% Simulation time 
total_time = 80;
dt_sim = 0.01;
time_vector = 0: dt_sim: total_time;
num_steps = length(time_vector);

% Ground Truth for Object 1 (manouevring)
true_trajectory_3D_1 = zeros(3, num_steps);
initial_pos_1 = [-10; 40; 3];

% multi-manouevre 
v1_seg1 = [-5; 0; 0.2];    % 1. Go left horizontal to Camera FOV
v1_seg2 = [-5; -3; 4];     % 2. Fly up diagonally and go behind camera FOV
v1_seg3 = [-3; -2; -4];    % 3. Come down diagonally (still behind camera)
v1_seg4 = [5; 5; -0.1];    % 4. Go back diagonally back into camera FOV
v1_seg5 = [2; 2; 4];       % 5. Once well within camera FOV, go up diagonally
v1_seg6 = [1; 2; -3];      % 6. Come down diagonally 
v1_seg7 = [1; 1; 0];       % 7. Go in a straight line at the end for 20s

manouevre_obj1_time1 = 3; 
manouevre_obj1_time2 = 10;  
manouevre_obj1_time3 = 16;  
manouevre_obj1_time4 = 28; 
manouevre_obj1_time5 = 48;
manouevre_obj1_time6 = 60;

current_vel = v1_seg1; 
true_trajectory_3D_1(:, 1) = initial_pos_1;

for i = 2:num_steps
    current_time = time_vector(i);
    % current_vel = v1_seg1; 
    
    if current_time >= manouevre_obj1_time1
        current_vel = v1_seg2;
    end

    if current_time >= manouevre_obj1_time2
        current_vel = v1_seg3;
    end

    if current_time >= manouevre_obj1_time3
        current_vel = v1_seg4; 
    end

    if current_time >= manouevre_obj1_time4
        current_vel = v1_seg5;
    end

    if current_time >= manouevre_obj1_time5
        current_vel = v1_seg6; 
    end

    if current_time >= manouevre_obj1_time6
        current_vel = v1_seg7;
    end
    
    true_trajectory_3D_1(:, i) = true_trajectory_3D_1(:, i-1) + current_vel * dt_sim;
end

% Ground truth for Object 2 going at a constant velocity 
true_trajectory_3D_2 = zeros(3, num_steps);
initial_pos_2 = [0; 50; 6];
true_trajectory_3D_2(:, 1) = initial_pos_2;

% Define velocity segments
v2_seg1 = [3; 0; 0.3];     % 1. Exit
v2_seg2 = [0; 3; -0.1];    % 2. Off camera, turn 90 degrees left and move forward. 
v2_seg3 = [-4; 0; -0.3];   % 3. Turn left, come back into view. 

% Define time points Object 2 trajectory 
obj2_seg1_time = 35.0; 
obj2_seg2_time = 45.0;

for i = 2:num_steps
    current_time = time_vector(i);
    
    if current_time < obj2_seg1_time
        current_vel = v2_seg1; 

    elseif current_time >= obj2_seg1_time && current_time < obj2_seg2_time
        current_vel = v2_seg2; 

    elseif current_time > obj2_seg2_time
        current_vel = v2_seg3; % Fly back in
    end
    
    true_trajectory_3D_2(:, i) = true_trajectory_3D_2(:, i-1) + current_vel * dt_sim;
end

% Constructing Camera 1 & 2's Coordinate Frame
% Lens Properties (K) 
K_cam1 = [500,  0 , 960;
           0 , 500, 540;
           0 ,  0 ,  1  ]; 

K_cam2 = [510,  0 , 955;
           0 , 510, 545;
           0 ,  0 ,  1  ];

image_width = 1920; image_height = 1080;

% Look at point is fixed, not dynamic. Represents the physical camera rig. 
look_at_point = [0; 80; 5];

% Camera 1 
cam1_pos_GCF = [20; 0; 1]; 
cam1_up_GCF  = [ 0; 0; 1];

% Z -> vector from camera to target. 
% X -> cross product of 2 vecs produces new vector perpendicular to 2 vecs.
% Hence X is horizontal direction to camera. 
% Y -> Z is forward. X is right. Y is down. So cross product of X and Z is
% down. 
Zc1 = (look_at_point - cam1_pos_GCF) / norm(look_at_point - cam1_pos_GCF); 
Xc1 = cross(cam1_up_GCF, Zc1) / norm(cross(cam1_up_GCF, Zc1)); 
Yc1 = cross(Zc1, Xc1);

% Camera Position (R & T)

% R -> Rotates world so that global axes align with camera axes. 
% T -> How far world origin has to move to align with camera's origin AFTER
% rotation applied. 
R_GCF_to_Cam1 = [Xc1'; Yc1'; Zc1']; 
t_GCF_to_Cam1 = -R_GCF_to_Cam1 * cam1_pos_GCF;

% Camera 2 
cam2_pos_GCF = [-20; 0; 1];
cam2_up_GCF  = [ 0; 0; 1];

Zc2 = (look_at_point - cam2_pos_GCF) / norm(look_at_point - cam2_pos_GCF); 
Xc2 = cross(cam2_up_GCF, Zc2) / norm(cross(cam2_up_GCF, Zc2)); 
Yc2 = cross(Zc2, Xc2);

R_GCF_to_Cam2 = [Xc2';Yc2';Zc2']; 
t_GCF_to_Cam2 = -R_GCF_to_Cam2 * cam2_pos_GCF;

% Camera Projection Matrix (3 x 4)  
% 3D Point onto 2D Image Sensor
P_cam1 = K_cam1 * [R_GCF_to_Cam1, t_GCF_to_Cam1]; 
P_cam2 = K_cam2 * [R_GCF_to_Cam2, t_GCF_to_Cam2];
P_birth_default = diag([25, 25, 25, 100, 100, 100]); % High uncertainty for tracks being birthed new 

%% Section 2: Simulate Asynchronous Multi-Object Measurements
cam1_fr = 30; 
cam2_fr = 25; 

cam1_dt = 1 / cam1_fr; 
cam2_dt = 1 / cam2_fr;

measurement_log = []; m_noise_std = 1.0;

% For loop -> Simulate what Camera 1 sees at its unique frame rate 
for t_cam1 = 0: cam1_dt: total_time

    % Aligning two timelines: 
    % Physics: Ground truth exists at high frequency (0.01s)
    % Camera: Cam captures frames at lower, specific frequency (~0.033s).
    % At the exact moment the camera opened, which row in 'Simulation time' best represents the world?
    idx = min(max(round(t_cam1 / dt_sim) + 1, 1), num_steps); 

    % true_trajectory_3D -> Each row contains either x, y or z. 
    % Column is time index, which is whats provided
    [uv1, obj1_in_cam1_fov] = project_point(true_trajectory_3D_1(:, idx), R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, ...
        image_width, image_height);

    % if object 1 visible for cam1 then add to measurement log 
    if obj1_in_cam1_fov
        % add new row: 
        % timestamp, camera id, add noise to the u,v coords, object id
        % transpose is to add everything horizontally. 
        measurement_log = [measurement_log; t_cam1, 1, (uv1 + randn(2,1) * m_noise_std)', 1]; 
    end

    [uv2, obj2_in_cam1_fov] = project_point(true_trajectory_3D_2(:,idx), R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, ...
        image_width,image_height);

    % if object 2 visible for cam1 then add to measurement log 
    if obj2_in_cam1_fov 
        % add new row: 
        % [timestamp, camera id, add noise to the u coord, add noise to v coord, object id]
        measurement_log = [measurement_log; t_cam1, 1, (uv2 + randn(2,1) * m_noise_std)', 2]; 
    end

end

% Camera 2 
for t_cam2 = 0: cam2_dt: total_time

    idx = min(max(round(t_cam2 / dt_sim) + 1, 1), num_steps); 

    % if object 1 visible for cam2 then add to measurement log 
    [uv1, obj1_in_cam2_fov] = project_point(true_trajectory_3D_1(:,idx), R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, ...
        image_width, image_height);

    if obj1_in_cam2_fov 
        % add new row: 
        % timestamp, camera id, add noise to the u,v coords, object id
        measurement_log = [measurement_log; t_cam2, 2, (uv1 + randn(2,1) * m_noise_std)', 1]; 
    end

    % if object 2 visible for cam2 then add to measurement log 
    [uv2, obj2_in_cam2_fov] = project_point(true_trajectory_3D_2(:,idx), R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, ...
        image_width,image_height);

    if obj2_in_cam2_fov
        % add new row: 
        %  [timestamp, camera id, add noise to the u coord, add noise to v coord, object id]
        measurement_log = [measurement_log; t_cam2, 2, (uv2 + randn(2,1) * m_noise_std)', 2]; 
    end

end

% sort rows according to timestamp in measurement log 
measurement_log = sortrows(measurement_log, 1);

if isempty(measurement_log)
    error('No measurements generated, check camera FOV or projection.');
end

%% Section 3: Multi-Object EKF Pipeline
% Track Management Setup 

% tracks -> Array that will hold each bird struct (state X, covariance P, etc.)
tracks = struct('id', {}, 'X', {}, 'P', {}, 'age', {}, 'missed_frames', {});

% Tentative holds first detection new object
% hits -> how many times tentative object was re-observed
% P_latest -> most recent 3D position observed. 
% T_latest -> timestamp for P_latest 
tentative_tracks = struct('Pos_first', {}, 'Timestamp_first', {}, 'object_id', {}, 'hits', {}, 'Pos_latest', {}, 'Timestamp_latest', {});
tentative_track_timeout = 3; % tentative track deleted if another hit not seen within 3s

next_track_id = 1;

% threshold (pixel)
association_threshold = 100; 
max_missed_frames = 30;

R_ekf = diag([m_noise_std^2, m_noise_std^2]);

% Continuous tracker 
prediction_rate = 20; 
dt_pred = 1 / prediction_rate;

total_sim_time = measurement_log(end, 1);
num_pred_steps = floor(total_sim_time / dt_pred);

% vertical list of timestamps for every iteration in the tracker loop 
time_history = (0 : num_pred_steps - 1)' * dt_pred;

% Cell array to store the evolution of a bird track over the whole main
% loop
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

    % 1. Predict all active tracks from t-1 to t (this exact moment)  
    for i = 1 : length(tracks)
        F_ekf = [1, 0, 0, dt_pred,  0    ,  0    ; 
                 0, 1, 0,   0    ,dt_pred,  0    ; 
                 0, 0, 1,   0    ,  0    ,dt_pred; 
                 0, 0, 0,   1    ,  0    ,  0    ; 
                 0, 0, 0,   0    ,  1    ,  0    ; 
                 0, 0, 0,   0    ,  0    ,  1     ];

        unmodeled_accel_std = 4.0; % m/s^2 
        q_pos = (0.5 * unmodeled_accel_std * dt_pred^2)^2;
        q_vel = (unmodeled_accel_std * dt_pred)^2;
        Q_ekf = diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel]); % How much the tracker trusts its own prediction 

        % each bird's new state and covariance (stored in a unique track)
        % X = F * X 
        % P = F * P * F' + Q
        tracks(i).X = F_ekf * tracks(i).X;
        tracks(i).P = F_ekf * tracks(i).P * F_ekf' + Q_ekf;
    end

    % 2. Find measurements in this time slice
    current_meas_indices = find(measurement_log(:,1) <= current_time & ...
        measurement_log(:,1) > (current_time - dt_pred));

    current_measurements = measurement_log(current_meas_indices, :);
    num_current_measurements = size(current_measurements, 1);

    % 3. Data Association (Nearest Neighbour) 

    % associations -> will indicate which measurement will go on with which
    % track. for each track with a valid measurement, do the EKF update
    % with that measurement 
    associations = zeros(length(tracks), 1); 
    if ~isempty(tracks) && num_current_measurements > 0

        % cost matrix shows how well each measurement matches each track 
        % row -> each track 
        % column -> each measurement 
        cost_matrix = inf(length(tracks), num_current_measurements); 

        % for loops -> for every track-measurement pair, project the tracks
        % 3D predicted position into the u,v image plane. 
        % find the distance between the predicted pixel and the measured pixel  
        for i = 1:length(tracks)
            for j = 1:num_current_measurements
                cam_id = current_measurements(j, 2); % (2 is the camerae_id column)
                if cam_id == 1 
                    [pred_uv, ~] = project_point(tracks(i).X(1:3), R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, image_width, image_height);
                else 
                    % if cam_id == 2
                    [pred_uv, ~] = project_point(tracks(i).X(1:3), R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, image_width, image_height); 
                end

                if ~any(isnan(pred_uv)) 
                    cost_matrix(i,j) = norm(pred_uv - current_measurements(j, 3:4)'); % 3:4 is getting u,v coords
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
            % min_val -> absolute best match left in entire cost matrix
            % (changes iteratively) 
            [min_val, min_idx] = min(cost_matrix(:));
            
            if min_val > association_threshold
                % If min dist. too large, every remaining distance is worse.
                % No more valid pairs exist, so break out of for loop. 
                break; 
            end

            % Get track and measurement no. of this minimum value index
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

    % gets list of all measurement indexes from curr. time slice not matched to
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

            % If potential partner already processed, go to next potential
            % partner. 
            if processed_indices(j) 
                continue; 
            end

            % Get POTENTIAL partner orphan measurment and its details
            partner_idx = unassigned_meas_indices(j);
            partner_meas = current_measurements(partner_idx, :);

            % Check for a valid pair 
            % (First check is confirming it is different cameras 
            % and second check is absolute time difference)
            if (partner_meas(2) ~= primary_meas(2)) && (abs(partner_meas(1) - primary_meas(1)) < max_time_diff)
                
                % Valid pair found, now triangulate

                % If first orphan was from Camera 1,
                % cam1 measurement -> Orphan 
                % cam2 measurement -> Partner Orphan
                if primary_meas(2) == 1
                    meas_cam1 = primary_meas; meas_cam2 = partner_meas;
                else
                    meas_cam1 = partner_meas; meas_cam2 = primary_meas;
                end
                
                Point_3d = triangulate(meas_cam1(3:4), meas_cam2(3:4), P_cam1, P_cam2)';

                if norm(Point_3d) > max_dist_from_origin
                    % point too far away, discard. 
                    continue; % Skip to next loop iteration
                end

                % Valid 3D point obtained

                % Cheating for simulation: use true object ID to match tentative points
                object_id = meas_cam1(5); 
                
                % B. Check IF THIS POINT can match to a TENTATIVE track. 

                % was_handled flag -> A way to remember if this 3D point
                % was used to confirm an existing tentative object or
                % whether it's a part of a new detected object 
                was_point_3d_handled = false;

                for tt = 1:length(tentative_tracks)

                    if tentative_tracks(tt).object_id == object_id

                        tentative_tracks(tt).hits = tentative_tracks(tt).hits + 1;
                        tentative_tracks(tt).Pos_latest = Point_3d;
                        tentative_tracks(tt).Timestamp_latest = meas_cam1(1);
                
                        % Promote after 4 hits
                        if tentative_tracks(tt).hits >= 4

                            Point1 = tentative_tracks(tt).Pos_first; Time1 = tentative_tracks(tt).Timestamp_first;
                            Point2 = tentative_tracks(tt).Pos_latest; Time2 = tentative_tracks(tt).Timestamp_latest;

                            dt_vel = Time2 - Time1;

                            if dt_vel > 1e-3

                                V0 = (Point2 - Point1) / dt_vel;

                                if norm(V0) > max_possible_velocity
                                    % 1. Reason for Deletion of TT
                                    % track: Impossible Velocity 
                                    tentative_tracks(tt) = []; 
                                    was_point_3d_handled = true; 
                                    break;   
                                end

                                new_track.id    = next_track_id;
                                new_track.X     = [Point2; V0];
                                new_track.P     = P_birth_default;
                                new_track.age   = 0;
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
                
                        was_point_3d_handled = true;
                        break; 
                        % 'break' once a match (update tt, promote to full
                        % track or delete tt due to vel) is found to stop searching. 
                    end
                end
                
                % C. If the 3D point was NOT used to update an existing tentative track, 
                % it's the FIRST point for a potentially newly detected object
                if ~was_point_3d_handled

                    new_tentative.Pos_first = Point_3d;
                    % get timestamp of detected 3d pt. from either camera
                    new_tentative.Timestamp_first = meas_cam1(1);
                    new_tentative.object_id = object_id;
                    new_tentative.hits = 1;
                    new_tentative.Pos_latest = Point_3d;
                    new_tentative.Timestamp_latest = meas_cam1(1);
                    tentative_tracks(end + 1) = new_tentative;
                    % This track is put in probation. 
                end

                % Mark both measurements as processed
                % Successfully used (i & j) pair of measurements to promote a track or 
                % create a new tentative one. 
                % Mark them as "processed" so as not to use them again in this time slice. 
                processed_indices(i) = true;
                processed_indices(j) = true;

                % Why 'break'?
                % Currently inside the Inner Loop (j), which is searching 
                % for a partner for the Outer Loop orphan (i).
                % If no break, Measurement i pairs with Measurement j (creates a 3D point).
                % and loop still continues and Measurement i might pair again 
                % with Measurement k (creates another 3D point).

                % ONE  detection cannot create two different birds

                break; 
            end
        end
    end

    % D. Clean up old, unconfirmed tentative tracks
    stale_tracks_mask = false(length(tentative_tracks), 1);
    for tt = 1:length(tentative_tracks)

        % Bird flying through lens should get 4 hits within 3 seconds (~1.3 hits/s)
        % so use timestamp_first
        if (current_time - tentative_tracks(tt).Timestamp_first) > tentative_track_timeout
            stale_tracks_mask(tt) = true;
        end
    end

    % If more than zero tentative_tracks exist currently in this time slice
    if ~isempty(tentative_tracks)
        % perform deletion of the expired tentative tracks (noise)
        tentative_tracks(stale_tracks_mask) = [];
    end

    % 6. Store all the history of active tracks throughout the run-time
    for i = 1:length(tracks)
        track_id = tracks(i).id;
        % First check -> creates a new slot for a new bird 
        % Second check -> Checks if slot exists but is empty 
        if track_id > numel(track_histories) || isempty(track_histories{track_id})
            track_histories{track_id} = [current_time, tracks(i).X'];

        else % take existing history for Bird #5 (100 rows) and add curr. state (101st row at the bottom).
            track_histories{track_id} = [track_histories{track_id}; current_time, tracks(i).X'];
        end
    end
end

%% Section 4: Visualisation

figure(1); clf; hold on;

% A) Visualise cameras and field of view 

% Plot Camera Positions
plot3(cam1_pos_GCF(1), cam1_pos_GCF(2), cam1_pos_GCF(3), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Camera 1');
plot3(cam2_pos_GCF(1), cam2_pos_GCF(2), cam2_pos_GCF(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Camera 2');

% Function to plot a camera's FOV (add this helper function at the end of your script)
plot_camera_fov(cam1_pos_GCF, R_GCF_to_Cam1, K_cam1, image_width, image_height, [0.6, 0, 0]);
plot_camera_fov(cam2_pos_GCF, R_GCF_to_Cam2, K_cam2, image_width, image_height, [0.1, 0.1, 0.1]);

% B) Plot Ground Truth Trajectories 
plot3(true_trajectory_3D_1(1,:), true_trajectory_3D_1(2,:), true_trajectory_3D_1(3,:), 'b-', 'LineWidth', 2, ...
    'DisplayName', 'True Path 1');
plot3(true_trajectory_3D_2(1,:), true_trajectory_3D_2(2,:), true_trajectory_3D_2(3,:), 'g-', 'LineWidth', 2, ...
    'DisplayName', 'True Path 2');

% C) Plot the tracks 
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

        % 3. Get x and y coords of curr. camera point by using PERSPECTIVE
        % DIVISION 
        x_norm = point_camera(1) / point_camera(3);
        y_norm = point_camera(2) / point_camera(3);

        % 4. Find the corresponding homogeneous pixel points (u, v) of this
        % camera point BY multiplying x norm, y norm by K matrix. 

        % point projected onto a normalised image plane 
        % this normalised image plane has a distance 1 metre to camera lens
        % (this is what is meant by the 1, it is there to make the matrix multiplication)
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

% Draws the field of view of each camera on the plot. 
function plot_camera_fov(cam_pos_GCF, R_GCF_to_Cam, K_cam, width, height, line_colour)

    line_style = ':';             % Dotted line

    % Define 4 corners of the image plane (pixels)
    corners_pixels = [0, width, width ,   0;
                      0,   0  , height, height;
                      1,   1  ,   1   ,   1];
    
    % Convert pixel corners to normalised image coordinates in camera frame
    corners_cam_norm = (K_cam \ corners_pixels);
    
    % Draw the field of view length up to 30 meters out
    fov_length = 30; % 
    
    % Scale the corners to create 3D points in the camera frame
    corners_cam_3d = corners_cam_norm * fov_length;
    
    % Transform these 3D points from the Camera Frame back to the Global Frame
    R_Cam_to_GCF = R_GCF_to_Cam'; 
    t_Cam_to_GCF = -R_Cam_to_GCF * (-R_GCF_to_Cam * cam_pos_GCF);
    
    % Camera corners in GCF coords
    corners_GCF = R_Cam_to_GCF * corners_cam_3d + t_Cam_to_GCF;
    
    % Draw the lines from the camera position to the corners of the frustum
    for i = 1:4
        plot3([cam_pos_GCF(1), corners_GCF(1, i)], ...
              [cam_pos_GCF(2), corners_GCF(2, i)], ...
              [cam_pos_GCF(3), corners_GCF(3, i)], 'Color', line_colour, ...
              'LineStyle', line_style, 'LineWidth', 1, 'HandleVisibility', 'off');
    end
    
    % Draw the rectangle at the end of the frustum
    plot3([corners_GCF(1,:), corners_GCF(1,1)], ...
          [corners_GCF(2,:), corners_GCF(2,1)], ...
          [corners_GCF(3,:), corners_GCF(3,1)], 'Color', line_colour, ...
          'LineStyle', line_style, 'LineWidth', 1, 'HandleVisibility', 'off');
end