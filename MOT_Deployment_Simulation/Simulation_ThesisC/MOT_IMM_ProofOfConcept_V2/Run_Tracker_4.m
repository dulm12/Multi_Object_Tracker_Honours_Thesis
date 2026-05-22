function [tracks, track_histories, track_colours] = Run_Tracker_4(measurement_log, measurement_noise_std, ...
                                                 R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, ...
                                                 R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, ... 
                                                 Projection_Cam1, Projection_Cam2, ...
                                                 P_birth_default, tracker_params)

    % Unpack Tracking parameters: 
    IMM_TPM = tracker_params.IMM_TPM;
    tentative_track_timeout = tracker_params.tentative_track_timeout; % tentative track deleted if another hit not seen within timeout
    association_threshold = tracker_params.association_threshold; % Pixels
    max_missed_frames = tracker_params.max_missed_frames;
    max_time_diff = tracker_params.max_time_diff; % maximum allowed delay between cam1 & cam2 when birthing new tracks
    max_dist_from_origin = tracker_params.max_dist_from_origin; % metres
    max_possible_velocity = tracker_params.max_possible_velocity; % m/s

    chi2_threshold = chi2inv(tracker_params.chi2_probability, 2); % Obtain chi2 threshold from probability. 99% confidence gate, 2 because 2 DOF for (u,v)

    % Track Management Setup 
    
    % IMM track structure -> Array that will hold each bird struct (state X, covariance P, etc.)
    tracks = struct('id', {}, 'X', {}, 'P', {}, ...
                    'X_cv', {}, 'P_cv', {}, ...
                    'X_ca', {}, 'P_ca', {}, ...
                    'mu', {}, 'c_bar', {}, 'age', {}, 'missed_frames', {});
    
    % Tentative holds first detection of new object
    % hits -> how many times tentative object was re-observed
    % Pos_latest -> most recent 3D position observed. 
    % Timestamp_latest -> timestamp for Pos_latest 
    tentative_tracks = struct('Pos_first', {}, 'Timestamp_first', {}, 'object_id', {}, 'hits', {}, 'Pos_latest', {}, 'Timestamp_latest', {});
    
    next_track_id = 1;
    
    R_ekf = diag([measurement_noise_std^2, measurement_noise_std^2]);
    
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
    
    % Main Continuous Loop
    for k = 1:num_pred_steps
        current_time = time_history(k);
    
        % 1. Predict all active tracks from t-1 to t (this exact moment)  
        tracks = Run_Tracker_Predict_4_1(tracks, IMM_TPM, dt_pred);
    
        % 2. Find measurements in this time slice
        current_meas_indices = find(measurement_log(:,1) <= current_time & ...
            measurement_log(:,1) > (current_time - dt_pred));
    
        current_measurements = measurement_log(current_meas_indices, :);
        num_current_measurements = size(current_measurements, 1);
    
        % 3. Data Association (Nearest Neighbour) 
        associations = Run_Tracker_Association_4_2( ...
            tracks, current_measurements, num_current_measurements, ...
            K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
            K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
            R_ekf, association_threshold, chi2_threshold);
            
        % 4. Update Associated Tracks
        [tracks, unassigned_meas_mask, tracks_to_keep] = Run_Tracker_Update_4_3( ...
            tracks, associations, current_measurements, num_current_measurements, ...
            K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
            K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
            R_ekf, max_missed_frames, max_dist_from_origin, max_possible_velocity);
    
        % Perform Deletion
        tracks = Run_Tracker_Delete_4_4(tracks, tracks_to_keep);
    
        % Step 4.5 gate sweep
        unassigned_meas_mask = Run_Tracker_GateSweep_4_5( ...
            tracks, current_measurements, num_current_measurements, ...
            K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
            K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
            R_ekf, chi2_threshold, association_threshold, unassigned_meas_mask);
        
        % 5. Track Birth: 
        % Two-point initialization strategy.
        [tentative_tracks, tracks, next_track_id, track_colours] = ...
            Run_Tracker_Birth_4_6( ...
                tracks, tentative_tracks, next_track_id, track_colours, ...
                current_measurements, unassigned_meas_mask, ...
                base_colours, P_birth_default, IMM_TPM, ...
                max_time_diff, max_dist_from_origin, max_possible_velocity, ...
                current_time, tentative_track_timeout, Projection_Cam1, Projection_Cam2);
    
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
end
