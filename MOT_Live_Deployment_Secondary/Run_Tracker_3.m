function [tracks, track_histories, track_colours, detection_stats] = Run_Tracker_3(measurement_log, LIDAR_log, ...
                                                 R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, ...
                                                 R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, ... 
                                                 Projection_Cam1, Projection_Cam2, ...
                                                 P_birth_default, tracker_params)

    %% Unpack Tracking parameters: 
    IMM_TPM                     = tracker_params.IMM_TPM;
    tentative_track_timeout     = tracker_params.tentative_track_timeout; % tentative track deleted if another hit not seen within timeout
    association_threshold       = tracker_params.association_threshold; % Pixels
    max_missed_frames           = tracker_params.max_missed_frames;
    max_time_diff               = tracker_params.max_time_diff; % maximum allowed delay between cam1 & cam2 when birthing new tracks
    max_dist_from_origin        = tracker_params.max_dist_from_origin; % metres
    max_possible_velocity       = tracker_params.max_possible_velocity; % m/s
    max_allowed_hits            = tracker_params.maximum_allowed_hits; % hits 
    LIDAR_match_threshold       = tracker_params.LIDAR_match_threshold; % metres 
    chi2_probability            = tracker_params.chi2_probability; 
    prediction_rate             = tracker_params.prediction_rate; 
    measurement_noise_std       = tracker_params.measurement_noise_std;
    measurement_noise_LIDAR_std = tracker_params.measurement_noise_LIDAR_std;
  
    chi2_Camera = 2 * gammaincinv(chi2_probability, 2/2, 'lower'); % Obtain chi2 threshold from probability. 99% confidence gate, 2 because 2 DOF for (u,v)
    chi2_LIDAR  = 3 * gammaincinv(chi2_probability, 3/2, 'lower');   % 3 DOF (x, y, z)

    %% Timestamp conversion to 0 from UNIX
    if isempty(measurement_log) || isempty(LIDAR_log)
        error('Empty measurement or LiDAR log. Check CSV files.');
    end

    % Normalise the timestamps in measurement/LIDAR log (hardware have them in UNIX)
    t0 = min(measurement_log(1, 1), LIDAR_log(1,1)); 
    measurement_log(:, 1) = measurement_log(:, 1) - t0; 
    LIDAR_log(:, 1) = LIDAR_log(:, 1) - t0; 

    %% Track Management Setup 
    
    % IMM track structure -> Array that will hold each bird struct (state X, covariance P, etc.)
    tracks = struct('id', {}, 'X', {}, 'P', {}, ...
                    'X_cv', {}, 'P_cv', {}, ...
                    'X_ca', {}, 'P_ca', {}, ...
                    'mu', {}, 'c_bar', {}, 'age', {}, 'missed_frames', {});
    
    % Tentative holds first detection of new object
    % hits -> how many times tentative object was re-observed
    % Pos_latest -> most recent 3D position observed. 
    % Timestamp_latest -> timestamp for Pos_latest 
    tentative_tracks = struct('Pos_first', {}, 'Timestamp_first', {}, 'hits', {}, 'Pos_latest', {}, 'Timestamp_latest', {}, 'has_camera_hit', {});
    next_track_id = 1;
    
    R_ekf = diag([measurement_noise_std^2, measurement_noise_std^2]);
    R_LIDAR = diag([measurement_noise_LIDAR_std^2, measurement_noise_LIDAR_std^2, measurement_noise_LIDAR_std^2]);
    
    % Continuous tracker 
    dt_pred = 1 / prediction_rate;
    
    total_session_time = max(measurement_log(end, 1), LIDAR_log(end, 1));
    num_predicted_steps = floor(total_session_time / dt_pred);
    
    % vertical list of timestamps for every iteration in the tracker loop 
    time_history = (0 : num_predicted_steps - 1)' * dt_pred;
    
    % Cell array to store the evolution of a bird track over the whole main
    % loop
    track_histories = {}; 
    
    track_colours = containers.Map('KeyType', 'double', 'ValueType', 'any'); 
    
    base_colours = lines(100); 
    track_colours(1) = base_colours(1, :); 
    track_colours(2) = base_colours(2, :); 

    % How many cam measurements of birds actually flying? 
    total_cam1_valid_meas = 0; 
    total_cam2_valid_meas = 0;

    % Variables for the Temporal Detection Distribution plot 
    detection_stats.time_history = time_history; 
    detection_stats.filtered_t_cam1 = [];
    detection_stats.filtered_t_cam2 = [];
    detection_stats.filtered_t_LIDAR = [];
    detection_stats.cam1_has_detections = [];
    detection_stats.cam2_has_detections = [];
    detection_stats.LIDAR_has_detections = [];
    detection_stats.dual_camera_t = [];

    %% Main Continuous Loop
    for k = 1:num_predicted_steps
        current_time = time_history(k);
    
        % 1. Predict all active tracks from t-1 to t (this exact moment)  
        tracks = Run_Tracker_Predict_3_1(tracks, IMM_TPM, dt_pred);
    
        % Find measurements in this time slice
        current_meas_indices = find(measurement_log(:,1) <= current_time & ...
            measurement_log(:,1) > (current_time - dt_pred));
    
        current_measurements = measurement_log(current_meas_indices, :);
        
        % 1.1. Roofline Filter 
        [current_measurements, num_current_measurements] = Helper_Roofline_Filter(current_measurements); 

        % 1.2. False Positive Exclusion Filter 
        [current_measurements, num_current_measurements] = Helper_False_Positive_Filter(current_measurements); 

        % 1.3. Compute LiDAR slice once per time step (used for stats + 7.5 diagnostic) 
        LIDAR_diagnostic_mask = LIDAR_log(:,1) <= current_time & LIDAR_log(:,1) > (current_time - dt_pred);
        LIDAR_diagnostic_slice = LIDAR_log(LIDAR_diagnostic_mask, :);

        % 2. Sequential Data Association & Camera Update 

        % Split measurements by camera 
        cam1_rows = []; cam2_rows = []; measurements_cam1 = []; measurements_cam2 = [];
        if num_current_measurements > 0
            cam1_rows = current_measurements(:, 2) == 1; 
            cam2_rows = current_measurements(:, 2) == 2; 

            measurements_cam1 = current_measurements(cam1_rows, :); 
            measurements_cam2 = current_measurements(cam2_rows, :);

            total_cam1_valid_meas = total_cam1_valid_meas + size(measurements_cam1, 1);
            total_cam2_valid_meas = total_cam2_valid_meas + size(measurements_cam2, 1);
        end
        
        % Update Detection Stats 
        detection_stats = Helper_Update_Detection_Stats( ...
                             detection_stats, measurements_cam1, measurements_cam2, ...
                             LIDAR_diagnostic_slice, current_time);

        num_measurements_cam1 = size(measurements_cam1, 1); 
        num_measurements_cam2 = size(measurements_cam2, 1); 

        % 2.1. First Pass (Camera 1 Association & Updates)
        associations_cam1 = Run_Tracker_Association_3_2( ...
            tracks, measurements_cam1, num_measurements_cam1, ...
            K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
            K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
            R_ekf, association_threshold, chi2_Camera);

        [tracks, unassigned_cam1_meas_mask, was_associated_cam1] = Run_Tracker_Update_3_3( ...
            tracks, associations_cam1, measurements_cam1, num_measurements_cam1, ...
            K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
            K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
            R_ekf, max_missed_frames, max_dist_from_origin, max_possible_velocity);
        
        % Frames not deleted or incremented yet until both passes are done 

        % Add right after cam1 update pass, before cam2 association
        if ~isempty(tracks)
            for i = 1:length(tracks)
                if was_associated_cam1(i) && tracks(i).id <= 3
                    point_cam2 = K_cam2 * (R_GCF_to_Cam2 * tracks(i).X(1:3) + t_GCF_to_Cam2);
                    uv_cam2_pred = point_cam2(1:2) / point_cam2(3);
                    fprintf('BETWEEN PASSES: Track %d cam1-corrected pos=[%.2f,%.2f,%.2f] -> cam2 predicts pixel [%.0f,%.0f]\n', ...
                    tracks(i).id, tracks(i).X(1), tracks(i).X(2), tracks(i).X(3), uv_cam2_pred(1), uv_cam2_pred(2));
                end
            end
        end

        % 2.2. Second Pass (Camera 2 Association & Updates)
        associations_cam2 = Run_Tracker_Association_3_2( ...
            tracks, measurements_cam2, num_measurements_cam2, ...
            K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
            K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
            R_ekf, association_threshold, chi2_Camera);

        [tracks, unassigned_cam2_meas_mask, was_associated_cam2] = Run_Tracker_Update_3_3( ...
            tracks, associations_cam2, measurements_cam2, num_measurements_cam2, ...
            K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
            K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
            R_ekf, max_missed_frames, max_dist_from_origin, max_possible_velocity);

        % 2.3 LiDAR State & Covariance Update 
        [tracks, LIDAR_associations] = Helper_LiDAR_Update( ...
                                        tracks, LIDAR_diagnostic_slice, R_LIDAR, ...
                                        LIDAR_match_threshold, chi2_LIDAR);
        
        % Now check which tracks associated with the LiDAR 
        was_associated_LIDAR = false(length(tracks),1);
        for i = 1:length(LIDAR_associations)
            was_associated_LIDAR(i) = ~isempty(LIDAR_associations{i});
        end

        % 3. Missed Frames and marking tracks for deletion
        % Combine deletion masks 
        tracks_to_keep = true(length(tracks), 1);

        % After both camera passes, updates and LiDAR update, handle the missed frames and
        % deletion 
        for i = 1:length(tracks) 
            if was_associated_cam1(i) || was_associated_cam2(i) || was_associated_LIDAR(i) 
                % Atleast one camera saw the track, reset missed counter 
                tracks(i).missed_frames = 0; 
            else 
                % Neither cameras saw it
                tracks(i).missed_frames = tracks(i).missed_frames + 1;
            end 

            % Track Deletions 
            if tracks(i).missed_frames >= max_missed_frames 
                tracks_to_keep(i) = false;
            end

            if tracks(i).X(1) < 0
                tracks_to_keep(i) = false; 
            end 

            if tracks(i).missed_frames > 20 
                if norm(tracks(i).X(1:3)) > max_dist_from_origin
                    tracks_to_keep(i) = false; 
                end 
                if norm(tracks(i).X(4:6)) > max_possible_velocity
                    tracks_to_keep(i) = false; 
                end 
            end 
        end 
        
        % Debug 3:
        for i = 1:length(tracks)
            if ~tracks_to_keep(i)
                fprintf('DELETE: Track %d final_pos=[%.2f,%.2f,%.2f] missed=%d age=%d\n', ...
                tracks(i).id, tracks(i).X(1), tracks(i).X(2), tracks(i).X(3), ...
                tracks(i).missed_frames, tracks(i).age);
            end
        end

        % 4. Perform Deletion
        tracks = Run_Tracker_Delete_3_4(tracks, tracks_to_keep);
    
        % 5. Gate sweep (combine unassigned meas. from both cameras) 
        % Rebuild the full unassigned mask for the track birth pipeline 
        unassigned_meas_mask = true(num_current_measurements, 1); 

        % Mark cam1 assignments 
        cam1_indices = find(cam1_rows); 
        for j = 1:num_measurements_cam1
            if ~unassigned_cam1_meas_mask(j)
                unassigned_meas_mask(cam1_indices(j)) = false; 
            end
        end 

        % Mark cam2 assignments 
        cam2_indices = find(cam2_rows); 
        for j = 1:num_measurements_cam2 
            if ~unassigned_cam2_meas_mask(j)
                unassigned_meas_mask(cam2_indices(j)) = false; 
            end
        end 

        unassigned_meas_mask = Run_Tracker_GateSweep_3_5( ...
            tracks, current_measurements, num_current_measurements, ...
            K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
            K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
            R_ekf, chi2_Camera, association_threshold, unassigned_meas_mask);
        
        % 6. Track Birth: 
        % Two-point initialization strategy.
        [tentative_tracks, tracks, next_track_id, track_colours] = ...
            Run_Tracker_Camera_Track_Birth_3_6( ...
                tracks, tentative_tracks, next_track_id, track_colours, max_allowed_hits, ...
                current_measurements, unassigned_meas_mask, ...
                base_colours, P_birth_default, IMM_TPM, ...
                max_time_diff, max_dist_from_origin, max_possible_velocity, ...
                current_time, tentative_track_timeout, Projection_Cam1, Projection_Cam2);

        % 6.1. LiDAR proximity diagnostic check 
        % Debug 5: 
        Helper_LiDAR_Diagnostic(tracks, LIDAR_diagnostic_slice);

        % 7. LiDAR Track Birth
        [tentative_tracks, tracks, next_track_id, track_colours] = ...
            Run_Tracker_LiDAR_Track_Birth_3_7( ...
                tentative_tracks, tracks, next_track_id, track_colours, max_allowed_hits, LIDAR_match_threshold, ...
                LIDAR_log, current_time, dt_pred, ...
                P_birth_default, IMM_TPM, base_colours, ...
                max_possible_velocity, max_dist_from_origin);
    
        % 8. Store all the history of active tracks throughout the run-time
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

    % Session Summary (Debug 4)
    fprintf('\n Session summary:\n');
    fprintf('Total confirmed tracks: %d\n', next_track_id - 1);
    for idx = 1:numel(track_histories)
        if ~isempty(track_histories{idx})
            h = track_histories{idx};
            fprintf('Track %d: duration=%.1fs, start_pos=[%.1f,%.1f,%.1f], end_pos=[%.1f,%.1f,%.1f], max_X=%.1f\n', ...
                idx, h(end,1)-h(1,1), h(1,2), h(1,3), h(1,4), h(end,2), h(end,3), h(end,4), max(h(:,2)));
        end
    end

    % total valid cam measurements 
    fprintf('Post-filter valid cam measurement totals: cam1=%d, cam2=%d\n', total_cam1_valid_meas, total_cam2_valid_meas);
end
    