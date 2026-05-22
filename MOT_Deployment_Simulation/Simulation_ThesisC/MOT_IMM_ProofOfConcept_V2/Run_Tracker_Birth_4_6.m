function [tentative_tracks, tracks, next_track_ID, track_colours] = ...
    Run_Tracker_Birth_4_6( ...
    tracks, tentative_tracks, next_track_ID, track_colours, ...
    current_measurements, unassigned_meas_mask, ...
    base_colours, P_birth_default, IMM_TPM, ...
    max_time_diff, max_dist_from_origin, max_possible_velocity, ...
    current_time, tentative_track_timeout, Projection_Cam1, Projection_Cam2)

   % Track Birth: 
    % Two-point initialization strategy.

    % gets list of all measurement indexes from curr. time slice not matched to
    % any existing tracks. (Orphans)
    unassigned_meas_indices = find(unassigned_meas_mask);

    % Find all valid triangulated pairs from the unassigned measurements

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
                    meas_cam1 = primary_meas; 
                    meas_cam2 = partner_meas;
                else
                    meas_cam1 = partner_meas; 
                    meas_cam2 = primary_meas;
                end
                
                Point_3d = triangulate(meas_cam1(3:4), meas_cam2(3:4), Projection_Cam1, Projection_Cam2)';

                if norm(Point_3d) > max_dist_from_origin
                    % point too far away, discard. 
                    continue; % Skip to next loop iteration
                end

                % Valid 3D point obtained

                % Cheating for simulation: use true object ID to match tentative points
                object_id = meas_cam1(5);

                % Call helper to update tentative tracks or promote to a full
                % track
                % Helper requires measurement timestamp of either Cam1 or
                % Cam2 of this current measurement. 
                measurement_timestamp = current_measurements(primary_idx, 1); 
                [tentative_tracks, tracks, next_track_ID, track_colours] = ... 
                    Helper_Tentative_Track_Update( ...
                         tentative_tracks, tracks, next_track_ID, track_colours, ...
                         Point_3d, measurement_timestamp, object_id, ...
                         P_birth_default, IMM_TPM, base_colours, ...
                         max_possible_velocity);
                
                % Mark both measurements as processed
                % Successfully used (i & j) pair of measurements to promote a track or 
                % create a new tentative one. 
                % Mark them as "processed" so as not to use them again in this time slice. 
                processed_indices(i) = true;
                processed_indices(j) = true;

                break 
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

end 