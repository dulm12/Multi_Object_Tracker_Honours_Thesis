function [tentative_tracks, tracks, next_track_ID, track_colours] = ...
    Run_Tracker_LiDAR_Track_Birth_3_7(...
    tentative_tracks, tracks, next_track_ID, track_colours, max_allowed_hits, LIDAR_match_threshold,...
    LIDAR_log, current_time, dt_pred, ... 
    P_birth_default, IMM_TPM, base_colours, ... 
    max_possible_velocity, max_dist_from_origin)

    % Find LiDAR detections in this time slice 
    LIDAR_mask = LIDAR_log(:, 1) <= current_time & ...
                 LIDAR_log(:, 1) > (current_time - dt_pred); 
    LIDAR_slice = LIDAR_log(LIDAR_mask, :); 

    if isempty(LIDAR_slice)
        return 
    end

    for i = 1:size(LIDAR_slice, 1)

        LIDAR_point = LIDAR_slice(i, 2:4)'; 
        LIDAR_time  = LIDAR_slice(i, 1); 

        % Discard physically implausible points 
        if norm(LIDAR_point) > max_dist_from_origin
            continue 
        end

        % 1. Check against existing confirmed tracks 
        % If LiDAR point already near a confirmed track, it is already
        % being tracked, dont do anything. 
        match_confirmed = false; 
        for t = 1:length(tracks)
            if norm(tracks(t).X(1:3) - LIDAR_point) < LIDAR_match_threshold
                match_confirmed = true; 
                break 
            end 
        end

        if match_confirmed 
            continue 
        end 

        % 2. Check against tentative tracks 
        % Either match this point to an existing tentative track and
        % promote if hits >= 4 or create a new tentative track
        [tentative_tracks, tracks, next_track_ID, track_colours] = ...
            Helper_Tentative_Track_Update( ...
                tentative_tracks, tracks, next_track_ID, track_colours, max_allowed_hits, ...
                LIDAR_point, LIDAR_time, false, P_birth_default, IMM_TPM, ...
                base_colours, max_possible_velocity);

    end

end 