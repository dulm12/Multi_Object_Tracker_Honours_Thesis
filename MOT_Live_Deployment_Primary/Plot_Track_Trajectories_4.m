function Plot_Track_Trajectories_4( ...
    measurement_log, LIDAR_log, ...
    track_histories, track_colours, ...
    valid_Track_IDs, track_labels, ...
    cam1_pos_GCF, cam2_pos_GCF, ...
    R_GCF_to_Cam1, R_GCF_to_Cam2, ...
    K_cam1, K_cam2, ...
    image_width, image_height)

    figure(1); clf; hold on;
    
    % Get the normalised timestamps for the lidar (not UNIX) for plotting
    t0 = min(measurement_log(1,1), LIDAR_log(1,1));
    LIDAR_log_normalised = LIDAR_log;
    LIDAR_log_normalised(:,1) = LIDAR_log(:,1) - t0;

    % 1. Visualise cameras and field of view 
    
    % Plot Camera Positions
    plot3(cam1_pos_GCF(1), cam1_pos_GCF(2), cam1_pos_GCF(3), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Camera 1');
    plot3(cam2_pos_GCF(1), cam2_pos_GCF(2), cam2_pos_GCF(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Camera 2');
    
    % Function to plot a camera's FOV (add this helper function at the end of your script)
    Helper_Plot_Camera_FOV(cam1_pos_GCF, R_GCF_to_Cam1, K_cam1, image_width, image_height, [0.6,  0 ,  0 ]);
    Helper_Plot_Camera_FOV(cam2_pos_GCF, R_GCF_to_Cam2, K_cam2, image_width, image_height, [0.1, 0.1, 0.1]);
    
    % 2. LiDAR detections as validation references 
    scatter3(LIDAR_log_normalised(:, 2), LIDAR_log_normalised(:, 3), LIDAR_log_normalised(:, 4), ...
        2, [0 0 0], 'filled', 'DisplayName', 'LiDAR Detections')
    
    % 3. Plot the tracks 
    % valid_ids = find(~cellfun(@isempty, track_histories)); 
    % num_valid_Track_IDS = numel(valid_ids);

    num_valid_Track_IDs = numel(valid_Track_IDs);
    
    for k = 1:num_valid_Track_IDs
        % actual track_id (e.g. 1, 3, 8, 12)
        track_ID = valid_Track_IDs(k); 
        track_history = track_histories{track_ID};

        if isempty(track_history)
            continue; 
        end 

        % return true if track_colours contain an entry for current track_id
        if isKey(track_colours, track_ID) 
            % retrieve the value (RGB triplet) stored in the map for this track_id
            colour = track_colours(track_ID); 
        else 
            % if track id wasn't assigned a colour (rare case)
            colour = [0.5 0.5 0.5];
        end

        % LABEL the track 
        track_label = char(track_labels(track_ID)); 
        display_name = sprintf('Track %d (%s)', track_ID, track_label);
        % history columns -> [time, x, y, z, vx, vy, vz] 
        plot3(track_history(:,2), track_history(:,3), track_history(:,4), '.-', ...
            'Color', colour, 'LineWidth', 1.5, 'DisplayName', display_name);
        % Label tracks on the 3D plot 
        text(track_history(end,2), track_history(end,3), track_history(end,4), ...
            sprintf(' T%d', track_ID), 'Color', colour, 'FontSize', 8, 'FontWeight', 'bold');
    end

    legend('show', 'Location', 'bestoutside'); grid on; axis equal; 
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('IMM-EKF MOT w/ LiDAR Validation'); 
    view(30,20);

end
