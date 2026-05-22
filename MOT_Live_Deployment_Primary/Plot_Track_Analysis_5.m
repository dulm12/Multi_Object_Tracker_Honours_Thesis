function Plot_Track_Analysis_5(track_histories, track_colours, measurement_log, LIDAR_log, num_tracks_to_show)
    
    % Get the normalised timestamps for the lidar (not UNIX) for plotting
    t0 = min(measurement_log(1,1), LIDAR_log(1,1));
    LIDAR_log_normalised = LIDAR_log;
    LIDAR_log_normalised(:,1) = LIDAR_log(:,1) - t0;

    % Find the tracks with the most camera measurements 
    scores = zeros(numel(track_histories), 1); 
    for i = 1:numel(track_histories)
        if ~isempty(track_histories{i})
            track_history = track_histories{i}; 
            max_x = max(abs(track_history(:, 2))); 
            num_updates = size(track_history, 1); 
            if max_x < 30 
                scores(i) = num_updates; 
            end 
        end 
    end 

    % durations = zeros(numel(track_histories), 1);
    % for i = 1:numel(track_histories)
    %     if ~isempty(track_histories{i})
    %         track_history = track_histories{i};
    %         durations(i) = track_history(end,1) - track_history(1,1);
    %     end
    % end
    
    % Get the ids of the most observed tracks  
    [~, sorted_ids] = sort(scores, 'descend');
    top_ids = sorted_ids(1:min(num_tracks_to_show, numel(sorted_ids)));
    
    for idx = 1:length(top_ids)
        top_id = top_ids(idx);
        track_history = track_histories{top_id};
        if isempty(track_history)
            continue; 
        end
        
        % get all details of track 
        time = track_history(:,1);     
        x = track_history(:,2);        
        y = track_history(:,3);         
        z = track_history(:,4);        
        
        % % Find nearest LiDAR detection at each track time step
        % LIDAR_x = nan(size(time));
        % LIDAR_y = nan(size(time));
        % LIDAR_z = nan(size(time));
        % LIDAR_distance = nan(size(time));
        % 
        % for k = 1:length(time)
        %     % Find LiDAR detections within +/- 0.5s of this time step
        %     time_mask = abs(LIDAR_log(:,1) - time(k)) < 0.5;
        %     nearby_lidar = LIDAR_log(time_mask, :);
        % 
        %     if ~isempty(nearby_lidar)
        %         % Find  closest LiDAR point
        %         cam_pt_lidar_distance = sqrt((nearby_lidar(:,2) - x(k)).^2 + ...
        %                      (nearby_lidar(:,3) - y(k)).^2 + ...
        %                      (nearby_lidar(:,4) - z(k)).^2);
        %         [min_dist, min_idx] = min(cam_pt_lidar_distance);
        %         LIDAR_x(k) = nearby_lidar(min_idx, 2);
        %         LIDAR_y(k) = nearby_lidar(min_idx, 3);
        %         LIDAR_z(k) = nearby_lidar(min_idx, 4);
        %         LIDAR_distance(k) = min_dist;
        %     end
        % end
        % 

        % Get ALL LiDAR detections during this specific track's lifetime
        lidar_mask = LIDAR_log_normalised(:,1) >= time(1) & LIDAR_log_normalised(:,1) <= time(end);
        lidar_during_track = LIDAR_log_normalised(lidar_mask, :);

        % Get track colour
        if isKey(track_colours, top_id)
            colour = track_colours(top_id);
        else
            colour = [0 0 1];
        end
        
        figure('Name', sprintf('Track %d Analysis', top_id), 'Position', [100 100 900 800]);
        
        % X position 
        subplot(4,1,1);
        plot(time, x, '-', 'Color', colour, 'LineWidth', 1.5); hold on;
        if ~isempty(lidar_during_track)
            plot(lidar_during_track(:, 1), lidar_during_track(:, 2), 'k.', 'MarkerSize', 10);
        end
        ylabel('X (m)');
        title(sprintf('Track %d  |  Duration: %.1fs  |  Updates: %d', top_id, time(end) - time(1), length(time)));
        legend('EKF Estimate', 'Nearest LiDAR', 'Location', 'best');
        grid on;
        
        % Y position 
        subplot(4,1,2);
        plot(time, y, '-', 'Color', colour, 'LineWidth', 1.5); hold on;
        if ~isempty(lidar_during_track)
            plot(lidar_during_track(:, 1), lidar_during_track(:, 3), 'k.', 'MarkerSize', 10);
        end
        ylabel('Y (m)');
        grid on;
        
        % Z position 
        subplot(4,1,3);
        plot(time, z, '-', 'Color', colour, 'LineWidth', 1.5); hold on;
        if ~isempty(lidar_during_track)
            plot(lidar_during_track(:, 1), lidar_during_track(:, 4), 'k.', 'MarkerSize', 10);
        end
        ylabel('Z (m)');
        grid on;
        
        % Distance to nearest LiDAR
        subplot(4,1,4);
        if ~isempty(lidar_during_track) 
            nearest_distance = nan(size(time)); 
            for k = 1:length(time)
                distances = sqrt((lidar_during_track(:, 2) - x(k)).^2 + ...
                                 (lidar_during_track(:, 3) - y(k)).^2 + ... 
                                 (lidar_during_track(:, 4) - z(k)).^2);
                nearest_distance(k) = min(distances); 
            end 
            plot(time, nearest_distance, 'r-', 'LineWidth', 1); 
            ylabel('Nearest LiDAR(m)'); 
        else 
            text(0.5, 0.5, 'No LiDAR detections during this tracks lifetime');
        end 
        % plot(time, LIDAR_distance, 'r.-', 'LineWidth', 1);
        % ylabel('LiDAR Distance (m)');
        % xlabel('Time (s)');
        % grid on;
        
        % Tighten axes
        for plot_index = 1:4
            subplot(4,1,plot_index);
            xlim([time(1) time(end)]);
        end
    end
end