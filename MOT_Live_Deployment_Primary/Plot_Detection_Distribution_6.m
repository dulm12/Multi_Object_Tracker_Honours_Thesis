

function Plot_Detection_Distribution_6(detection_stats)

    % 1. File paths
    cam1_has_detections = detection_stats.cam1_has_detections; 
    cam2_has_detections = detection_stats.cam2_has_detections; 
    LIDAR_has_detections = detection_stats.LIDAR_has_detections; 

    filtered_t_cam1  = unique(detection_stats.filtered_t_cam1); 
    filtered_t_cam2  = unique(detection_stats.filtered_t_cam2); 
    dual_camera_t    = unique(detection_stats.dual_camera_t); 
    filtered_t_LIDAR = unique(detection_stats.filtered_t_LIDAR); 

    % 2. Obtain Percentages 
    % 2.1. Get Session duration and total steps 
    time_history = detection_stats.time_history; 
   
    session_duration = time_history(end) - time_history(1); 
    total_steps = length(time_history); 

    % 2.2. Get either camera detection %, dual-camera detections AND LiDAR detection 
    %  any_camera_pct: fraction of time steps where at least one camera had a detection.
    %  dual_camera_pct: fraction of time steps where BOTH cameras had detections simultaneously.
    any_camera_pct = (sum(cam1_has_detections | cam2_has_detections) / total_steps) * 100; 
    dual_camera_pct = (sum(cam1_has_detections & cam2_has_detections) / total_steps) * 100; 
    LIDAR_pct = (sum(LIDAR_has_detections) / total_steps) * 100; 
    
    % 2.3. Conditional percentages
    %   dual_given_camera_pct: of the time steps when at least one camera was active,
    %                         what percentage had BOTH cameras active.
    %   lidar_given_camera_pct: of the time steps when at least one camera was active,
    %                          what percentage also had LiDAR activity (P(LiDAR | Camera))
    active_camera_mask = cam1_has_detections | cam2_has_detections; 
    active_camera_steps = sum(active_camera_mask); 
    if active_camera_steps == 0
        dual_given_camera_pct = 0; 
        lidar_given_camera_pct = 0;
    else 
        dual_given_camera_pct  = (sum(cam1_has_detections & cam2_has_detections) / active_camera_steps) * 100;
        lidar_given_camera_pct = (sum(LIDAR_has_detections & active_camera_mask) / active_camera_steps) * 100;
    end 

    % 2.4. The other conditional metric
    %   camera_given_lidar_pct: of the time steps when LiDAR was active,
    %                          what percentage also had at least one camera detection (P(Camera | LiDAR)).
    if sum(LIDAR_has_detections) == 0
        camera_given_lidar_pct = 0;
    else
        camera_given_lidar_pct = (sum(LIDAR_has_detections & active_camera_mask) / sum(LIDAR_has_detections)) * 100;
    end
    
    % 3. Normalise to session start
    t0 = min([filtered_t_cam1; filtered_t_cam2; filtered_t_LIDAR]);
    t_cam1 = filtered_t_cam1 - t0;
    t_cam2 = filtered_t_cam2 - t0;
    t_LIDAR = filtered_t_LIDAR - t0;
    t_dual_camera = dual_camera_t - t0; 
    
    % 4. Event Plot  
    figure;
    hold on;
    plot(t_cam1,  ones(size(t_cam1)) *  4, 'b.', 'MarkerSize', 8);
    plot(t_cam2,  ones(size(t_cam2)) *  3, 'r.', 'MarkerSize', 8);
    plot(t_dual_camera, ones(size(t_dual_camera)) * 2, 'm.', 'MarkerSize', 10); 
    plot(t_LIDAR, ones(size(t_LIDAR)) * 1, 'k.', 'MarkerSize', 8);
    
    yticks([1 2 3 4]);
    yticklabels({'LiDAR','Dual-camera Overlap', 'Camera 2','Camera 1'});
    xlabel('Time (s)');
    ylabel('Detection Source');
    title('Detection Distribution of Detection Timestamps');
    grid on;
    ylim([0.5 4.5]);

    fprintf('total_steps = %d, session_duration = %.2f, implied rate = %.1f Hz\n', ...
    total_steps, session_duration, total_steps/session_duration);

    % 5. Print the % values 
    fprintf('\nDetection Distribution Summary\n');

    fprintf('Session duration: %.2f seconds\n', session_duration);
    fprintf('Any camera detections: %.2f%% of time steps\n', any_camera_pct);
    fprintf('Dual-camera detections: %.2f%% of time steps\n', dual_camera_pct);
    fprintf('LiDAR detections: %.2f%% of time steps\n', LIDAR_pct);
    fprintf('Dual-camera given every camera-active step: %.2f%%\n', dual_given_camera_pct);
    fprintf('P(LiDAR | Camera): %.2f%%\n', lidar_given_camera_pct);
    fprintf('P(Camera | LiDAR): %.2f%%\n', camera_given_lidar_pct);
end
