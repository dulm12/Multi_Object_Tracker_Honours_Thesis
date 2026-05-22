function stats = Helper_Update_Detection_Stats (stats, ...
                                                measurements_cam1, ...
                                                measurements_cam2, ...
                                                LIDAR_slice, ...
                                                current_time)

    %% Camera 1 stats
    if ~isempty(measurements_cam1)
        stats.filtered_t_cam1 = [stats.filtered_t_cam1; measurements_cam1(:,1)];
        stats.cam1_has_detections = [stats.cam1_has_detections; true];
    else
        stats.cam1_has_detections = [stats.cam1_has_detections; false];
    end

    %% Camera 2 stats
    if ~isempty(measurements_cam2)
        stats.filtered_t_cam2 = [stats.filtered_t_cam2; measurements_cam2(:,1)];
        stats.cam2_has_detections = [stats.cam2_has_detections; true];
    else
        stats.cam2_has_detections = [stats.cam2_has_detections; false];
    end

    %% Dual‑camera overlap
    if ~isempty(measurements_cam1) && ~isempty(measurements_cam2)
        stats.dual_camera_t = [stats.dual_camera_t; current_time];
    end

    %% LiDAR stats
    if ~isempty(LIDAR_slice)
        stats.filtered_t_LIDAR = [stats.filtered_t_LIDAR; current_time];
        stats.LIDAR_has_detections = [stats.LIDAR_has_detections; true];
    else
        stats.LIDAR_has_detections = [stats.LIDAR_has_detections; false];
    end
end
