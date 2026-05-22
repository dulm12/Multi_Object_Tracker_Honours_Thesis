function [measurement_log, LIDAR_log] = Load_Hardware_Data_1(params)
    % 1. Load Camera Detections 
    cam1_table = readtable(params.cam1_csv); 
    cam2_table = readtable(params.cam2_csv); 

    % Convert table to matrices for EKF [timestamp, camera_id, u, v] 
    cam1_data =[cam1_table{:,1}, cam1_table{:,2}, cam1_table{:,3}, cam1_table{:,4}];  
    cam2_data =[cam2_table{:,1}, cam2_table{:,2}, cam2_table{:,3}, cam2_table{:,4}]; 

    % Combine all cam measurements into one log 
    measurement_log = [cam1_data; 
                       cam2_data];
    measurement_log = sortrows(measurement_log, 1); % Sort by timestamp (col1)

    % 2. Load the LiDAR Detections 
    %  lidar csv: [Timestamp, X_m, Y_m, Z_m] 
    LIDAR_table = readtable(params.LIDAR_csv); 
    LIDAR_log =[LIDAR_table{:,1}, LIDAR_table{:,2}, LIDAR_table{:,3}, LIDAR_table{:,4}];
    LIDAR_log = sortrows(LIDAR_log, 1); 

    % 3. Transform LiDAR detections into camera CF
    for i = 1:size(LIDAR_log, 1)
        LIDAR_log(i, 2:4) = (params.R_lidar_to_GCF * LIDAR_log(i, 2:4)' + ...
                             params.t_lidar_to_GCF)';
    end

    % 4. ROI filter in GCF after transformation
    ROI_mask = LIDAR_log(:,3) >= -20  & LIDAR_log(:,3) <= 1.70;  
    LIDAR_log = LIDAR_log(ROI_mask, :);

    fprintf('Succesfully loaded:\n') 
    fprintf(' -%d LiDAR clusters\n', size(LIDAR_log, 1)); 

end
