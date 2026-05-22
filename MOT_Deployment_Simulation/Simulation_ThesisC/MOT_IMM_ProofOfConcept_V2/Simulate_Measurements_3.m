
% SIMULATE_MEASUREMENTS
% This function reproduces Section 2 of your original script exactly.
% It simulates asynchronous multi-object measurements from Camera 1 and Camera 2.
% All variable names and comments are preserved exactly as in your original code.

function [measurement_log, measurement_noise_std] = Simulate_Measurements_3(true_trajectory_3D_1, true_trajectory_3D_2, ...
                                                 time_vector, num_steps, ...
                                                 R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, ...
                                                 R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, ...
                                                 image_width, image_height, ...
                                                 total_time)

    %% Section 2: Simulate Asynchronous Multi-Object Measurements
    
    cam1_fr = 30;
    cam2_fr = 25;
    
    cam1_dt = 1 / cam1_fr;
    cam2_dt = 1 / cam2_fr;
    
    measurement_log = [];
    measurement_noise_std = 1;
    
    %% Camera 1 loop
    for time_cam1 = 0: cam1_dt: total_time
    
        % Aligning two timelines:
        % Physics: Ground truth exists at high frequency (0.01s)
        % Camera: Cam captures frames at lower, specific frequency (~0.033s).
        % At the exact moment the camera opened, which row in 'Simulation time' best represents the world?
        idx = min(max(round(time_cam1 / (time_vector(2)-time_vector(1))) + 1, 1), num_steps);
    
        % true_trajectory_3D -> Each row contains either x, y or z.
        % Column is time index, the index of the no. of steps taken so far.
        [uv1, obj1_in_cam1_fov] = Helper_Project_Point(true_trajectory_3D_1(:, idx), ...
                                                R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, ...
                                                image_width, image_height);
    
        % if object 1 visible for cam1 then add to measurement log
        if obj1_in_cam1_fov
            % add new row:
            % timestamp, camera id, add noise to the u,v coords, object id
            % transpose is to add everything horizontally.
            measurement_log = [measurement_log; time_cam1, 1, (uv1 + randn(2,1) * measurement_noise_std)', 1];
        end
    
        [uv2, obj2_in_cam1_fov] = Helper_Project_Point(true_trajectory_3D_2(:, idx), ...
                                                R_GCF_to_Cam1, t_GCF_to_Cam1, K_cam1, ...
                                                image_width, image_height);
    
        % if object 2 visible for cam1 then add to measurement log
        if obj2_in_cam1_fov
            measurement_log = [measurement_log; time_cam1, 1, (uv2 + randn(2,1) * measurement_noise_std)', 2];
        end
    
    end
    
    %% Camera 2 loop
    for time_cam2 = 0: cam2_dt: total_time
    
        idx = min(max(round(time_cam2 / (time_vector(2)-time_vector(1))) + 1, 1), num_steps);
    
        % if object 1 visible for cam2 then add to measurement log
        [uv1, obj1_in_cam2_fov] = Helper_Project_Point(true_trajectory_3D_1(:, idx), ...
                                                R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, ...
                                                image_width, image_height);
    
        if obj1_in_cam2_fov
            % add new row:
            % timestamp, camera id, add noise to the u,v coords, object id
            measurement_log = [measurement_log; time_cam2, 2, (uv1 + randn(2,1) * measurement_noise_std)', 1];
        end
    
        % if object 2 visible for cam2 then add to measurement log
        [uv2, obj2_in_cam2_fov] = Helper_Project_Point(true_trajectory_3D_2(:, idx), ...
                                                R_GCF_to_Cam2, t_GCF_to_Cam2, K_cam2, ...
                                                image_width, image_height);
    
        if obj2_in_cam2_fov
            % add new row:
            % [timestamp, camera id, add noise to the u coord, add noise to v coord, object id]
            measurement_log = [measurement_log; time_cam2, 2, (uv2 + randn(2,1) * measurement_noise_std)', 2];
        end
    
    end
    
    %% Sort rows according to timestamp in measurement log
    measurement_log = sortrows(measurement_log, 1);
    
    if isempty(measurement_log)
        error('No measurements generated, check camera FOV or projection.');
    end

end
