function Plot_All_Trajectories_5( ...
    true_trajectory_3D_1, true_trajectory_3D_2, ...
    track_histories, track_colours, ...
    cam1_pos_GCF, cam2_pos_GCF, ...
    R_GCF_to_Cam1, R_GCF_to_Cam2, ...
    K_cam1, K_cam2, ...
    image_width, image_height)

    figure(1); clf; hold on;
    
    % A) Visualise cameras and field of view 
    
    % Plot Camera Positions
    plot3(cam1_pos_GCF(1), cam1_pos_GCF(2), cam1_pos_GCF(3), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Camera 1');
    plot3(cam2_pos_GCF(1), cam2_pos_GCF(2), cam2_pos_GCF(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Camera 2');
    
    % Function to plot a camera's FOV (add this helper function at the end of your script)
    Helper_Plot_Camera_FOV(cam1_pos_GCF, R_GCF_to_Cam1, K_cam1, image_width, image_height, [0.6,  0 ,  0 ]);
    Helper_Plot_Camera_FOV(cam2_pos_GCF, R_GCF_to_Cam2, K_cam2, image_width, image_height, [0.1, 0.1, 0.1]);
    
    % B) Plot Ground Truth Trajectories 
    plot3(true_trajectory_3D_1(1,:), true_trajectory_3D_1(2,:), true_trajectory_3D_1(3,:), 'b-', 'LineWidth', 2, ...
        'DisplayName', 'True Path 1');
    plot3(true_trajectory_3D_2(1,:), true_trajectory_3D_2(2,:), true_trajectory_3D_2(3,:), 'g-', 'LineWidth', 2, ...
        'DisplayName', 'True Path 2');
    
    % C) Plot the tracks 
    valid_ids = find(~cellfun(@isempty, track_histories)); 
    num_valid = numel(valid_ids); 
    
    for idx = 1:num_valid
        % actual track_id (e.g. 1, 3, 8, 12)
        track_id = valid_ids(idx); 
        history = track_histories{track_id};
    
        % return true if track_colours contain an entry for current track_id
        if isKey(track_colours, track_id) 
            % retrieve the value (RGB triplet) stored in the map for this track_id
            colour = track_colours(track_id); 
        else 
            % if track id wasn't assigned a colour (rare case)
            colour = [0.5 0.5 0.5];
        end
        % history columns -> [time, x, y, z, vx, vy, vz] 
        plot3(history(:,2), history(:,3), history(:,4), '.-', 'Color', colour, ...
            'DisplayName', ['Track ' num2str(track_id)]);
    end
    
    legend('show', 'Location', 'best'); grid on; axis equal; 
    title('Multi-Object EKF Tracking'); 
    view(30,20);

end
