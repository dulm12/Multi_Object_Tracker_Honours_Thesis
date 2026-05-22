% Gate Sweep: Prevents left-over tracks from spawning new ghost tracks
function unassigned_meas_mask = Run_Tracker_GateSweep_4_5( ...
    tracks, current_measurements, num_current_measurements, ...
    K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
    K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
    R_EKF, chi2_threshold, association_threshold, unassigned_meas_mask)
    
    % Step 4.5
    for j = 1:num_current_measurements
        if ~unassigned_meas_mask(j)
            continue % Already assigned, skip
        end
    
        cam_id = current_measurements(j, 2);
        if cam_id == 1
            K_cam = K_cam1; 
            R_cam = R_GCF_to_Cam1; 
            t_cam = t_GCF_to_Cam1;
        else
            K_cam = K_cam2; 
            R_cam = R_GCF_to_Cam2; 
            t_cam = t_GCF_to_Cam2;
        end
    
        for i = 1:length(tracks)
    
            % Use helper to compute Mahalanobis + pixel distance
            [mahalanobis_dist, pixel_distance] = Helper_Compute_Measurement_Distance( ...
                tracks(i).X, tracks(i).P, current_measurements(j, 3:4)', ...
                K_cam, R_cam, t_cam, R_EKF);
    
            % If behind camera, mahalanobis_dist = inf, so it will fail the gate automatically.
    
            if mahalanobis_dist < chi2_threshold && pixel_distance < association_threshold
                % This unassigned measurement is statistically consistent
                % with a confirmed track — suppress it from birth
                unassigned_meas_mask(j) = false;
                break
            end
        end
    end

end
