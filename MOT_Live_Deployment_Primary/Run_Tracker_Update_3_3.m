function [tracks, unassigned_meas_mask, was_associated] = Run_Tracker_Update_3_3( ...
                                                            tracks, associations, current_measurements, num_current_measurements, ...
                                                            K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
                                                            K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
                                                            R_ekf, max_missed_frames, max_dist_from_origin, max_possible_velocity)

    % Update Associated Tracks
    % for each track, this mask will track whether each measurement was
    % assigned a track in the below for loop 
    unassigned_meas_mask = true(num_current_measurements, 1);
    was_associated = false(length(tracks), 1);

    decay_factor = 0.95; 
    
    % Update associated tracks 
    for i = 1:length(tracks)
        measurement_idx = associations(i);

        if measurement_idx > 0 % Track was associated
            cam_id = current_measurements(measurement_idx, 2);
    
            % Measured u, v
            Z_m = current_measurements(measurement_idx, 3:4)';
    
            if cam_id == 1 
                K_cam = K_cam1; R_cam = R_GCF_to_Cam1; t_cam = t_GCF_to_Cam1;
            else 
                K_cam = K_cam2; R_cam = R_GCF_to_Cam2; t_cam = t_GCF_to_Cam2;
            end
    
            % A. Update Both Models Individually 
            [X_cv_update, P_cv_update, L_cv] = Helper_EKF_Update_Step( ...
                tracks(i).X_cv, tracks(i).P_cv, Z_m, R_ekf, K_cam, R_cam, t_cam);
    
            [X_ca_update, P_ca_update, L_ca] = Helper_EKF_Update_Step( ...
                tracks(i).X_ca, tracks(i).P_ca, Z_m, R_ekf, K_cam, R_cam, t_cam);
    
            % B. Update Mode Probabilities (mu)
    
            c_bar = tracks(i).c_bar; % Retrieved from prediction step 
    
            % Multiply likelihood by normalisation constant 
            mu_cv_unscaled = L_cv * c_bar(1); 
            mu_ca_unscaled = L_ca * c_bar(2); 
    
            % Normalise so addition up to 1.0 (e.g. 0.85 and 0.15)
            mu_sum = mu_cv_unscaled + mu_ca_unscaled; 
            % Guard against numerical underflow
            if mu_sum < 1e-30
                % Both models found measurement equally implausible
                % Fall back to predicted probabilities rather than corrupting mu
                tracks(i).mu = tracks(i).c_bar;
            else
                tracks(i).mu = [mu_cv_unscaled; mu_ca_unscaled] / mu_sum;
            end 
    
            % C. Combine for Global Updated State 
    
            tracks(i).X_cv = X_cv_update; tracks(i).P_cv = P_cv_update; 
            tracks(i).X_ca = X_ca_update; tracks(i).P_ca = P_ca_update; 
    
            % D. Global Fusion 
            tracks(i).X = tracks(i).mu(1) * X_cv_update + tracks(i).mu(2) * X_ca_update; 
    
            diff_cv = X_cv_update - tracks(i).X; 
            diff_ca = X_ca_update - tracks(i).X; 
    
            tracks(i).P = tracks(i).mu(1) * (P_cv_update + diff_cv * diff_cv') + ...
                          tracks(i).mu(2) * (P_ca_update + diff_ca * diff_ca'); 
            
            % Debug 2 
            point_cam_debug = K_cam * (R_cam * tracks(i).X(1:3) + t_cam);
            uv_debug = point_cam_debug(1:2) / point_cam_debug(3);
            pxdist_debug = norm(current_measurements(measurement_idx,3:4)' - uv_debug);
            fprintf('UPDATE: Track %d cam%d pos=[%.2f,%.2f,%.2f] pxdist=%.1f\n', ...
                        tracks(i).id, current_measurements(measurement_idx,2), ...
                        tracks(i).X(1), tracks(i).X(2), tracks(i).X(3), pxdist_debug);
            
            was_associated(i) = true; 
            unassigned_meas_mask(measurement_idx) = false;
    
        else % Track was not associated

            % If track not seen, assume it stoped accelerating and returned
            % to smooth glide
            tracks(i).X_ca(7:9) = tracks(i).X_ca(7:9) * decay_factor;
            tracks(i).X(7:9)    = tracks(i).X(7:9) * decay_factor;

            % If no measurement, probabilities don't change 
            % Keep predicted states as the updated states 
            tracks(i).mu = tracks(i).c_bar; 
        end
    end
end
