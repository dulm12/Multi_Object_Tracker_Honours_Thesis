function associations = Run_Tracker_Association_4_2( ...
                                                        tracks, current_measurements, num_current_measurements, ...
                                                        K_cam1, R_GCF_to_Cam1, t_GCF_to_Cam1, ...
                                                        K_cam2, R_GCF_to_Cam2, t_GCF_to_Cam2, ...
                                                        R_ekf, association_threshold, chi2_threshold)

    % Data Association
   
    % associations -> will indicate which measurement will go on with which
    % track. for each track with a valid measurement, do the EKF update
    % with that measurement 
    associations = zeros(length(tracks), 1); 
    
    if ~isempty(tracks) && num_current_measurements > 0
    
        % cost matrix shows how well each measurement matches each track 
        % row -> each track 
        % column -> each measurement 
        cost_matrix = inf(length(tracks), num_current_measurements); 
    
        % for loops -> for every track-measurement pair, project the tracks
        % 3D predicted position into the u,v image plane. 
        % find the distance between the predicted pixel and the measured pixel  
        for i = 1:length(tracks)
            X_pred = tracks(i).X; 
            P_pred = tracks(i).P; 
    
            for j = 1:num_current_measurements
                cam_id = current_measurements(j, 2); % (2 is the camera_id column)
    
                % Select correct camera matrices
                if cam_id == 1
                    K_cam = K_cam1;
                    R_cam = R_GCF_to_Cam1;
                    t_cam = t_GCF_to_Cam1;
                else
                    K_cam = K_cam2;
                    R_cam = R_GCF_to_Cam2;
                    t_cam = t_GCF_to_Cam2;
                end
    
                % Find Mahalanobis distance: 
                % How SURPRISING is this measurement, given what the filter knows about it's own uncertainty?
                % d_mahal = Z' * inv(S) * Z
                % inv(s) is stretches and squishes the ellipse. 
                % A measurement landing inside the ellipse → small Mahalanobis distance → yes,within my expected uncertainty
                % A measurement landing outside the ellipse → large Mahalanobis distance → too surprising, I wasn't expecting that
                % The size and shape of the ellipse changes every single timestep as P and S evolve.

                [mahalanobis_dist, pixel_distance] = Helper_Compute_Measurement_Distance( ...
                    X_pred, P_pred, current_measurements(j,3:4)', ...
                    K_cam, R_cam, t_cam, R_ekf);
    
                % Chi-squared gate, only allowing statistically plausible pairs 
                % Check pixel distance too.Prevent track from being
                % associated with highly uncertain measurements when
                % accelerating (when accelerating P is large so highly
                % uncertain measurements can still be accepted)
                if mahalanobis_dist < chi2_threshold && pixel_distance < association_threshold
                    cost_matrix(i, j) = mahalanobis_dist; % Mahalanobis is the cost 
                end
    
                % Pairs outside the gate remain inf, so excluded. 
            end
        end
    
        % Hungarian Algorithm -> Globally Optimal Assignment 
        assignment = matchpairs(cost_matrix, chi2_threshold); 
    
        for m = 1:size(assignment, 1)
            associations(assignment(m, 1)) = assignment(m, 2); 
        end
    end 

end
