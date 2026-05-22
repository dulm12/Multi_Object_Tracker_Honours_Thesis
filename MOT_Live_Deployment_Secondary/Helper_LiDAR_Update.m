function [tracks, LIDAR_associations] = Helper_LiDAR_Update(tracks, LIDAR_slice, R_LIDAR, LIDAR_match_threshold, chi2_threshold)
    %  Updates IMM Subfilters and fused state, along with the global state and
    %  covariance. 
    % 
    % Inputs
    %   tracks                 - array of track structs with fields .X (9 State) and .P (9x9)
    %   LIDAR_slice            - N x M matrix where cols 2:4 are [x y z] in same frame as tracks
    %   R_lidar                - 3x3 measurement covariance
    %   LIDAR_match_threshold  - Euclidean distance threshold (meters)
    %   chi2_threshold         - Mahalanobis threshold (chi2inv based for 3 DOF)
    %
    % Outputs
    %   tracks                 - updated tracks (state and covariance)
    %   lidar_associations     - cell array: lidar_associations{i} = indices of LIDAR points associated to track i

    LIDAR_associations = cell(length(tracks),1);
    if isempty(LIDAR_slice) || isempty(tracks)
        return
    end

    H = [eye(3), zeros(3,6)]; % measurement matrix mapping [x y z vx vy vz ax ay az] -> [x y z]
    num_tracks = length(tracks); 
    num_LIDAR = size(LIDAR_slice, 1);

    % Cost matrix: rows = tracks, columns = LiDAR detections 
    cost_matrix = inf(num_tracks, num_LIDAR); 

    for i = 1:length(tracks)

        if numel(tracks(i).X) < 9 || isempty(tracks(i).P)
            continue
        end

        X_pred = tracks(i).X;
        P_pred = tracks(i).P;

        Z_pred = H * X_pred;

        for j = 1:num_LIDAR
            Z_measured = LIDAR_slice(j, 2:4)'; % column vector [x;y;z]
            innovation = Z_measured - Z_pred;
            S = H * P_pred * H' + R_LIDAR;
            
            % Mahalanobis distance (3 DOF)
            mahalanobis_dist = real(innovation' * (S \ innovation));
            euclidean_dist = norm(innovation);
            
            % Accept if within both gates; choose smallest Mahalanobis
            if mahalanobis_dist < chi2_threshold && euclidean_dist < LIDAR_match_threshold
                cost_matrix(i, j) = mahalanobis_dist; 
            end
        end     
    end

    % One-to-one assignment of tracks to detections 
    % assignment looks like: [track_index, lidar_detection_index]
    % which means this track index and this lidar detection point index had the
    % smallest mahalanobis distance 
    assignment = matchpairs(cost_matrix, chi2_threshold); 

    % Go through each matched pairs one row at a time 
    for m = 1:size(assignment , 1)
        i = assignment(m, 1); % Track index 
        j = assignment(m, 2); % LiDAR detection index 

        Z_measured = LIDAR_slice(j, 2:4)'; % column vector [x;y;z]

                    % CV Model Update 
        [X_cv_upd, P_cv_upd] = Helper_LiDAR_Update_One_Model(tracks(i).X_cv, tracks(i).P_cv, Z_measured, H, R_LIDAR);
        
        % CA Model Update 
        [X_ca_upd, P_ca_upd] = Helper_LiDAR_Update_One_Model(tracks(i).X_ca, tracks(i).P_ca, Z_measured, H, R_LIDAR);
        
        mu = tracks(i).mu;

        % Model conditioned states: 
        tracks(i).X_cv = X_cv_upd; 
        tracks(i).P_cv = P_cv_upd; 
        tracks(i).X_ca = X_ca_upd; 
        tracks(i).P_ca = P_ca_upd;

        % Global Fusion 
        tracks(i).X = mu(1) * X_cv_upd + mu(2) * X_ca_upd; 

        difference_CV = X_cv_upd - tracks(i).X; 
        difference_CA = X_ca_upd - tracks(i).X; 

        tracks(i).P = mu(1) * (P_cv_upd + difference_CV * difference_CV') + ...
                      mu(2) * (P_ca_upd + difference_CA * difference_CA');

        tracks(i).missed_frames = 0; % reset missed counter on LiDAR hit

        LIDAR_associations{i} = [LIDAR_associations{i}, j];

    end 
end
 