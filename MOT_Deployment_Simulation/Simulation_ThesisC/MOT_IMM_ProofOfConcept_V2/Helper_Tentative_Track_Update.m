%   - Updating existing tentative tracks
%   - Promoting tentative tracks to full tracks
%   - Deleting impossible tentative tracks
%   - Creating new tentative tracks
function [tentative_tracks, tracks, next_track_ID, track_colours] = ...
    Helper_Tentative_Track_Update( ...
                    tentative_tracks, tracks, next_track_ID, track_colours, ...
                    Point_3d, meas_timestamp, object_id, ...
                    P_birth_default, IMM_TPM, base_colours, ...
                    max_possible_velocity)
    
    %% A. Check if this 3D point matches an existing tentative track
    
    for tt = 1:length(tentative_tracks)
    
        if tentative_tracks(tt).object_id == object_id
    
            % Update tentative track
            tentative_tracks(tt).hits = tentative_tracks(tt).hits + 1;
            tentative_tracks(tt).Pos_latest = Point_3d;
            tentative_tracks(tt).Timestamp_latest = meas_timestamp;
    
            % Promote after 4 hits
            if tentative_tracks(tt).hits >= 4
    
                Point1 = tentative_tracks(tt).Pos_first;
                Time1  = tentative_tracks(tt).Timestamp_first;
    
                Point2 = tentative_tracks(tt).Pos_latest;
                Time2  = tentative_tracks(tt).Timestamp_latest;
    
                dt_vel = Time2 - Time1;
    
                if dt_vel > 1e-3
    
                    V0 = (Point2 - Point1) / dt_vel;
    
                    % Impossible velocity so delete tentative track
                    if norm(V0) > max_possible_velocity
                        % 1. Reason for Deletion of TT
                        % track: Impossible Velocity 
                        tentative_tracks(tt) = [];
                        was_point_3d_handled = true;
    
                        return; 
                    end
    
                    % Create a new full track 
                    new_track.id = next_track_ID;
                    
                    % Initialise 9 state vector
                    initial_X = [Point2; V0; 0; 0; 0];
    
                    new_track.X     = initial_X;
                    new_track.P     = P_birth_default;
    
                    % Initialise both internal IMM models with same starting
                    % belief.
                    new_track.X_cv  = initial_X;
                    new_track.P_cv  = P_birth_default;
    
                    new_track.X_ca  = initial_X;
                    new_track.P_ca  = P_birth_default;
    
                    % Initial Probabilities [80% CV, 20% CA]
                    new_track.mu    = [0.8; 0.2];
                    new_track.c_bar = IMM_TPM' * new_track.mu;
    
                    new_track.age = 0;
                    new_track.missed_frames = 0;
    
                    tracks(end + 1) = new_track;
                    next_track_ID = next_track_ID + 1;
    
                    % Assign colour
                    if ~isKey(track_colours, new_track.id)
                        track_colours(new_track.id) = base_colours(mod(new_track.id - 1, 100) + 1, :);
                    end
    
                    % 2. Reason for Deletion of TT track:
                    % Succesful FULL track birth
                    tentative_tracks(tt) = [];
                end
            end
    
            % Match was found and handled (even if not promoted). 
            return; % EXIT FUNCTION
        end
    end
    
    %% B. If no existing tentative track matched:
    % Only way the code reaches this line is if the loop above never hit a 'return'.
    % Therefore, it is a newly detected object. 
    % it is first point for a potentially newly detected object 
    new_tentative.Pos_first        = Point_3d;
    new_tentative.Timestamp_first  = meas_timestamp;
    new_tentative.object_id        = object_id;
    new_tentative.hits             = 1;
    new_tentative.Pos_latest       = Point_3d;
    new_tentative.Timestamp_latest = meas_timestamp;

    tentative_tracks(end + 1) = new_tentative;
    % This track is put in probation. 

end
