function [valid_Track_IDs, track_labels] = Helper_Track_Scoring_System(track_histories)
    % Post-analysis track screening:
    % 1. Reject physically implausible tracks
    % 2. Score remaining tracks
    % 3. Label tracks as high / medium / low confidence
  
    % Input:
    %   track_histories{track_id} = [time, x, y, z, vx, vy, vz, ax, ay, az]
  
    % Outputs:
    %   valid_Track_IDs : tracks kept for plotting / analysis
    %   track_scores    : numeric score for every track
    %   track_labels    : "HIGH", "MEDIUM", "LOW", or "REJECTED"
    %   rejected_Track_IDs    : track IDs rejected on physical grounds
    %   reject_reasons  : cell array of rejection reasons
    
    num_tracks = numel(track_histories);

    track_scores   = nan(num_tracks,1);
    track_labels   = strings(num_tracks,1);
    rejected_reasons = strings(num_tracks,1);

    valid_Track_IDs     = [];
    rejected_Track_IDs  = [];

    % Physical Limits 
    MAX_X = 20;          % m
    MAX_ABS_Y = 20;      % m
    MIN_Z = -1;          % m
    MAX_Z = 20;          % m
    MAX_SPEED = 25;      % m/s

    % Scoring references 
    REF_DURATION = 5.0;      % s
    REF_UPDATES  = 20;       % samples
    REF_MAX_X    = 12;       % m, "good" corridor depth
    REF_MAX_ERRATIC_SPEED = 15; % m/s

    for track_ID = 1:num_tracks

        if isempty(track_histories{track_ID})
            track_labels(track_ID) = "Empty";
            continue;
        end

        track_history = track_histories{track_ID};

        % History columns: [time, x, y, z, vx, vy, vz, ax, ay, az]
        track_time  = track_history(:,1);
        track_x  = track_history(:,2);
        track_y  = track_history(:,3);
        track_z  = track_history(:,4);

        track_duration = track_time(end) - track_time(1);
        num_track_updates = size(track_history,1);

        if size(track_history, 2) >= 7
            vx = track_history(:, 5); 
            vy = track_history(:, 6); 
            vz = track_history(:, 7);

            speed = sqrt(vx.^2 + vy.^2 + vz.^2);
            max_speed = max(speed);
        else
            max_speed = 0;
        end

        max_x = max(track_x);
        max_abs_y = max(abs(track_y));
        min_z = min(track_z);
        max_z = max(track_z);

        % Rejecting tracks 

        % 1. If track is behind camera 
        if any(track_x < -0.0)
            track_labels(track_ID) = "REJECTED";
            rejected_reasons(track_ID) = "Behind camera";
            rejected_Track_IDs(end + 1, 1) = track_ID;
            continue;
        end
        
        % 2. If track has gone too far forward from rig 
        if max_x > MAX_X
            track_labels(track_ID) = "REJECTED";
            rejected_reasons(track_ID) = "Exceeded max X/forward bound";
            rejected_Track_IDs(end + 1, 1) = track_ID;
            continue;
        end
        
        % 3. If track has gone too far laterally
        if max_abs_y > MAX_ABS_Y
            track_labels(track_ID) = "REJECTED";
            rejected_reasons(track_ID) = "Exceeded maximum Y/side bound";
            rejected_Track_IDs(end + 1, 1) = track_ID;
            continue;
        end
        
        % 4. If track is too high or too low from the rig 
        if min_z < MIN_Z || max_z > MAX_Z
            track_labels(track_ID) = "REJECTED";
            rejected_reasons(track_ID) = "Exceeded vertical bound";
            rejected_Track_IDs(end + 1, 1) = track_ID;
            continue;
        end

        % 5. If track's maximum speed is implausible 
        if max_speed > MAX_SPEED
            track_labels(track_ID) = "REJECTED";
            rejected_reasons(track_ID) = "Exceeded speed bound";
            rejected_Track_IDs(end + 1, 1) = track_ID;
            continue;
        end

        % Confidence score 
        score = 0;

        % Duration contribution
        score = score + min(track_duration / REF_DURATION, 1.0);

        % Number of updates contribution
        score = score + min(num_track_updates / REF_UPDATES, 1.0);

        % Prefer tracks that remain in plausible near field
        score = score + max(0, 1 - (max_x / REF_MAX_X));

        % Penalise high-speed erratic motion without full rejecting 
        speed_penalty = min(max_speed / REF_MAX_ERRATIC_SPEED, 1.0);
        score = score + (1 - speed_penalty);

        track_scores(track_ID) = score;

        if score >= 3.0
            track_labels(track_ID) = "High";
        elseif score >= 2.0
            track_labels(track_ID) = "Medium";
        else
            track_labels(track_ID) = "Low";
        end

        valid_Track_IDs(end + 1, 1) = track_ID;
    end

    fprintf('\nFiltered plotting summary:\n');
    fprintf('Valid tracks kept: %d\n', numel(valid_Track_IDs));
    fprintf('Rejected tracks: %d\n', numel(rejected_Track_IDs));
    
    for i = 1:numel(rejected_Track_IDs)
        fprintf('Rejected Track %d: %s\n', rejected_Track_IDs(i), rejected_reasons(rejected_Track_IDs(i)));
    end
end