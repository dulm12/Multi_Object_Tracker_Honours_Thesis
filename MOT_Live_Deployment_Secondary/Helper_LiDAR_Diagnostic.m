function Helper_LiDAR_Diagnostic(tracks, LIDAR_diagnostic_slice)
    if isempty(LIDAR_diagnostic_slice)
       return
    end
    
    maximum_track_distance_to_LIDAR = 5; 

    for i = 1:length(tracks)
        for j = 1:size(LIDAR_diagnostic_slice, 1)
            dist = norm(tracks(i).X(1:3) - LIDAR_diagnostic_slice(j, 2:4)');
            if dist < maximum_track_distance_to_LIDAR  % Only print if LiDAR detection is within 5m of track

                fprintf('LIDAR CHECK: Track %d pos=[%.2f,%.2f,%.2f] lidar=[%.2f,%.2f,%.2f] dist=%.2f\n', ...
                    tracks(i).id, tracks(i).X(1), tracks(i).X(2), tracks(i).X(3), ...
                    LIDAR_diagnostic_slice(j,2), LIDAR_diagnostic_slice(j,3), LIDAR_diagnostic_slice(j,4), dist);
                
            end
        end
    end

end