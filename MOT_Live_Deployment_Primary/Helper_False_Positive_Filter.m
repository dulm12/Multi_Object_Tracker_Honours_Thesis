function [current_measurements, num_current_measurements] = Helper_False_Positive_Filter (current_measurements)

    if isempty(current_measurements)
        current_measurements = []; 
        num_current_measurements = 0; 

        return
    end

    num_current_measurements = size(current_measurements,1);
    
    fp_tolerance = 5; 
    
    % fp hotspots for March 17th
    % fp_hotspots = [
    %     383, 221, 1;
    %     240, 411, 1;
    %     261, 418, 1;
    %     267, 432, 1;
    %     300, 470, 1;
    %     423, 413, 1;
    %     464, 508, 1;
    %     393, 346, 1;
    %     441, 413, 1;
    %     754, 396, 1;
    %     41,  344, 1;
    %     593, 549, 1;
    %     372, 217, 2;
    %     255, 404, 2;
    %     260, 433, 2;
    %     273, 197, 2;
    %     293, 469, 2;
    %     389, 350, 2;
    %     425, 414, 2;
    % ];

    % fp hotspots for April 1st 
    fp_hotspots = [
        277, 257, 1; 
        503, 447, 1; 
        471, 561, 1; 
        355, 324, 1;
        341, 321, 2; 
        273, 259, 2; 
        465, 558, 2; 
        493, 445, 2; 
        452, 285, 2;
    ];

    valid_mask = true(num_current_measurements, 1);
    for j = 1:num_current_measurements
        cam_id = current_measurements(j, 2);
        u = current_measurements(j, 3);
        v = current_measurements(j, 4);
        
        for z = 1:size(fp_hotspots, 1)
            u0 = fp_hotspots(z, 1); 
            v0 = fp_hotspots(z, 2); 
            cam = fp_hotspots(z, 3); 
    
            if cam_id == cam && abs(u-u0) <= fp_tolerance && abs(v-v0) <= fp_tolerance
                valid_mask(j) = false;
                break;
            end
        end
    end

    current_measurements = current_measurements(valid_mask, :);
    num_current_measurements = size(current_measurements, 1); 

end
