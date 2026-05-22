function [current_measurements, num_current_measurements] = Helper_Roofline_Filter (current_measurements)
    
    if isempty(current_measurements)
        current_measurements = [];
        num_current_measurements = 0; 
        
        return
    end

    num_current_measurements = size(current_measurements,1);

    roofline_v_cam1 = 510;
    roofline_v_cam2 = 510;
    
    valid_mask = true(num_current_measurements,1);
    
    for j = 1:num_current_measurements
        cam_id = current_measurements(j,2);
        v = current_measurements(j,4);
    
        if cam_id == 1 && v > roofline_v_cam1
            valid_mask(j) = false;
        elseif cam_id == 2 && v > roofline_v_cam2
            valid_mask(j) = false;
        end
    end
    
    current_measurements = current_measurements(valid_mask,:);
    num_current_measurements = size(current_measurements, 1); 
end
