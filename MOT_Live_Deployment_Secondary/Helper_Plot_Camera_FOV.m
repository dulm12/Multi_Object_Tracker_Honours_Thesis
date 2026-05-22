% Draws the field of view of each camera on the plot. 
function Helper_Plot_Camera_FOV(cam_pos_GCF, R_GCF_to_Cam, K_cam, width, height, line_colour)

    line_style = ':';             % Dotted line

    % Define 4 corners of the image plane (pixels)
    corners_pixels = [0, width, width ,   0;
                      0,   0  , height, height;
                      1,   1  ,   1   ,   1];
    
    % Convert pixel corners to normalised image coordinates in camera frame
    corners_cam_norm = (K_cam \ corners_pixels);
    
    % Draw the field of view length up to 30 meters out
    fov_length = 30; % 
    
    % Scale the corners to create 3D points in the camera frame
    corners_cam_3d = corners_cam_norm * fov_length;
    
    % Transform these 3D points from the Camera Frame back to the Global Frame
    R_Cam_to_GCF = R_GCF_to_Cam'; 
    t_Cam_to_GCF = -R_Cam_to_GCF * (-R_GCF_to_Cam * cam_pos_GCF);
    
    % Camera corners in GCF coords
    corners_GCF = R_Cam_to_GCF * corners_cam_3d + t_Cam_to_GCF;
    
    % Draw the lines from the camera position to the corners of the frustum
    for i = 1:4
        plot3([cam_pos_GCF(1), corners_GCF(1, i)], ...
              [cam_pos_GCF(2), corners_GCF(2, i)], ...
              [cam_pos_GCF(3), corners_GCF(3, i)], 'Color', line_colour, ...
              'LineStyle', line_style, 'LineWidth', 1, 'HandleVisibility', 'off');
    end
    
    % Draw the rectangle at the end of the frustum
    plot3([corners_GCF(1,:), corners_GCF(1,1)], ...
          [corners_GCF(2,:), corners_GCF(2,1)], ...
          [corners_GCF(3,:), corners_GCF(3,1)], 'Color', line_colour, ...
          'LineStyle', line_style, 'LineWidth', 1, 'HandleVisibility', 'off');
end
