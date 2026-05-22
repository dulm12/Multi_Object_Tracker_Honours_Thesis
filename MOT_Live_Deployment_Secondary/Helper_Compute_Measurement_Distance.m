function [mahalanobis_dist, pixel_distance] = Helper_Compute_Measurement_Distance( ...
                                                X_pred, P_pred, Z_measured, ...
                                                K_cam, R_GCF_to_Cam, t_GCF_to_Cam, R_EKF)

    % Project predicted 3D position into pixel space. 
    point_camera = R_GCF_to_Cam * X_pred(1:3) + t_GCF_to_Cam;
    
    % If behind camera, then invalid point. 
    if point_camera(3) <= 0.01
        mahalanobis_dist = inf;
        pixel_distance = inf;
        Z_expected = [inf; inf];
        return
    end
    
    Xc_val_pred = point_camera(1);
    Yc_val_pred = point_camera(2);
    Zc_val_pred = point_camera(3);
    
    Xc_norm_pred = Xc_val_pred / Zc_val_pred; 
    Yc_norm_pred = Yc_val_pred / Zc_val_pred;

    % Predicted pixel location (predicted u,v)
    Z_expected = K_cam * [Xc_norm_pred; Yc_norm_pred; 1];
    Z_expected = Z_expected(1:2);
    
    % Jacobian H
    fx = K_cam(1,1);
    fy = K_cam(2,2);
    
    % Jacobian of pixel projection w.r.t 3D point in camera coords 
    % u = fx * Xc/Zc + cx
    % v = fy * Yc/Zc + cy
    % 2 x 3 matrix 
    dZexpected_dPoint_camera = [(fx/Zc_val_pred),      0          , (-fx * Xc_val_pred)/(Zc_val_pred^2);
                                       0        , (fy/Zc_val_pred), (-fy * Yc_val_pred)/(Zc_val_pred^2)];
    
    H = [dZexpected_dPoint_camera * R_GCF_to_Cam, zeros(2, 6)];
    
    % Innovation covariance
    S = H * P_pred * H' + R_EKF;
    
    % Innovation vector
    Z_innovation = Z_measured - Z_expected;
    
    % Mahalanobis distance
    mahalanobis_dist = Z_innovation' * (S \ Z_innovation);
    
    % Pixel distance (used in Step 4.5)
    pixel_distance = norm(Z_innovation);

end
