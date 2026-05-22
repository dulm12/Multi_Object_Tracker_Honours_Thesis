
% Builds camera intrinsics, extrinsics, and projection matrices.
% Returns a struct containing:
%   - K_cam1, K_cam2
%   - camera1_position_GCF, camera2_position_GCF
%   - R_GCF_to_Cam1, R_GCF_to_Cam2
%   - t_GCF_to_Cam1, t_GCF_to_Cam2
%   - Projection_cam1, Projection_cam2
%   - P_birth_default

function cameras = Setup_Cameras_2(params)
    % Intrinsic Parameters
    K_cam1 = params.cam1.K;
    K_cam2 = params.cam2.K;

    image_width = params.image_width;
    image_height = params.image_height;

    % Camera positions & up vectors
    camera1_position_GCF = params.cam1.pos;
    camera2_position_GCF = params.cam2.pos;

    camera1_up_GCF  = params.cam1.up;
    camera2_up_GCF  = params.cam2.up;

    look_at_point  = params.look_at_point;

    %% Camera 1 Orientation 

    % Z -> vector from camera to target. (distance)
    % X -> cross product of 2 vecs produces new vector perpendicular to 2 vecs.
    % Hence X is horizontal direction to camera. 
    % Y -> Z is forward. X is right/left. So cross product of X and Z is
    % down. (Y)
    Zc1 = (look_at_point - camera1_position_GCF) / norm(look_at_point - camera1_position_GCF);
    Xc1 = cross(camera1_up_GCF, Zc1); Xc1 = Xc1 / norm(Xc1);
    Yc1 = cross(Zc1, Xc1);

    % Camera Position (R & T)
    % R -> Rotates world so that global axes align with camera axes. 
    % T -> How far world origin has to move to align with camera's origin AFTER
    % rotation applied. 
    R_GCF_to_Cam1 = [Xc1'; Yc1'; Zc1'];
    t_GCF_to_Cam1 = -R_GCF_to_Cam1 * camera1_position_GCF;

    %% Camera 2 Orientation
    Zc2 = (look_at_point - camera2_position_GCF) / norm(look_at_point - camera2_position_GCF);
    Xc2 = cross(camera2_up_GCF, Zc2); Xc2 = Xc2 / norm(Xc2);
    Yc2 = cross(Zc2, Xc2);

    R_GCF_to_Cam2 = [Xc2'; Yc2'; Zc2'];
    t_GCF_to_Cam2 = -R_GCF_to_Cam2 * camera2_position_GCF;

    %% Projection Matrices 
    % Feed 3D Point and spits out a 2D Pixel Coordinate 
    Projection_Cam1 = K_cam1 * [R_GCF_to_Cam1, t_GCF_to_Cam1];
    Projection_Cam2 = K_cam2 * [R_GCF_to_Cam2, t_GCF_to_Cam2];

    %% State Covariance Matrix for newly birthed Tracks 
    % This P is Covariance/Uncertainty matrix. 
    % High uncertainty in Vel and Acc for tracks being birthed new 
    P_birth_default = diag([25, 25, 25, 100, 100, 100, 500, 500, 500]); 

    %% Pack output
    cameras.K_cam1 = K_cam1;
    cameras.K_cam2 = K_cam2;

    cameras.camera1_position_GCF = camera1_position_GCF;
    cameras.camera2_position_GCF = camera2_position_GCF;

    cameras.R_GCF_to_Cam1 = R_GCF_to_Cam1; cameras.t_GCF_to_Cam1 = t_GCF_to_Cam1;
    cameras.R_GCF_to_Cam2 = R_GCF_to_Cam2; cameras.t_GCF_to_Cam2 = t_GCF_to_Cam2;

    cameras.Projection_Cam1 = Projection_Cam1; cameras.Projection_Cam2 = Projection_Cam2;

    cameras.P_birth_default = P_birth_default; 

    cameras.image_width  = image_width;
    cameras.image_height = image_height;
end
