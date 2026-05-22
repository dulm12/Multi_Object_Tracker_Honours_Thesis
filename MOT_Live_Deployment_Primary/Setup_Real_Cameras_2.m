
% Reads camera intrinsics, extrinsics, and projection matrices.
% Returns a struct containing:
%   - K_cam1, K_cam2
%   - R_GCF_to_Cam1, R_GCF_to_Cam2
%   - t_GCF_to_Cam1, t_GCF_to_Cam2
%   - Projection_cam1, Projection_cam2
%   - P_birth_default
%   - image width and image height 

function cameras = Setup_Real_Cameras_2(params)
    cameras.K_cam1 = params.cam1.K; 
    cameras.K_cam2 = params.cam2.K; 

    cameras.R_GCF_to_Cam1 = params.cam1.R; 
    cameras.t_GCF_to_Cam1 = params.cam1.t; 

    cameras.camera1_position_GCF = params.cam1.pos_GCF; 
    cameras.camera2_position_GCF = params.cam2.pos_GCF; 

    cameras.R_GCF_to_Cam2 = params.cam2.R; 
    cameras.t_GCF_to_Cam2 = params.cam2.t;

    cameras.Projection_Cam1 = cameras.K_cam1 * [cameras.R_GCF_to_Cam1, cameras.t_GCF_to_Cam1]; 
    cameras.Projection_Cam2 = cameras.K_cam2 * [cameras.R_GCF_to_Cam2, cameras.t_GCF_to_Cam2];

    cameras.P_birth_default = diag([25, 25, 25, 100, 100, 100, 500, 500, 500]); 

    cameras.image_width = params.image_width; 
    cameras.image_height = params.image_height; 
    
end
