function params = All_Parameters()
    params.total_time = 80;
    params.dt_sim = 0.01;
    
    % True Object 1
    params.obj1.initial_pos = [-10; 40; 3];
    
    params.obj1.vel_segments = { ...
        [-5; 0; 0.2], ...
        [-5; -3; 4], ...
        [-3; -2; -4], ...
        [5; 5; -0.1], ...
        [2; 2; 4], ...
        [1; 2; -3], ...
        [1; 1; 0] };
    
    params.obj1.switch_times = [3, 10, 16, 28, 48, 60];
    
    % Object 2
    params.obj2.initial_pos = [0; 50; 6];
    
    params.obj2.vel_segments = { ...
        [4; 0; 0.3], ...
        [0; 3; -0.1], ...
        [-4; 0; -0.3] };
    
    params.obj2.switch_times = [35, 45];
    
    % Camera intrinsics
    params.cam1.K = [500,  0 , 960; 
                      0 , 500, 540; 
                      0 ,  0 ,  1 ];
    
    params.cam2.K = [510,  0 , 955; 
                      0 , 510, 545; 
                      0 ,  0 ,  1 ];
    
    % Camera positions
    params.cam1.pos = [20; 0; 1];
    params.cam2.pos = [-20; 0; 1];
    
    params.cam1.up = [0; 0; 1];
    params.cam2.up = [0; 0; 1];
    
    params.look_at_point = [0; 80; 5];
    
    params.image_width = 1920;
    params.image_height = 1080;
    
    % Tracker Parameters 

    % IMM Transition Probability Matrix (2 Models: CV and CA) 
    % Row1: CV -> [Stay CV, Switch to CA] 
    % Row2: CA -> [Switch to CV, Stay CA]
    params.tracker.IMM_TPM = [0.95, 0.05; 
                              0.05, 0.95 ]; 
    params.tracker.tentative_track_timeout = 3;      % seconds
    params.tracker.association_threshold   = 120;    % pixels
    params.tracker.max_missed_frames       = 100;    % frames
    params.tracker.prediction_rate         = 20;     % Hz
    params.tracker.max_time_diff           = 0.2;    % seconds
    params.tracker.max_dist_from_origin    = 70;     % meters
    params.tracker.max_possible_velocity   = 40;     % m/s
    params.tracker.chi2_probability        = 0.99; 

end
