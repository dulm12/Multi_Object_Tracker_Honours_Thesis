
% Generates 3D trajectories for Object 1 (manoeuvring) and Object 2 (constant velocity).
% Returns a struct containing:
%   - time_vector
%   - true_trajectory_3D_1
%   - true_trajectory_3D_2
%   - num_steps

function truth = Simulate_Ground_Truth_1(params)

    % Simulation time 
    total_time = params.total_time;
    dt_sim     = params.dt_sim;
    time_vector = 0:dt_sim:total_time;
    num_steps   = length(time_vector);

    % Ground Truth for Object 1 (manouevring)
    true_trajectory_3D_1 = zeros(3, num_steps);

    velocity1 = params.obj1.vel_segments;
    switch_times1 = params.obj1.switch_times;

    current_vel = velocity1{1};
    true_trajectory_3D_1(:,1) = params.obj1.initial_pos;

    for i = 2:num_steps
        current_time = time_vector(i);

        % Switch velocity segments at defined times
        for k = 1:length(switch_times1)
            if current_time >= switch_times1(k)
                current_vel = velocity1{k+1};
            end
        end

        true_trajectory_3D_1(:,i) = true_trajectory_3D_1(:,i-1) + current_vel * dt_sim;
    end

    % Object 2 (constant velocity)
    true_trajectory_3D_2 = zeros(3, num_steps);
    true_trajectory_3D_2(:,1) = params.obj2.initial_pos;

    velocity2 = params.obj2.vel_segments;
    switch_times2 = params.obj2.switch_times;

    for i = 2:num_steps
        current_time = time_vector(i);

        if current_time < switch_times2(1)
            current_vel = velocity2{1};

        elseif current_time < switch_times2(2)
            current_vel = velocity2{2};

        else
            current_vel = velocity2{3};
        end

        true_trajectory_3D_2(:,i) = true_trajectory_3D_2(:, i-1) + current_vel * dt_sim;
    end

    % Pack output
    truth.time_vector          = time_vector;
    truth.num_steps            = num_steps;
    truth.true_trajectory_3D_1 = true_trajectory_3D_1;
    truth.true_trajectory_3D_2 = true_trajectory_3D_2;
end
