% Simulation 

% Simualtes the 3D trajectory of a single object and then projects this trajectory 
% onto the 2D image plane of a virtual pinhole camera.

% The primary output is the sequence of 2D pixel coordinates representing 
% where the object would appear in the camera's view over time

%% Section 01: Time Vector 

total_time = 10; 
dt = 0.1; 
time_vector = 0:dt:total_time; 

%% Section 02: Object Initial State and Velocity Vector 

% Initial 3D Position: [x; y; z] (metres) 
initial_position = [0; 0; 5];

% Constant 3D Velocity [vx; vy; vz] (metres/second) 
velocity = [1; 0.5; -0.2]; 

%% Section 03: Calculating 3D Trajectory 

num_steps = length(time_vector); 
trajectory_3D = zeros(3, num_steps); 

trajectory_3D(:, 1) = initial_position; 

for i = 2:num_steps 
    % new position = old position + change in position 
    % (Constant Velocity model) 
    trajectory_3D(:, i) = trajectory_3D(:, i-1) + velocity * dt; 
end

%% Section 04: Virtual Camera, K Matrix

focal_length = 500; 
principal_point = [320; 240]; 

% Transform 3D points from the camera's own coordinate system 
% into 2D pixel coordinates on the image plane.
K = [focal_length,      0      , principal_point(1); 
           0     , focal_length, principal_point(2); 
           0     ,      0      ,          1         ]; 

%% Section 05: Virtual Camera Position Definition 

camera_position_world = [-5; -5; 1]; % Camera position in GCF coordinates
look_at_point_world = [0; 0; 0]; % Point the camera is looking at
up_vector_world = [0; 0; 1]; % Defines the 'up' direction for the camera (usually GCF Z-up)

% Rotation (R) and Translation (t) to transform GCF to camera coordinates

% Zc -> Vector pointing from camera to the look at point. 
Zc = (look_at_point_world - camera_position_world) / norm(look_at_point_world - camera_position_world);

% Xc -> Camera's local x-axis. 
Xc = cross(up_vector_world, Zc) / norm(cross(up_vector_world, Zc));

% Yc -> Camera's local y-axis. 
Yc = cross(Zc, Xc);

R_world_to_cam = [Xc' ; Yc'; Zc']; % Rotation matrix used to transform GCF to LCF
t_world_to_cam = -R_world_to_cam * camera_position_world; % Position of GCF origin as seen from camera's coordinate system. 

%% Section 06: Project 3D Trajectory to 2D Image Plane

trajectory_2D_pixels = zeros(2, num_steps); % Store [u; v] pixel coordinates
is_in_FoV = false(1, num_steps);            % row vector to keep track of points in the camera's FoV

% Define image plane boundaries (for FoV check)
image_width = 640;
image_height = 480;

% Iterate through every 3D point in trajectory_3D
for i = 1:num_steps 
    % Get current 3D Point in GCF 
    P_world = trajectory_3D(:, i); 

    % 1. Transform point from GCF to Camera coordinates
    P_camera = R_world_to_cam * P_world + t_world_to_cam;

    % 2. Project to Normalized Image Plane 
    % Only project if the point is in front of the camera (Z_camera > 0)
    if P_camera(3) > 0.01 % Avoiding division by zero

        % Perspective division, coordinates on a normalised image plane
        x_normalized = P_camera(1) / P_camera(3);
        y_normalized = P_camera(2) / P_camera(3);

        % 3. Apply K to get Pixel Coordinates
        % The "homogeneous" part signifies that an extra dimension (the w component, here set to 1) 
        % has been added to [x_normalized; y_normalized] so that K can be
        % applied. 
        p_homogeneous_normalized = [x_normalized; y_normalized; 1];
        p_pixels_homogeneous = K * p_homogeneous_normalized;

        u = p_pixels_homogeneous(1);
        v = p_pixels_homogeneous(2);

        % Store the 2D pixel coordinates
        trajectory_2D_pixels(:, i) = [u; v];

        % Field of View (FoV) Check
        if u >= 0 && u < image_width && v >= 0 && v < image_height
            is_in_FoV(i) = true;
        else
            trajectory_2D_pixels(:, i) = [NaN; NaN]; % Mark as out of FoV
        end

    end 

end

%% Section 07: Visualization 

figure (1);

% Subplot 1: Entire 3D Simulation Plot
subplot(1,2,1); % 1 Row 2 Column Grid
% Plot entire 3D object trajectory as a blue line. 
plot3(trajectory_3D(1,:), trajectory_3D(2,:), trajectory_3D(3,:), 'b-');
hold on;

% Plot points that were in FoV in a different colour
plot3(trajectory_3D(1,is_in_FoV), trajectory_3D(2,is_in_FoV), trajectory_3D(3,is_in_FoV), 'ro');

% Plot camera position
plot3(camera_position_world(1), camera_position_world(2), camera_position_world(3), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
text(camera_position_world(1), camera_position_world(2), camera_position_world(3), ' Camera');

xlabel('X_w (m)'); ylabel('Y_w (m)'); zlabel('Z_w (m)');
title('3D Trajectory & Camera');
grid on; axis equal; view(30,30);
hold off;

% Subplot 2: 2D Image Plane Projection (What camera actually sees on the 2D image
% sensor) 

subplot(1,2,2); % Create a subplot for 2D
% Plot 2D pixel coordinates of the visible points as a red line. 
plot(trajectory_2D_pixels(1,is_in_FoV), trajectory_2D_pixels(2,is_in_FoV), 'r-o');
xlabel('u (pixels)');
ylabel('v (pixels)');
title('2D Image Projection (Visible Points)');
axis([0 image_width 0 image_height]); % Set axis limits to image dimensions
set(gca, 'YDir','reverse'); 
grid on;
axis equal;

% plot3(trajectory_3D(1,:), trajectory_3D(2,:), trajectory_3D(3,:), 'b-o');
% xlabel('X (meters)');
% ylabel('Y (meters)');
% zlabel('Z (meters)');
% title('3D Object Trajectory');
% grid on;
% axis equal; 
% view(30, 30); % Adjust viewing angle



%% Section 08: Output
disp('First 5 2D pixel coordinates (if in FoV):');
disp(trajectory_2D_pixels(:, find(is_in_FoV, 5, 'first')));



