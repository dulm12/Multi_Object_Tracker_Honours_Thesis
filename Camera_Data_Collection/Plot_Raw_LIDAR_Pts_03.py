import numpy as np
import os
import open3d as o3d
import matplotlib.pyplot as plt 

# Configuration
bin_file = r"C:\Users\Dulmith Pitigalage\Thesis_C\Data_Sessions\Session_2026-04-01_16-35-45\lidar_raw_2026_04_01--16_35_52.bin"

# Flight Corridor Mask
ROI_X_MIN, ROI_X_MAX = 0, 12.0   
ROI_Y_MIN, ROI_Y_MAX = -10.0, 1.75 
ROI_Z_MIN, ROI_Z_MAX = 0.01, 7.0    

point_format = np.dtype([('timestamp', '<f8'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4')])

print("Loading binary file.")
raw = np.fromfile(bin_file, dtype=point_format)

print("Applying Flight Corridor Mask.")
mask = (
    (raw['x'] >= ROI_X_MIN) & (raw['x'] <= ROI_X_MAX) &
    (raw['y'] >= ROI_Y_MIN) & (raw['y'] <= ROI_Y_MAX) &
    (raw['z'] >= ROI_Z_MIN) & (raw['z'] <= ROI_Z_MAX)
)

zone = raw[mask]
# # Open3D is so fast you can probably plot [::1] (every point), but we'll do [::2] to be safe.
# plot_zone = zone[::1] 

print(f"Total raw points: {len(raw):,}")
print(f"Points in Flight Corridor: {len(zone):,}")

# Extract X, Y, Z into an N x 3 matrix and the timestamps 
xyz = np.column_stack((zone['x'], zone['y'], zone['z']))
timestamps = zone['timestamp']
print(f"Minimum timestamp: {timestamps.min()}s\n")
print(f"Maximum timestamp: {timestamps.max()}s\n")
# normalise the timestamps to 0
t_normalised = timestamps - timestamps.min()
t_max = t_normalised.max()

# Create colour based on time using a colourmap 
# each 30 second interval gets a unique colour 
interval = 3 # seconds 
num_intervals = int(np.ceil(t_max / interval))

# colormap with distinct colours 
# base_colours = plt.get_cmap('tab20').colors; 

base_colours = np.array([
    [0.00, 0.00, 0.80],  # dark blue
    [0.80, 0.00, 0.00],  # dark red
    [0.00, 0.55, 0.00],  # dark green
    [0.55, 0.00, 0.55],  # dark purple
    [0.85, 0.45, 0.00],  # dark orange
    [0.00, 0.60, 0.60],  # teal
    [0.35, 0.20, 0.05],  # brown
    [0.20, 0.20, 0.20],  # dark gray
])

# base_colours = np.array([
#     [0.00, 0.00, 0.80],  # dark blue
#     [0.60, 0.80, 1.00],  # light blue

#     [0.80, 0.00, 0.00],  # dark red
#     [1.00, 0.60, 0.60],  # light red

#     [0.00, 0.55, 0.00],  # dark green
#     [0.60, 0.90, 0.60],  # light green

#     [0.55, 0.00, 0.55],  # dark purple
#     [0, 0, 0.],          # black

#     [0.85, 0.45, 0.00],  # dark orange
#     [0.65, 0.50, 0.00],  # dark yellow 
# ])

np.random.seed(0)
np.random.shuffle(base_colours)

colours = np.zeros((len(zone), 3)); 
for i in range(num_intervals): 
    t_start = i * interval
    t_end   = (i + 1) * interval

    if i == num_intervals - 1: 
        mask_interval = (t_normalised >= t_start) & (t_normalised <= t_end)
    else: 
        mask_interval = (t_normalised >= t_start) & (t_normalised < t_end)

    colour_RGB = base_colours[i % len(base_colours)][:3]
    colours[mask_interval] = colour_RGB

# Create Open3D PointCloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
pcd.colors = o3d.utility.Vector3dVector(colours)

# Create axes (Size = 5 meters long)
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0, origin=[0, 0, 0])

# 3. CREATE THE FLIGHT CORRIDOR BOUNDING BOX
bbox = o3d.geometry.AxisAlignedBoundingBox (
        min_bound=(ROI_X_MIN, ROI_Y_MIN, ROI_Z_MIN), 
        max_bound=(ROI_X_MAX, ROI_Y_MAX, ROI_Z_MAX)
    )
bbox.color = (0.5, 0.5, 0.5) # Red wireframe box

# -the legend 
print("\n" + "="*50)
print("Colour legend (according to time)")
for i in range(num_intervals): 
    t_start = i * interval 
    t_end = (i + 1) * interval 
    rgb = base_colours[i % len(base_colours)][:3]
    count = np.sum((t_normalised >= t_start) & (t_normalised <= t_end))
    print(f"    {t_start:5.0f}s : RGB{rgb[0]:.2f}, {rgb[1]:.2f}, {rgb[2]:.2f}) | {count} points")
print("="*50 + "\n")
o3d.visualization.draw_geometries([pcd, axes, bbox], window_name="LiDAR Flight Corridor")