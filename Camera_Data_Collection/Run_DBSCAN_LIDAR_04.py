import numpy as np 
import csv 
import os 
import struct 
from sklearn.cluster import DBSCAN

# Confirugation 
Session_Folder = r"C:\Users\Dulmith Pitigalage\Thesis_C\Data_Sessions\Session_2026-03-17_17-11-01"
BATCH_Window = 0.1 # Accumulate points for 0.1 seconds before clustering

# DBSCAN Parameters 
EPS = 0.3        # Clustering Radius (metres)
MIN_SAMPLES = 1  # Minimum points for cluster

# ROI filter (metres), the flight corridor 
# Flight Corridor Mask
ROI_X_MIN, ROI_X_MAX = 0, 12.0   
ROI_Y_MIN, ROI_Y_MAX = -10.0, 1.75 
ROI_Z_MIN, ROI_Z_MAX = 0.01, 7.0    

# Bird size filter (metres) 
Min_DIM = 0.02
Max_DIM = 0.6

Point_size = 20 # bytes per LiDAR point 

# Bird size Filter 
def is_valid_cluster(cluster_points): 
    x, y, z = cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2]

    length = x.max() - x.min()
    width  = y.max() - y.min()
    height = z.max() - z.min()

    if length < Min_DIM or length > Max_DIM: return False
    if width  < Min_DIM or width  > Max_DIM: return False 
    if height < Min_DIM or height > Max_DIM: return False 

    return True 

# Run DBSCAN on one packet 
# One LiDAR Packet -> 96 points sharing the same timestamp. 
# Inputs: curr timestamp, list of points, list to append results to 
def process_packet(packet_points, detections): 
    if len(packet_points) < MIN_SAMPLES: 
        return 
    
    # 1. Split times and xyz coordinates 
    timestamps = packet_points['timestamp']
    xyz = np.column_stack((packet_points['x'], packet_points['y'], packet_points['z']))

    # 2. Keep only points that fall within the defined flight corridor 
    ROI_mask = (
        (xyz[:, 0] >= ROI_X_MIN) & (xyz[:, 0] <= ROI_X_MAX) & 
        (xyz[:, 1] >= ROI_Y_MIN) & (xyz[:, 1] <= ROI_Y_MAX) &
        (xyz[:, 2] >= ROI_Z_MIN) & (xyz[:, 2] <= ROI_Z_MAX) 
    )
    
    # Apply mask to delete left wall & floor pionts 
    xyz = xyz[ROI_mask]
    timestamps = timestamps[ROI_mask]

    # If xyz is empty after the ROI filter, skip the DBSCAN 
    if len(xyz) < MIN_SAMPLES: 
        return 
    
    # 3. Execute clustering math. 
    # labels -> [0, 0, -1, 1, 1, -1]. Which cluster each point belongs to (-1 means noise, point had no cluster)
    clusterer = DBSCAN(eps = EPS, min_samples = MIN_SAMPLES)
    labels = clusterer.fit_predict(xyz)  

    unique_labels = set(labels) # How many unique clusters were found 
    unique_labels.discard(-1) # Remove noise 

    for label in unique_labels: 

        cluster_mask = (labels == label)
        cluster_points = xyz[cluster_mask]
        cluster_times = timestamps[cluster_mask] # The exact times the laser hit THIS bird

        # Send cluster to size filter. 
        if len(cluster_points) > 1 and not is_valid_cluster(cluster_points):
            continue # Skip rest of the loop. move to next cluster

        # Calculate geometric centroid by averaging each coordinate
        c_x = cluster_points[:, 0].mean()
        c_y = cluster_points[:, 1].mean() 
        c_z = cluster_points[:, 2].mean()
        c_time = cluster_times.mean()  # exact microsecond center of time for this bird


        detections.append([c_time, c_x, c_y, c_z])

def report_progress(points_processed, total_points, last_reported_percentage): 
    percentage = (points_processed * 100) // total_points 
    percentage_10 = (percentage // 10) * 10 # Rounds down, e.g.: 47 -> 40, 83 -> 80

    if percentage_10 != last_reported_percentage: 
        print(f"\rProgress: {percentage_10}% ({points_processed:,} / {total_points:,})", end = "", flush = True)
        last_reported_percentage = percentage_10
    
    return last_reported_percentage

# Main 
# Find .bin file. Open it. Stream raw bytes. 
# Grab 20 bytes. Decode into a point. Store it. 
# Continue until timestamp changes (full packet received). 
# Send the packet to process_packet. 
# process_packet -> Execute clustering math. Find unique clusters. 
#                   For each unique cluster, send to is_valid_cluster. 
#                   If valid, find centroid x, y, z by averaging. 
#                   Save to detections.  
if __name__ == "__main__": 
    
    # 1. Find .bin file 
    bin_file = None 
    for f in os.listdir(Session_Folder): 
        if f.endswith(".bin"):
            # When found, save the path of the bin file. 
            bin_file = os.path.join(Session_Folder, f)
            break 
    
    if bin_file is None: 
        print("Error: No .bin file found.")
        exit() 
    
    print(f"Processing: {os.path.basename(bin_file)}")

    # 2. Define C++ Struct for NumPy 
    # Tells NumPy how to read 20 bytes (8, 4, 4, 4) 
    point_format = np.dtype([
        ('timestamp', '<f8'), 
        ('x', '<f4'),
        ('y', '<f4'),
        ('z', '<f4'),
    ])

    # 3. Read whole file into RAM 
    print("Loading binary file into RAM.")
    raw_data = np.fromfile(bin_file, dtype = point_format)
    total_points = len(raw_data)
    print(f"Succesfully loaded {total_points:,} points. Start clustering algorithm.")

    detections = []

    if total_points > 0: 
        # 4. Batching Logic 
        current_batch_start_time = raw_data['timestamp'][0] # Clock start for 0.1s batch window 
        batch_start_point_index = 0

        # Iterate through every point
        for i in range(total_points): 
            current_time = raw_data['timestamp'][i] # Get timestamp of current point 

            # Check if current batch is ready to be processed 
            if (current_time - current_batch_start_time) > BATCH_Window: 

                # Slice array to obtain points in the 0.1s window 
                batch = raw_data[batch_start_point_index : i]
                process_packet(batch, detections)

                # Reset for next batch 
                batch_start_point_index = i
                current_batch_start_time = current_time 
            
            # Print progress print every 20,000,000 points 
            if i % 10000000 == 0: 
                print(f"\rProgress: {(i / total_points) * 100:.1f}%", end = "", flush = True)
        
        # Process final leftover chunk 
        final_batch = raw_data[batch_start_point_index:]
        process_packet(final_batch, detections)

    # 5. Open a new CSV file
    output_csv = os.path.join(Session_Folder, "LiDAR_detections.csv")
    # Copy the entire detections into the output_csv. 
    with open(output_csv, "w", newline = "") as f: 
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "X_m", "Y_m", "Z_m"])
        writer.writerows(detections) 

    print(f"\nDone. {len(detections)} detections written to LiDAR_detections.csv")