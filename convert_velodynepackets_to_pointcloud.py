import velodyne_decoder as vd
import rosbag
import rospy 
import struct
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from velodyne_msgs.msg import VelodyneScan
from std_msgs.msg import Header
from tqdm import tqdm  # Import tqdm for progress bar


output_bag = "out_anymal_1.bag"
input_bag = './anymal_1/2021-09-23-16-31-51_anymal-cerberus_mission_raw_sensors.bag'
lidar_topics = ['/lidar/packets']

# Function to process Velodyne packets and extract point cloud data
def process_lidar_packets(cloud, seq):

    timestamp = cloud[0].device
    points = []
    for point_data in cloud[1]:
        x,y,z = point_data[0:3]
        intensity = point_data[3]
        time = point_data[4]
        ring = point_data[6]
        points.append((x, y, z, intensity, int(ring), time))

    # Create PointCloud2 message
    header = Header()
    header.seq = seq
    header.frame_id = 'lidar'
    header.stamp = rospy.Time.from_sec(timestamp)
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        PointField('ring', 16, PointField.UINT16, 1),  # Store ring ID
        PointField('time', 18, PointField.FLOAT32, 1)  # Store time per point
    ]

    cloud_msg = pc2.create_cloud(header, fields, points)
    return cloud_msg, header.stamp

# Read packets using vd library
cloud_arrays = []
for points in tqdm(vd.read_bag(input_bag, topics=lidar_topics), desc="Processing LiDAR Data"):
    cloud_arrays.append(points)

with rosbag.Bag(output_bag, "w") as out_bag:
    for seq, cloud in tqdm(enumerate(cloud_arrays), desc="Writing PointClouds to Bag", total=len(cloud_arrays)):
        pointcloud_msg, t = process_lidar_packets(cloud, seq)
        out_bag.write("/pointcloud", pointcloud_msg, t)

print(f"Conversion complete: {output_bag}")

#timestamp = cloud_arrays[i][0].device
#x,y,z = cloud_arrays[i][1][j][0:3]
#intensity = cloud_arrays[i][1][j][3]
#time = cloud_arrays[i][1][j][4]
#ring = cloud_arrays[i][1][j][6]
#print(timestamp)
#print("x,y,z", x,y,z)
#print("i: ", intensity, "t: ",  time, "r: ",  ring)
#print(cloud_arrays[i][1][j])
