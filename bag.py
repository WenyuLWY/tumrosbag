#! /usr/bin python
import rosbag
from sensor_msgs.msg import PointCloud2


# bag = rosbag.Bag('/home/Downloads/dynamic_warehouse.bag')
# for topic, msg, t in bag.read_messages():
#     if topic == "/camera/depth/color/points" and isinstance(msg, PointCloud2):
#         fields = msg.fields
#         for field in fields:
#             print("Field Name:", field.name)
#             print("Field Type:", field.datatype)

bag = rosbag.Bag('/home/Downloads/dynamic_warehouse.bag','r')
# for topic, msg, t in bag.read_messages():
#     if topic == "/camera/depth/color/points" :
#         fields = msg.fields
#         for field in fields:
#             print("Field Name:", field.name)
#             print("Field Type:", field.datatype)

for topic, msg, t in bag.read_messages():
    print("once start")
    if topic == "/camera/color/image_raw":
        print("image start")
        # print(msg.header.stamp.to_sec())
        print(t)
        print("image end")
    if topic == "/camera/depth/color/points":
        print("pointcloud start")
        # print(msg.header.stamp.to_sec())
        print(t)
        print("pointcloud end")
    print("once end")

bag.close()