import rosbag
import rospy
from tqdm import tqdm
import sys
import numpy as np
import matplotlib.pyplot as plt
from math import atan2, sin, cos
from relative_nav.msg import NodeInfo

inbag = rosbag.Bag('/home/superjax/rosbag/small_loop.bag', mode='r')
outbag = rosbag.Bag('/home/superjax/rosbag/small_loop.new.bag', mode='w')

for topic, msg, t in tqdm(inbag.read_messages(), total=inbag.get_message_count()):
  if topic == '/keyframe':
      node_msg = NodeInfo()
      node_msg.header = msg.header
      node_msg.node_id = node_msg.keyframe_id = msg.keyframe_id
      node_msg.node_to_body.w = 1.0
      node_msg.node_to_body.x = 0.0
      node_msg.node_to_body.y = 0.0
      node_msg.node_to_body.z = 0.0
      node_msg.camera_to_body.rotation = node_msg.node_to_body
      node_msg.camera_to_body.translation.x = 0.0
      node_msg.camera_to_body.translation.y = 0.0
      node_msg.camera_to_body.translation.z = 0.0
      outbag.write('/node', node_msg, t)

  outbag.write(topic, msg, t)

outbag.close()
outbag.reindex()

