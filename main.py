import rosbag
import rospy

import roslib
roslib.load_manifest('amrl_msgs')
roslib.load_manifest('geometry_msgs')

from std_msgs.msg import Int32, String
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseWithCovarianceStamped
from amrl_msgs.msg import Localization2DMsg

import math
import os


def convert_bag_files(input_fn, output_fn):
  bag = rosbag.Bag(input_fn, "r")
  out_bag = rosbag.Bag(output_fn, 'w')
  try:
    seq = 0 # aribtray starting point
    for topic, msg, t in bag.read_messages():
        if topic == "/initialpose":       
          out_bag.write(topic, msg, t)
          topic = "/set_pose"
          new_msg = Localization2DMsg()
          new_msg.pose.x = msg.pose.pose.position.x
          new_msg.pose.y = msg.pose.pose.position.y
          new_msg.pose.theta = 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
          new_msg.header.seq = seq
          new_msg.header.stamp.secs = int(t.to_nsec() * 10**-9)
          new_msg.header.stamp.nsecs = t.to_nsec() % (10**9)
          new_msg.header.frame_id = "GDC1"
          new_msg.map = "GDC1"        
          msg=new_msg   
          out_bag.write(topic, msg, t)
          seq += 1
        elif topic == "/reference_localization":
          new_msg = Localization2DMsg()
          new_msg.pose.x = msg.x
          new_msg.pose.y = msg.y
          new_msg.pose.theta = msg.theta
          new_msg.header.seq = seq
          new_msg.header.stamp.secs = int(t.to_nsec() * 10**-9)
          new_msg.header.stamp.nsecs = t.to_nsec() % (10**9)
          new_msg.header.frame_id = "GDC1"
          new_msg.map = "GDC1"
          msg=new_msg
          out_bag.write(topic, msg, t)        
          seq += 1
        else:
          out_bag.write(topic, msg, t)
    print "conversion of {} to {} is complete".format(input_fn, output_fn)
  finally:
    bag.close()
    out_bag.close()

def main():
  in_directory = "input"
  out_directory = "output"
  print "Warning: Please make sure all files in {} are bag files".format(in_directory)
  for entry in os.listdir(in_directory):
      convert_bag_files("{}/{}".format(in_directory, entry), "{}/{}".format(out_directory, entry))

if __name__ == "__main__":
  main()
