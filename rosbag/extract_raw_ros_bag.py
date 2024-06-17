import bagpy
from bagpy import bagreader
import pandas as pd
from collections import defaultdict
import yaml

bag_file = "ninth"
# Create a bagreader instance
b = bagreader(f"rosbag/{bag_file}.bag")

# Read the messages from the /tf topic
tf_msgs = b.message_by_topic('/tf')
