import rosbag
from tf2_msgs.msg import TFMessage
import pickle
import numpy as np

name = 'fifth'
bag = rosbag.Bag(f"{name}.bag")

# Extract names first
all_robots_name = []
all_animals_name = []
for topic, msg, t in bag.read_messages(topics='/tf'):
    if "cf" in msg.transforms[0].child_frame_id and msg.transforms[0].child_frame_id not in all_robots_name:
        all_robots_name.append(msg.transforms[0].child_frame_id)
    if "R" in msg.transforms[0].child_frame_id and msg.transforms[0].child_frame_id not in all_animals_name:
        all_animals_name.append(msg.transforms[0].child_frame_id)

all_robots_states = {}
all_animals_states = {}

for robot_name in all_robots_name:
    all_robots_states[robot_name] = np.empty((0, 3))
for animal_name in all_animals_name:
    all_animals_states[animal_name] = np.empty((0, 3))

for topic, msg, t in bag.read_messages(topics=['/tf']):
    if msg.transforms[0].child_frame_id in all_animals_name:
        animal_name = msg.transforms[0].child_frame_id
        x = msg.transforms[0].transform.translation.x
        y = msg.transforms[0].transform.translation.y
        z = msg.transforms[0].transform.translation.z
        pose = np.array([x, y, z])
        all_animals_states[animal_name] = np.vstack(
            (all_animals_states[animal_name], pose))
    if msg.transforms[0].child_frame_id in all_robots_name:
        robot_name = msg.transforms[0].child_frame_id
        x = msg.transforms[0].transform.translation.x
        y = msg.transforms[0].transform.translation.y
        z = msg.transforms[0].transform.translation.z
        pose = np.array([x, y, z])
        all_robots_states[robot_name] = np.vstack(
            (all_robots_states[robot_name], pose))

all_states = {}
all_states["animals"] = all_animals_states
all_states["robot"] = all_robots_states

with open(f'{name}.pickle', 'wb') as handle:
    pickle.dump(all_states, handle, protocol=pickle.HIGHEST_PROTOCOL)
bag.close()
