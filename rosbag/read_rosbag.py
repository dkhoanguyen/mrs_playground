import bagpy
from bagpy import bagreader
import pandas as pd
import pickle


def parse_ros_data(data):
    lines = data.strip().split('\n')
    result = {}
    stack = [result]

    for line in lines:
        indent_level = len(line) - len(line.lstrip())
        line = line.strip()

        if ':' in line:
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip()

            if value == '':
                value = {}
                stack[-1][key] = value
                stack.append(value)
            else:
                if value.isdigit():
                    value = int(value)
                elif value.replace('.', '', 1).isdigit() and 'e' not in value.lower():
                    value = float(value)
                elif value.lower() in ['true', 'false']:
                    value = value.lower() == 'true'
                elif value.startswith('"') and value.endswith('"'):
                    value = value[1:-1]
                stack[-1][key] = value

        else:
            stack = stack[:indent_level // 2 + 1]

    return result


def extract_tf_messages_from_bag(bag_file):
    # Initialize a dictionary to store transformations by topic
    all_data = []

    # Read the CSV file containing the /tf messages
    tf_df = pd.read_csv(f"rosbag/{bag_file}/tf.csv")

    # Iterate over each row in the dataframe
    for index, row in tf_df.iterrows():
        data = row["transforms"]
        data_dict = parse_ros_data(data)
        data_dict["Time"] = row["Time"]
        all_data.append(data_dict)
    return all_data


# Example usage
bag_file = 'seventh_run'
all_tf_data = extract_tf_messages_from_bag(bag_file)
with open(f"rosbag/extracted_data/{bag_file}.pickle", 'wb') as file:
    pickle.dump(all_tf_data, file)
