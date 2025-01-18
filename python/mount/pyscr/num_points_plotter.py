import argparse
import rosbag
import matplotlib.pyplot as plt
import os


class NumPointsPlotter():
    def __init__(self):
        self.args = self.setArgument().parse_args()

    def setArgument(self):
        arg_parser = argparse.ArgumentParser()
        arg_parser.add_argument('--read_rosbag_path', type=str, required=True)
        arg_parser.add_argument('--write_image_path', type=str, default='../output/pc_plotter.png')
        arg_parser.add_argument('--target_pc_topic', type=str, required=True)
        arg_parser.add_argument('--interval_sec', type=int, default=0)
        return arg_parser
    
    def plot(self):
        first_timestamp = None
        last_timestamp = None
        num_points_list = []
        ave_num_points_list = []
        timestamp_list = []

        bag = rosbag.Bag(self.args.read_rosbag_path)
        for topic_name, msg, timestamp in bag.read_messages():
            if topic_name == self.args.target_pc_topic:
                if first_timestamp == None:
                    first_timestamp = timestamp
                    last_timestamp = timestamp
                num_points_list.append(msg.height * msg.width)
                if (timestamp.to_sec() - last_timestamp.to_sec()) > self.args.interval_sec:
                    ave_num_points_list.append(sum(num_points_list) / len(num_points_list))
                    timestamp_list.append(timestamp.to_sec() - first_timestamp.to_sec())
                    num_points_list = []
                    last_timestamp = timestamp
        plt.xlabel('time [s]')
        plt.ylabel('#points [-]')
        plt.plot(timestamp_list, ave_num_points_list, color="black")
        plt.tight_layout()
        os.makedirs(os.path.dirname(self.args.write_image_path), exist_ok=True)
        plt.savefig(self.args.write_image_path)
        plt.show()


def run():
    num_points_plotter = NumPointsPlotter()
    num_points_plotter.plot()


if __name__ == '__main__':
    run()