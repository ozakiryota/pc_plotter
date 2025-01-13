import argparse
import rosbag
import sensor_msgs.point_cloud2
import matplotlib.pyplot as plt
import numpy as np
import cv2


class PcPlotter():
    def __init__(self):
        self.args = self.setArgument().parse_args()
        self.num_rows = 2 # xy, yz
        if self.args.target_image_topic != None:
            self.num_rows += 1 # image

    def setArgument(self):
        arg_parser = argparse.ArgumentParser()
        arg_parser.add_argument('--read_rosbag_path', type=str, required=True)
        arg_parser.add_argument('--target_pc_topic', type=str, required=True)
        arg_parser.add_argument('--target_image_topic', type=str)
        arg_parser.add_argument('--num_show', type=int, default=1)
        arg_parser.add_argument('--interval_sec', type=int, default=0)
        return arg_parser
    
    def plot(self):
        point_x_list = []
        point_y_list = []
        point_z_list = []
        first_timestamp = None
        last_timestamp = None
        fig = plt.figure()
        pc_counter = 0
        image_counter = 0

        bag = rosbag.Bag(self.args.read_rosbag_path)
        for topic_name, msg, timestamp in bag.read_messages():
            if topic_name == self.args.target_pc_topic and pc_counter < self.args.num_show:
                if first_timestamp == None:
                    first_timestamp = timestamp
                if last_timestamp == None or (timestamp.to_sec() - last_timestamp.to_sec()) > self.args.interval_sec:
                    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z', 'intensity')):
                        point_x_list.append(point[0])
                        point_y_list.append(point[1])
                        point_z_list.append(point[2])
                    last_timestamp = timestamp
                    ## plot-xy
                    plt.subplot(self.num_rows, self.args.num_show, self.rowColToIndex(0, pc_counter))
                    plt.title("timestamp: " + '{:.1f}'.format(timestamp.to_sec() - first_timestamp.to_sec()) + " [s]")
                    plt.xlabel('y [m]')
                    plt.ylabel('x [m]')
                    plt.plot(point_y_list, point_x_list, linestyle='None', marker='.', color = "black")
                    ## plot-yz
                    plt.subplot(self.num_rows, self.args.num_show, self.rowColToIndex(1, pc_counter))
                    plt.title("#points: " + str(msg.height * msg.width))
                    plt.xlabel('x [m]')
                    plt.ylabel('z [m]')
                    plt.plot(point_x_list, point_z_list, linestyle='None', marker='.', color = "black")
                    ## counter
                    pc_counter += 1
            elif topic_name == self.args.target_image_topic and image_counter < self.args.num_show:
                if image_counter < pc_counter:
                    image_np = np.frombuffer(msg.data, np.uint8)
                    image_cv = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
                    plt.subplot(self.num_rows, self.args.num_show, self.rowColToIndex(2, image_counter))
                    plt.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False)
                    plt.tick_params(bottom=False, left=False, right=False, top=False)
                    plt.imshow(cv2.cvtColor(image_cv, cv2.COLOR_BGR2RGB))
                    image_counter += 1

        plt.tight_layout()
        plt.show()

    def rowColToIndex(self, row, col):
        return self.args.num_show * row + col + 1


def run():
    pc_plotter = PcPlotter()
    pc_plotter.plot()


if __name__ == '__main__':
    run()