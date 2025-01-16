#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include "/tmp/matplotlib-cpp/matplotlibcpp.h"

class TemperatePlotter
{
private:
    /*node handle*/
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    /*buffer*/
    float first_timestamp_sec_;
    /*parameter*/
    std::string read_rosbag_path_;
    std::string write_image_path_;
    std::string target_pc_topic_;
    float interval_sec_;
    /*function*/
    std::string getDefaultCsvPath();

public:
    TemperatePlotter();
    void run();
};

TemperatePlotter::TemperatePlotter()
    : nh_private_("~"), first_timestamp_sec_(-1)
{
    std::cout << "----- temperature_plotter -----" << std::endl;

    /*parameter*/
    if (!nh_private_.getParam("read_rosbag_path", read_rosbag_path_))
    {
        std::cerr << "Set read_rosbag_path." << std::endl;
        exit(true);
    }
    std::cout << "read_rosbag_path_ = " << read_rosbag_path_ << std::endl;
    nh_private_.param("write_image_path", write_image_path_, getDefaultCsvPath());
    std::cout << "write_image_path_ = " << write_image_path_ << std::endl;
    nh_private_.param("target_pc_topic", target_pc_topic_, std::string(""));
    std::cout << "target_pc_topic_ = " << target_pc_topic_ << std::endl;
    nh_private_.param("interval_sec", interval_sec_, 0.0f);
    std::cout << "interval_sec_ = " << interval_sec_ << std::endl;
}

std::string TemperatePlotter::getDefaultCsvPath()
{
    const char *tmp = getenv("ROS_WORKSPACE");
    std::string output_path(tmp ? tmp : "");
    if (output_path.empty())
    {
        std::cerr << "Cannot get $ROS_WORKSPACE." << std::endl;
        exit(true);
    }
    output_path += std::string("/src/pc_plotter/output/") + std::filesystem::path(read_rosbag_path_).stem().string() + ".png";
    return output_path;
}

void TemperatePlotter::run()
{
    rosbag::Bag bag;

    try
    {
        bag.open(read_rosbag_path_, rosbag::bagmode::Read);
    }
    catch (rosbag::BagException const &)
    {
        std::cerr << "Cannot open " << read_rosbag_path_ << std::endl;
        exit(true);
    }

    rosbag::View view;
    view.addQuery(bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
    rosbag::View::iterator view_itr;
    view_itr = view.begin();

    std::vector<float> time_list;
    std::vector<float> num_points_list;
    float last_timestamp_sec;
    uint64_t num_points_counter = 0;
    size_t num_pc_counter = 0;
    while (view_itr != view.end())
    {
        if (target_pc_topic_ == "" || view_itr->getTopic() == target_pc_topic_)
        {
            const sensor_msgs::PointCloud2ConstPtr pc_ptr = view_itr->instantiate<sensor_msgs::PointCloud2>();
            const float &curr_timestamp_sec = pc_ptr->header.stamp.toSec();
            if (first_timestamp_sec_ < 0)
            {
                first_timestamp_sec_ = curr_timestamp_sec;
                last_timestamp_sec = curr_timestamp_sec;
            }
            num_points_counter += pc_ptr->height * pc_ptr->width;
            num_pc_counter++;
            if (curr_timestamp_sec - last_timestamp_sec > interval_sec_)
            {
                time_list.push_back(curr_timestamp_sec - first_timestamp_sec_);
                num_points_list.push_back(num_points_counter / (float)num_pc_counter);
                last_timestamp_sec = curr_timestamp_sec;
                num_points_counter = 0;
                num_pc_counter = 0;
            }
        }
        view_itr++;
    }

    matplotlibcpp::plot(time_list, num_points_list, "k");
    matplotlibcpp::xlabel("time [s]");
    matplotlibcpp::ylabel("#points [-]");
    matplotlibcpp::save(write_image_path_);
    matplotlibcpp::show();

    bag.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "temperature_plotter");

    TemperatePlotter temperature_plotter;
    temperature_plotter.run();
}