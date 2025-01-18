#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "/home/user/matplotlib-cpp/matplotlibcpp.h"

class PcPlotter
{
private:
    /*node handle*/
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    /*parameter*/
    std::string read_rosbag_path_;
    std::string write_image_path_;
    std::string target_pc_topic_;
    float interval_sec_;
    int num_show_;
    /*function*/
    std::string getDefaultOutputPath();

public:
    PcPlotter();
    void run();
};

PcPlotter::PcPlotter()
    : nh_private_("~")
{
    std::cout << "----- pc_plotter -----" << std::endl;

    /*parameter*/
    if (!nh_private_.getParam("read_rosbag_path", read_rosbag_path_))
    {
        std::cerr << "Set read_rosbag_path." << std::endl;
        exit(true);
    }
    std::cout << "read_rosbag_path_ = " << read_rosbag_path_ << std::endl;
    nh_private_.param("write_image_path", write_image_path_, getDefaultOutputPath());
    std::cout << "write_image_path_ = " << write_image_path_ << std::endl;
    nh_private_.param("target_pc_topic", target_pc_topic_, std::string(""));
    std::cout << "target_pc_topic_ = " << target_pc_topic_ << std::endl;
    nh_private_.param("interval_sec", interval_sec_, 0.0f);
    std::cout << "interval_sec_ = " << interval_sec_ << std::endl;
    nh_private_.param("num_show", num_show_, 1);
    std::cout << "num_show_ = " << num_show_ << std::endl;
}

std::string PcPlotter::getDefaultOutputPath()
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

void PcPlotter::run()
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

    size_t show_counter = 0;
    float first_timestamp_sec_ = -1;
    float last_timestamp_sec_ = -1;
    std::vector<float> pc_x_list;
    std::vector<float> pc_y_list;
    std::vector<float> pc_z_list;
    while (view_itr != view.end())
    {
        if (target_pc_topic_ == "" || view_itr->getTopic() == target_pc_topic_)
        {
            const sensor_msgs::PointCloud2::Ptr msg_ptr = view_itr->instantiate<sensor_msgs::PointCloud2>();
            const float &curr_timestamp_sec = msg_ptr->header.stamp.toSec();
            if (first_timestamp_sec_ < 0)
                first_timestamp_sec_ = curr_timestamp_sec;
            if (last_timestamp_sec_ < 0 || curr_timestamp_sec - last_timestamp_sec_ > interval_sec_)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr pc_pcl(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromROSMsg(*msg_ptr, *pc_pcl);
                pc_x_list.resize(pc_pcl->points.size());
                pc_y_list.resize(pc_pcl->points.size());
                pc_z_list.resize(pc_pcl->points.size());
                for (size_t i = 0; const pcl::PointXYZI &point : pc_pcl->points)
                {
                    pc_x_list[i] = point.x;
                    pc_y_list[i] = point.y;
                    pc_z_list[i] = point.z;
                    i++;
                }
                last_timestamp_sec_ = curr_timestamp_sec;
                show_counter++;
                matplotlibcpp::subplot(2, num_show_, show_counter);
                matplotlibcpp::title(std::to_string(curr_timestamp_sec - first_timestamp_sec_));
                matplotlibcpp::xlabel("y [m]");
                matplotlibcpp::ylabel("x [m]");
                matplotlibcpp::plot(pc_y_list, pc_x_list, ".k");
                matplotlibcpp::subplot(2, num_show_, show_counter + num_show_);
                matplotlibcpp::xlabel("x [m]");
                matplotlibcpp::ylabel("z [m]");
                matplotlibcpp::plot(pc_x_list, pc_z_list, ".k");
                if (show_counter == num_show_)
                    break;
            }
        }
        view_itr++;
    }
    bag.close();

    matplotlibcpp::save(write_image_path_);
    matplotlibcpp::show();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_plotter");

    PcPlotter pc_plotter;
    pc_plotter.run();
}