#include <string>
#include <vector>
#include <chrono>
#include <map>

#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/simple_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <kinematics_toolbox/kinematics.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/ros_helpers.hpp>

#include <cdcpd_ros/Float32MultiArrayStamped.h>

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::Matrix3Xf;
using Eigen::Matrix3Xd;
using Eigen::Isometry3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using pcl::PointXYZ;
using kinematics::Vector6d;

typedef kinematics::VectorVector6d AllGrippersSinglePoseDelta;
typedef EigenHelpers::VectorIsometry3d AllGrippersSinglePose;

namespace gm = geometry_msgs;
namespace sm = sensor_msgs;
namespace stdm = std_msgs;

using namespace cv;
using namespace std::chrono_literals;

std::vector<sm::Image::ConstPtr> color_images;
std::vector<sm::Image::ConstPtr> depth_images;
std::vector<sm::CameraInfo::ConstPtr> camera_infos;
std::vector<cdcpd_ros::Float32MultiArrayStamped::ConstPtr> grippers_config;
std::vector<cdcpd_ros::Float32MultiArrayStamped::ConstPtr> grippers_dot;

std::tuple<cv::Mat, cv::Mat, cv::Matx33d> toOpenCv(
    const sm::Image::ConstPtr &rgb_img,
    const sm::Image::ConstPtr &depth_img,
    const sm::CameraInfo::ConstPtr &cam_info)
{
    #ifdef SIMULATION
    cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_img, sm::image_encodings::TYPE_8UC3);
    #else
    cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_img, sm::image_encodings::BGR8);
    #endif
    cv::Mat color_image = rgb_ptr->image.clone();

    #ifdef SIMULATION
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_img, sm::image_encodings::TYPE_32FC1);
    #else
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_img, sm::image_encodings::TYPE_16UC1);
    #endif
    cv::Mat depth_image = depth_ptr->image.clone();

    image_geometry::PinholeCameraModel cameraModel;
    cameraModel.fromCameraInfo(cam_info);

    return { color_image, depth_image, cameraModel.fullIntrinsicMatrix() };
}

std::tuple<AllGrippersSinglePose,
           AllGrippersSinglePoseDelta> //,MatrixXi>
           toGripperConfig(
    const cdcpd_ros::Float32MultiArrayStamped::ConstPtr &g_config,
    const cdcpd_ros::Float32MultiArrayStamped::ConstPtr &g_dot)
{
    uint32_t num_gripper = (g_config->data.layout).dim[0].size;
    uint32_t num_config = (g_config->data.layout).dim[1].size;
    uint32_t num_dot = (g_dot->data.layout).dim[1].size;

    std::cout << "num of gripper " << num_gripper << std::endl;
    std::cout << "num of config " << num_config << std::endl;
    std::cout << "num of gripper dot " << num_dot << std::endl;

    AllGrippersSinglePose one_frame_config;
    AllGrippersSinglePoseDelta one_frame_velocity;

    for (uint32_t g = 0; g < num_gripper; ++g)
    {
        Isometry3d one_config;
        Vector6d one_velocity;

        for (uint32_t row = 0; row < 4; ++row)
        {
            for (uint32_t col = 0; col < 4; ++col)
            {
                one_config(row, col) = double((g_config->data.data)[num_config*g + row*4 + col]);
            }
        }

        for (uint32_t i = 0; i < num_dot; ++i)
        {
            one_velocity(i) = double((g_dot->data.data)[num_dot*g + i]);
        }

        one_frame_config.push_back(one_config);
        one_frame_velocity.push_back(one_velocity);
    }

    return {one_frame_config, one_frame_velocity};
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cdcpd_bagfile");
    std::cout << "Starting up..." << std::endl;

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    std::vector<std::string> topics;

    topics.emplace_back(std::string("/kinect2_victor_head/qhd/image_color_rect"));
    topics.emplace_back(std::string("/kinect2_victor_head/qhd/image_depth_rect"));
    topics.emplace_back(std::string("/kinect2_victor_head/qhd/camera_info"));
    topics.emplace_back(std::string("/kinect2_victor_head/qhd/dot_config"));
    topics.emplace_back(std::string("/kinect2_victor_head/qhd/gripper_config"));

    std::cout << "topics created" << std::endl;

    auto const bagfile = ROSHelpers::GetParam<std::string>(ph, "bagfile", "normal");
    auto const folder = "/home/victorrope/catkin_ws_cdcpd/src/cdcpd_test/dataset/";
    std::cout << "about to read bag" << std::endl;
    rosbag::Bag bag;
    bag.open(folder + bagfile + ".bag", rosbag::bagmode::Read);
    std::cout << "after open bag" << std::endl;
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::cout << "about to read" << std::endl;
    for(rosbag::MessageInstance const& m: view)
    {
        if (m.getTopic() == topics[0])
        {
            auto i = m.instantiate<sm::Image>();
            if (i != nullptr)
            {
                color_images.emplace_back(i);
            }
            else
            {
                std::cout << "NULL initiation!" << std::endl;
            }
        }
        else if (m.getTopic() == topics[1])
        {
            auto i = m.instantiate<sm::Image>();
            if (i != nullptr)
            {
                depth_images.emplace_back(i);
            }
            else
            {
                std::cout << "NULL initiation!" << std::endl;
            }
        }
        else if (m.getTopic() == topics[2])
        {
            auto i = m.instantiate<sm::CameraInfo>();
            if (i != nullptr)
            {
                camera_infos.emplace_back(i);
            }
            else
            {
                std::cout << "NULL initiation!" << std::endl;
            }
        }
        else if (m.getTopic() == topics[3])
        {
            auto i = m.instantiate<cdcpd_ros::Float32MultiArrayStamped>();
            if (i != nullptr)
            {
                grippers_dot.emplace_back(i);
            }
            else
            {
                std::cout << "NULL initiation!" << std::endl;
            }
        }
        else if (m.getTopic() == topics[4])
        {
            auto i = m.instantiate<cdcpd_ros::Float32MultiArrayStamped>();
            if (i != nullptr)
            {
                grippers_config.emplace_back(i);
            }
            else
            {
                std::cout << "NULL initiation!" << std::endl;
            }
        }
        else
        {
            std::cerr << "Invalid topic: " << m.getTopic() << std::endl;
        }
    }
    bag.close();

    auto color_iter = color_images.cbegin();
    auto depth_iter = depth_images.cbegin();
    auto info_iter = camera_infos.cbegin();
    auto config_iter = grippers_config.cbegin();
    auto velocity_iter = grippers_dot.cbegin();

    std::cout << "rgb images size: " << color_images.size() << std::endl;
    std::cout << "depth images size: " << depth_images.size() << std::endl;
    std::cout << "camera infos size: " << camera_infos.size() << std::endl;
    std::cout << "gripper configuration size: " << grippers_config.size() << std::endl;
    std::cout << "gripper velocity size: " << grippers_dot.size() << std::endl;

    auto [g_config, g_dot] = toGripperConfig(*config_iter, *velocity_iter);
    auto [color_image_bgr, depth_image, intrinsics] = toOpenCv(*color_iter, *depth_iter, *info_iter);

    std::cout << "rbg image size: " << color_image_bgr.rows << "-" << color_image_bgr.cols << std::endl;
    std::cout << "depth image size: " << depth_image.rows << "-" << depth_image.cols << std::endl;
    std::cout << "intrinsics: " << std::endl;
    std::cout << intrinsics << std::endl << std::endl;
    std::cout << "gripper configuration: " << std::endl;
    std::cout << g_config[0].matrix() << std::endl << std::endl;
    std::cout << g_config[1].matrix() << std::endl << std::endl;
    std::cout << "gripper velocity: " << std::endl;
    std::cout << g_dot[0].matrix() << std::endl << std::endl;
    std::cout << g_dot[1].matrix() << std::endl << std::endl;

    return 0;
}
