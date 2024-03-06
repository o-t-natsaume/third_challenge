#ifndef THEARD_CHALLENGE_NATSUME
#define THEARD_CHALLENGE_NATSUME 


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory>
#include <optional>
#include <nav_msgs/msg/odometry.hpp>
#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <image_geometry/pinhole_camera_model.h>
#include <functional>  // bind & placeholders用
#include <chrono>
#include <tf2/utils.h>


using namespace std::chrono_literals;

class ThirdChallenge : public rclcpp::Node
{
    public:
        ThirdChallenge();
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr yolo_pub_;
        rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr cmd_vel_pub_; // 制御入力
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_; // カメラ情報の取得
        rclcpp::TimerBase::SharedPtr timer_;

       std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
       std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
       std::string target_frame_;
       geometry_msgs::msg::Quaternion output_pose_; 
        roomba_500driver_meiji::msg::RoombaCtrl cmd_vel_;
        
        image_geometry::PinholeCameraModel camera_model_;
        double frontal_threshold;
        double base_omega_;
        double target_theta_;
        double prev_theta_;
        double current_angle_;
        std::chrono::steady_clock::time_point start_time;  
        void timer_callback();
        void box_callback(const std_msgs::msg::Float32MultiArray& msg);
        void camera_info_callback(const sensor_msgs::msg::CameraInfo& msg);
      
        void run(float velocity, float omega);
      
};



#endif