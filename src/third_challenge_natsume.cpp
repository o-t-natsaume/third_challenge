#include "third_challenge/third_challenge_natsume.hpp"
#include <math.h>

ThirdChallenge::ThirdChallenge():Node("third_challenge_natsume")
{
    printf("constructor\n");
    timer_ = this->create_wall_timer(500ms, std::bind(&ThirdChallenge::timer_callback, this));  
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    yolo_pub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/ros2_yolo/box",
     rclcpp::QoS(1).reliable(),
     std::bind(&ThirdChallenge::box_callback,this,std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("/roomba/control",rclcpp::QoS(1).reliable()); 
   
    camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/color/camera_info",
        rclcpp::QoS(1).reliable(),
        std::bind(&ThirdChallenge::camera_info_callback,this,std::placeholders::_1)
    );
    
    base_omega_ = 0.5;
    
}

void ThirdChallenge::camera_info_callback(const sensor_msgs::msg::CameraInfo& msg){
    printf("camera_info_callback\n");
    camera_model_.fromCameraInfo(msg);
}


// 一定の周期で呼び出され，制御指令を生成する関数
// ルンバを目標方位角の方に旋回させる
void ThirdChallenge::timer_callback()
{
    printf("timer_call_back\n");
    
    if(target_theta_ == 0){
        printf("theta=0");
        run(0,0);
        return;
    }


    // ルンバの座標からベースリンク座標への変換を取得する
    geometry_msgs::msg::TransformStamped transform;

    try{
        // ルンバのローカル座標から
        // 起動位置からの絶対座標に変換するトランスフォームを取得する
        transform = tf_buffer_->lookupTransform(
            "odom","base_link",tf2::TimePointZero
        );
    } catch (const tf2::TransformException & ex) {
        printf("トランスフォーム取得エラー");
        run(0,0);
        return;
    }

    geometry_msgs::msg::Quaternion robot_quat;
    // 座標を変換する
    // ローカル座標のルンバがオドム座標系でどのように見えるかを算出する
    // 例：前回ルンバが４０度に見えた場合、ルンバが-３０度回転したいたら１０度となる
    // つまり、ルンバの回転方向は指定された座標とは逆回転で正面を向いたと判定する
    tf2::doTransform(output_pose_,robot_quat,transform);

    // クォータニオンからオイラー角度に変換
    double yaw = tf2::getYaw(robot_quat);

    printf("よー %f",yaw);
    if(std::abs(yaw) >= 0.5){
       if(yaw < 0){
        run(0,0.5); 
       }else{
        run(0,-0.5);
       }

    }else{
        run(0,0);
    }

    
}




void ThirdChallenge::run(float velocity, float omega)
{
    // roombaの制御モード
    // 基本的に11（DRIVE_DIRECT）固定で良い
    cmd_vel_.mode = 11;

    // 並進速度を指定
    cmd_vel_.cntl.linear.x = velocity;
    // 旋回速度を指定
    cmd_vel_.cntl.angular.z = omega;

    // cmd_velをpublish
    // <publisher名>->publish(<変数名>);
    // ここに書く
    cmd_vel_pub_->publish(cmd_vel_);
}


void ThirdChallenge::box_callback(const std_msgs::msg::Float32MultiArray& msg){
    

    printf("box_call_bask\n") ;


    if (msg.data.size() == 2) // 長さが2の場合，検出は無し
    {
        target_theta_ = 0.0;
        printf("no target \n");
        return;
    }
  
    float x = (msg.data[4] + msg.data[2]) / 2;
    float base_x = (x - camera_model_.cx()) / camera_model_.fx();
 
    target_theta_  = std::atan2(base_x, 1.0);

    tf2::Quaternion q;
    q.setRPY(0, 0, target_theta_);
    // オイラー角をクォータニオンに変換後、正規化する
    q.normalize();
    // オイラー角をクォータニオンに変換
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q);

    // ロボットからルンバへの座標変換用の座標を取得
    geometry_msgs::msg::TransformStamped transform;
    try{
        // 起動した場所の絶対座標からローカル座標へ変換を行うためのトランスフォームを取得する
        // base_linkはルンバのローカル座標
        // odomは起動位置からの絶対座標
        transform = tf_buffer_->lookupTransform(
            "base_link","odom",tf2::TimePointZero
        );
    } catch (const tf2::TransformException & ex) {
          return;
    }

    // ルンバ座標に変換した時の座標を格納しておく
    tf2::doTransform(msg_quat,output_pose_,transform);
}





