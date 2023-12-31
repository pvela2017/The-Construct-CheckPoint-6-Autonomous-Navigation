#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav2_apps/srv/go_to_loading.hpp"
#include <mutex>

std::mutex mu;

class ApproachServiceServer : public rclcpp::Node
{
public:
    ApproachServiceServer() : Node("approach_service_server_node")
    {
        tf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
        lift_cart_publisher_ = this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);

        // Callback group
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = callback_group_;

        callback_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options2;
        options2.callback_group = callback_group2_;

        // Service callback
        service_ = this->create_service<nav2_apps::srv::GoToLoading>( "/approach_shelf",
                                                               std::bind(&ApproachServiceServer::approachCallBack, this, std::placeholders::_1, std::placeholders::_2));

        // Create subscribers
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ApproachServiceServer::laserscanCallback, this, std::placeholders::_1), options1);           
        //odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&ApproachServiceServer::odomCallback, this, std::placeholders::_1), options1);

        timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&ApproachServiceServer::pubTF, this), callback_group_);
        timer_->cancel();

        timer2_ = create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&ApproachServiceServer::last_approach, this), callback_group2_);
        timer2_->cancel();
    }

private:
    rclcpp::Service<nav2_apps::srv::GoToLoading>::SharedPtr service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr callback_group2_;
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr lift_cart_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;

    std::string target_frame_ = "cart_frame";    
    std::string source_frame_ = "robot_front_laser_link";
    bool result_ = true;
    bool cart_frame_flag_ = true;
    bool to_position_flag_ = false;
    bool in_position_flag_ = false;

    void approachCallBack(const std::shared_ptr<nav2_apps::srv::GoToLoading::Request> request,
                          const std::shared_ptr<nav2_apps::srv::GoToLoading::Response> response)
    {
        if (request->attach_to_shelf)
        {
            timer_->reset();
                 
            while(cart_frame_flag_)
            {
                cart_approach();
                if (in_position_flag_) 
                {
                    break;
                }
            }

            // Move 30 cm more
            // Create and publish Twist message
            timer2_->reset();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "1");
            rclcpp::sleep_for(std::chrono::milliseconds(3000));
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "2");
            timer2_->cancel();
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.0;
            twist.angular.z = 0;
            cmd_vel_publisher_->publish(twist);

            // Load the shelf
            std_msgs::msg::Empty empty_message;
            lift_cart_publisher_->publish(empty_message);

            // Finish
            response->complete = result_;      
        }
        else
        {
            // Do nothing
            response->complete = false;
        }
        timer_->cancel();
    }

    void laserscanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        mu.lock();
        this->laser_msg_ = msg;
        mu.unlock();
    }

    void pubTF() 
    {
        std::vector<int> legs_position, left_leg, right_leg;
        double zero =  0.0; // front
        double izq_lim_ = -1.57; //90 degrees
        double der_lim_ = 1.57; //90 degrees

        mu.lock();  
        double zero_index_float = (zero - this->laser_msg_->angle_min) / this->laser_msg_->angle_increment;
        int zero_index = static_cast<int>(zero_index_float + 0.5); // Round to the nearest integer

        double izq_index_float = (izq_lim_ - this->laser_msg_->angle_min) / this->laser_msg_->angle_increment;
        int izq_index = static_cast<int>(izq_index_float + 0.5); // Round to the nearest integer

        double der_index_float = (der_lim_ - this->laser_msg_->angle_min) / this->laser_msg_->angle_increment;
        int der_index = static_cast<int>(der_index_float + 0.5); // Round to the nearest integer

        // Find the legs
        for (int i = 0; i < this->laser_msg_->intensities.size(); i++)
        {
            if (this->laser_msg_->intensities[i] >= 3000 && i >= izq_index && i <= der_index) 
            {
                legs_position.push_back(i);
            }
        }
        
        // Get the center point of each leg
        for (int i = 0; i < legs_position.size() - 1; i++)
        {
            int distance = legs_position[i+1] - legs_position[i];
            //RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Distance:%d  index:%d", distance, legs_position[i]);
            if ( distance < 2 && legs_position[i] < zero_index)
            {
                left_leg.push_back(legs_position[i]);
            }

            if (distance < 2 && legs_position[i] > zero_index)
            {
                right_leg.push_back(legs_position[i]);
            }
        }

        //RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "izq:%ld  der:%ld", left_leg.size(), right_leg.size());

        if (left_leg.empty() || right_leg.empty() && to_position_flag_)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "In position");
            in_position_flag_ = true;
            timer_->cancel();
            return;
        }

        if (left_leg.empty() || right_leg.empty())
        {
            // Only 1 leg detected
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "One leg detected");
            cart_frame_flag_ = false;
            timer_->cancel();
            result_ = false;
            return;
        }

        int center_left = left_leg[left_leg.size() / 2];    // middle point left leg
        int center_right = right_leg[right_leg.size() / 2]; // middle point right leg

        // Calculate center
        float alfa = this->laser_msg_->angle_min + this->laser_msg_->angle_increment*center_left;
        float beta = this->laser_msg_->angle_min + this->laser_msg_->angle_increment*center_right;
        float AN = std::abs(this->laser_msg_->ranges[center_left] * std::sin(alfa));
        float NB = std::abs(this->laser_msg_->ranges[center_right] * std::sin(beta));
        float AB = AN + NB;
        float AC = AB/2.0;
        float CB = AC;
        float Cx = this->laser_msg_->ranges[center_left] * std::cos(alfa);

        mu.unlock();

        float Cy, theta;

        if (AN < NB)
        {
            Cy = NB - CB;
            theta = std::atan2(Cy, Cx);
        }
        else if (AN > NB)
        {
            Cy = AC - AN;
            theta = std::atan2(Cy, Cx);
        }
        else // AN == NB middle point
        {
            Cy = 0;
            theta = 0;
        }
        
        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta);

        // Broadcast transform
        // Create a TransformStamped message
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = source_frame_;
        transform.child_frame_id = target_frame_;

        // Set the translation
        transform.transform.translation.x = Cx;
        transform.transform.translation.y = Cy;
        transform.transform.translation.z = 0.0;  

        // Set the rotation
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();

        // Publish the transform
        tf_broadcaster_->sendTransform(transform);
        to_position_flag_ = true;     
    }

    void cart_approach()
    {
      try
      {
        geometry_msgs::msg::TransformStamped transformStamped =
            tf_->lookupTransform(source_frame_, target_frame_, tf2::TimePointZero, tf2::durationFromSec(0.3));

        // Calculate distance and angular error
        float error_yaw = atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
        float error_distance = sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));

        // Define gains
        double kp_distance = 0.5;
        double kp_yaw = -0.5;

        // Calculate linear and angular velocities
        double linear_velocity = kp_distance * error_distance;
        double angular_velocity = kp_yaw * error_yaw;

        // Create and publish Twist message
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_velocity;
        twist.angular.z = angular_velocity;
        cmd_vel_publisher_->publish(twist);
      } 
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(get_logger(), "Transform exception: %s", ex.what());
      }    
    }

    void last_approach()
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "asdasd");
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.2;
        twist.angular.z = 0;
        cmd_vel_publisher_->publish(twist);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "asdasd");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ApproachServiceServer> node = std::make_shared<ApproachServiceServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}