#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav2_apps/srv/go_to_loading.hpp"
#include <cmath>
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
        lift_cart_publisher_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

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
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&ApproachServiceServer::odomCallback, this, std::placeholders::_1), options1);

        timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                std::bind(&ApproachServiceServer::pubTF, this), callback_group_);
        timer_->cancel();

        timer2_ = create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&ApproachServiceServer::pubCmd, this), callback_group2_);
        timer2_->cancel();

        // Initialize the last 2D position
        last_position_.x = 0.0;
        last_position_.y = 0.0;
        distance_moved_ = 0.0;

        twist_.linear.x = 0.0;
        twist_.angular.z = 0;
    }

private:
    rclcpp::Service<nav2_apps::srv::GoToLoading>::SharedPtr service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr callback_group2_;
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lift_cart_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;

    geometry_msgs::msg::Twist twist_;

    geometry_msgs::msg::Point last_position_;
    double distance_moved_;

    std::string target_frame_ = "cart_frame";    
    std::string source_frame_ = "robot_front_laser_link";
    bool result_ = true;
    bool cart_frame_flag_ = true;
    bool first_look_flag_ = true;
    bool in_position_flag_ = false;

    float Cx_;

    void approachCallBack(const std::shared_ptr<nav2_apps::srv::GoToLoading::Request> request,
                          const std::shared_ptr<nav2_apps::srv::GoToLoading::Response> response)
    {
        // Restart value
        result_ = true;
        cart_frame_flag_ = true;
        first_look_flag_ = true;
        in_position_flag_ = false;
        Cx_ = -1;
        timer2_->reset();

        if (request->attach_to_shelf)
        {
            timer_->reset();   
            while(cart_frame_flag_)
            {
                cart_approach();
            }

            // Move 30 cm more
            double current_distance = distance_moved_;
            
            while (distance_moved_ - current_distance <= 0.6)
            {
                twist_.linear.x = 0.1;
                twist_.angular.z = 0;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "approaching");
            }
            twist_.linear.x = 0.0;
            twist_.angular.z = 0;

            // Load the shelf
            std_msgs::msg::String empty_message;
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
        timer2_->cancel();     
    }

    void laserscanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        std::lock_guard<std::mutex> lockGuard(mu);
        this->laser_msg_ = msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        // Get the current 2D position from the odometry message
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        if (last_position_.x != 0.0 && last_position_.y != 0.0) 
        {
            // Calculate distance moved using Euclidean distance formula
            double distance_x = current_x - last_position_.x;
            double distance_y = current_y - last_position_.y;
            double distance = sqrt(distance_x * distance_x + distance_y * distance_y);

            // Update the total distance moved
            distance_moved_ += distance;
        }

        // Update the last 2D position
        last_position_.x = current_x;
        last_position_.y = current_y;
    }

    void pubTF() 
    {
        std::vector<int> legs_position, left_leg, right_leg;
        double zero =  0.0; // front
        double izq_lim_ = -1.57; //90 degrees
        double der_lim_ = 1.57; //90 degrees

        std::lock_guard<std::mutex> lockGuard(mu);

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

        if ((left_leg.empty() || right_leg.empty()) && first_look_flag_) 
        {
            // Only 1 leg detected
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "One leg detected");
            cart_frame_flag_ = false;
            timer_->cancel();
            result_ = false;
            return;
        }

        if (left_leg.empty() || right_leg.empty()) 
        {
            // Too close to detect legs
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Too close to detect legs");
            cart_frame_flag_ = false;
            timer_->cancel();
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

        Cx_ = this->laser_msg_->ranges[center_left] * std::cos(alfa);

        float Cy, theta;

        if (AN < NB)
        {
            Cy = NB - CB;
            theta = std::atan2(Cy, Cx_);
        }
        else if (AN > NB)
        {
            Cy = AC - AN;
            theta = std::atan2(Cy, Cx_);
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
        transform.transform.translation.x = Cx_;
        transform.transform.translation.y = Cy;
        transform.transform.translation.z = 0.0;  

        // Set the rotation
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();

        // Publish the transform
        tf_broadcaster_->sendTransform(transform);
        first_look_flag_ = false;   
    }

    void cart_approach()
    {
      // Define gains
      double kp_distance = 0.5;
      double kp_yaw = -0.5;
       
      try
      {
        geometry_msgs::msg::TransformStamped transformStamped =
            tf_->lookupTransform(source_frame_, target_frame_, tf2::TimePointZero);

        // Calculate distance and angular error
        float error_yaw = atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
        float error_distance = sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));

        std::lock_guard<std::mutex> lockGuard(mu);
        float center_error = Cx_;
        
        //RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "izq:%f der:%f  asd:%f", error_yaw, error_distance, Cx_);

        if (error_distance <= 0.101 && error_yaw <= 0.018)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "In position");
            cart_frame_flag_ = false;
            timer_->cancel();
            // Stop the robot
            twist_.linear.x = 0.0;
            twist_.angular.z = 0.0;
            return;
        }

        // If robot is close to the middle point, align
        if (std::abs(center_error) <= 0.02 && error_distance <= 0.10)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Aligning");
            kp_distance = 0.0;
        }
        
        // Calculate linear and angular velocities
        double linear_velocity = kp_distance * error_distance;
        double angular_velocity = kp_yaw * error_yaw;

        // Limits
        if (linear_velocity > 0.2) linear_velocity = 0.2;
        if (angular_velocity > 0.1) linear_velocity = 0.1;

        if (linear_velocity < -0.2) linear_velocity = -0.2;
        if (angular_velocity < -0.1) linear_velocity = -0.1;

        // Create and publish Twist message
        twist_.linear.x = linear_velocity;
        twist_.angular.z = angular_velocity;
        
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } 
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(get_logger(), "Transform exception: %s", ex.what());
      }    
    }

    void pubCmd()
    {
        cmd_vel_publisher_->publish(twist_);
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