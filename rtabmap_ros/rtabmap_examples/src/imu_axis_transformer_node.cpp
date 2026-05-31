/**
 * IMU Axis Transformer Node (C++)
 * 
 * High-performance C++ node for transforming OAK-D IMU data from sensor frame
 * to ROS REP-103 base frame convention (X=forward, Y=left, Z=up).
 * 
 * This node provides the same functionality as the Python imu_axis_transformer
 * but with deterministic C++ performance for handling 400-500 Hz IMU data.
 * 
 * Note we still get much quicker if this fix is implemented
 * in stereo_inertial_publisher_transformed.cpp, but we use this node because
 * Difference in performance wasnt that big + its more clean to do it this way
 * Transformation (corrected Z-axis):
 * - ros_x = sensor_z   (forward)
 * - ros_y = -sensor_x  (left)
 * - ros_z = sensor_y   (up, POSITIVE not negative)
 * 
 * Rotation matrix R:
 * | 0   0   1 |
 * |-1   0   0 |
 * | 0   1   0 |
 * 
 * Author: Atlas AI Agent
 * Date: February 12, 2026
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <array>
#include <functional>

class ImuAxisTransformerNode : public rclcpp::Node
{
public:
    ImuAxisTransformerNode() 
    : Node("imu_axis_transformer_cpp")
    {
        // Declare parameters
        this->declare_parameter<bool>("enable_transform", true);
        this->declare_parameter<std::string>("output_frame", "oak-d-base-frame");
        this->declare_parameter<int>("qos_depth", 200);
        
        // Get parameters
        enable_transform_ = this->get_parameter("enable_transform").as_bool();
        output_frame_ = this->get_parameter("output_frame").as_string();
        int qos_depth = this->get_parameter("qos_depth").as_int();
        
        // Configure QoS for high-throughput IMU data (400-500 Hz)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth))
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        
        // Create subscriber
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos,
            std::bind(&ImuAxisTransformerNode::imuCallback, this, std::placeholders::_1));
        
        // Create publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/transformed", qos);
        
        if (enable_transform_) {
            RCLCPP_INFO(this->get_logger(), 
                "C++ IMU Axis Transformer started - Transformation ENABLED");
            RCLCPP_INFO(this->get_logger(), 
                "Output frame: %s, QoS depth: %d", output_frame_.c_str(), qos_depth);
            RCLCPP_INFO(this->get_logger(), 
                "Optimized C++ for 400-500 Hz IMU (corrected Z-axis)");
        } else {
            RCLCPP_INFO(this->get_logger(), 
                "C++ IMU Axis Transformer started - PASS-THROUGH mode");
        }
    }

private:
    void transformVector(const double x, const double y, const double z,
                        double& out_x, double& out_y, double& out_z) const
    {
        // Rotation matrix: [[0,0,1],[-1,0,0],[0,1,0]]
        // ros_x = sensor_z
        // ros_y = -sensor_x
        // ros_z = sensor_y  (CORRECTED: positive, not negative)
        out_x = z;
        out_y = -x;
        out_z = y;  // Changed from -y to +y to fix Z-axis inversion
    }
    
    void transformCovariance(const std::array<double, 9>& cov_in,
                           std::array<double, 9>& cov_out) const
    {
        // Check for all-zero covariance (unknown in ROS convention)
        if (cov_in[0] == 0.0 && cov_in[4] == 0.0 && cov_in[8] == 0.0) {
            cov_out.fill(0.0);
            return;
        }
        
        // Apply R × C × R^T using pre-computed index mapping
        // For R = [[0,0,1],[-1,0,0],[0,1,0]]:
        // C_out[0,0]=C[2,2], C_out[0,1]=-C[2,0], C_out[0,2]=C[2,1]
        // C_out[1,0]=-C[0,2], C_out[1,1]=C[0,0], C_out[1,2]=-C[0,1]
        // C_out[2,0]=C[1,2], C_out[2,1]=-C[1,0], C_out[2,2]=C[1,1]
        cov_out[0] =  cov_in[8]; cov_out[1] = -cov_in[6]; cov_out[2] =  cov_in[7];  // row 0
        cov_out[3] = -cov_in[2]; cov_out[4] =  cov_in[0]; cov_out[5] = -cov_in[1];  // row 1
        cov_out[6] =  cov_in[5]; cov_out[7] = -cov_in[3]; cov_out[8] =  cov_in[4];  // row 2
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto out_msg = sensor_msgs::msg::Imu();
        
        if (enable_transform_) {
            // Copy and update header
            out_msg.header = msg->header;
            out_msg.header.frame_id = output_frame_;
            
            // Transform linear acceleration
            transformVector(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                out_msg.linear_acceleration.x,
                out_msg.linear_acceleration.y,
                out_msg.linear_acceleration.z
            );
            
            // Transform angular velocity
            transformVector(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z,
                out_msg.angular_velocity.x,
                out_msg.angular_velocity.y,
                out_msg.angular_velocity.z
            );
            
            // Transform covariance matrices
            std::array<double, 9> accel_cov_in, accel_cov_out;
            std::array<double, 9> gyro_cov_in, gyro_cov_out;
            
            std::copy(msg->linear_acceleration_covariance.begin(),
                     msg->linear_acceleration_covariance.end(),
                     accel_cov_in.begin());
            std::copy(msg->angular_velocity_covariance.begin(),
                     msg->angular_velocity_covariance.end(),
                     gyro_cov_in.begin());
            
            transformCovariance(accel_cov_in, accel_cov_out);
            transformCovariance(gyro_cov_in, gyro_cov_out);
            
            std::copy(accel_cov_out.begin(), accel_cov_out.end(),
                     out_msg.linear_acceleration_covariance.begin());
            std::copy(gyro_cov_out.begin(), gyro_cov_out.end(),
                     out_msg.angular_velocity_covariance.begin());
            
            // Copy orientation unchanged (computed downstream by Madgwick)
            out_msg.orientation = msg->orientation;
            out_msg.orientation_covariance = msg->orientation_covariance;
        } else {
            // Pass through unchanged
            out_msg = *msg;
        }
        
        // Publish transformed message
        imu_pub_->publish(out_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    bool enable_transform_;
    std::string output_frame_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ImuAxisTransformerNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
