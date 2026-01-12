#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <evo_be_Common.h>
#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using namespace evo_be;

class BionicControlNode : public rclcpp::Node
{
public:
    BionicControlNode()
        : Node("bionic_ctrl_node"), device_(nullptr)
    {
        // Initialize BionicEyes device
        initDevice();

        // Create subscribers for eye and neck target angles
        left_eye_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "left_eye_target_angles", 10,
            std::bind(&BionicControlNode::leftEyeTargetCallback, this, std::placeholders::_1));

        right_eye_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "right_eye_target_angles", 10,
            std::bind(&BionicControlNode::rightEyeTargetCallback, this, std::placeholders::_1));

        neck_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "neck_target_angles", 10,
            std::bind(&BionicControlNode::neckTargetCallback, this, std::placeholders::_1));

        // Create publishers for current eye and neck angles
        left_eye_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "left_eye_current_angles", 10);

        right_eye_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "right_eye_current_angles", 10);

        neck_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "neck_current_angles", 10);

        // Timer for publishing current positions
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&BionicControlNode::publishCurrentPositions, this));

        RCLCPP_INFO(this->get_logger(), "Bionic Control Node initialized");
    }

    ~BionicControlNode()
    {
        if (device_)
        {
            // Return to initial position before shutdown
            if (device_->haveNeckLinked())
            {
                device_->goInitPosition_Neck(enumNeckAllMotor);
            }
            device_->goInitPosition(enumAllMotor);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            delete device_;
        }
    }

private:
    void initDevice()
    {
        try
        {
            // Create device in control mode (local connection)
            device_ = CBionicEyes::create();

            if (!device_)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to create BionicEyes device");
                return;
            }

            // Disable VOR and SV for manual control
            device_->onoff_VOR(false, false);
            device_->onoff_SV(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // Get initial positions
            device_->getInitPosition(enumAllMotor, left_eye_init_pos_);
            device_->getInitPosition(enumAllMotor, right_eye_init_pos_);

            // Get neck initial position if available
            if (device_->haveNeckLinked())
            {
                device_->getInitPosition_Neck(enumNeckAllMotor, neck_init_pos_);
                RCLCPP_INFO(this->get_logger(), "Neck detected and initialized");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No neck linked to device");
            }

            // Move to initial position
            device_->goInitPosition(enumAllMotor);
            if (device_->haveNeckLinked())
            {
                device_->goInitPosition_Neck(enumNeckAllMotor);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            RCLCPP_INFO(this->get_logger(), "BionicEyes device initialized successfully");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception during device initialization: %s", e.what());
        }
    }

    void leftEyeTargetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!device_ || msg->data.size() != 3)
        {
            return;
        }

        try
        {
            // Left eye motors: 0 (pitch), 1 (roll), 2 (yaw)
            bool motorWorkFlag[6] = {true, true, true, false, false, false};
            float motor_value[6] = {
                msg->data[0],  // pitch
                msg->data[1],  // roll
                msg->data[2],  // yaw
                0.0f, 0.0f, 0.0f
            };

            device_->setAbsolutePosition(motorWorkFlag, motor_value);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            RCLCPP_DEBUG(this->get_logger(), "Left eye target: pitch=%.2f, roll=%.2f, yaw=%.2f",
                       msg->data[0], msg->data[1], msg->data[2]);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting left eye position: %s", e.what());
        }
    }

    void rightEyeTargetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!device_ || msg->data.size() != 3)
        {
            return;
        }

        try
        {
            // Right eye motors: 3 (pitch), 4 (roll), 5 (yaw)
            bool motorWorkFlag[6] = {false, false, false, true, true, true};
            float motor_value[6] = {
                0.0f, 0.0f, 0.0f,
                msg->data[0],  // pitch
                msg->data[1],  // roll
                msg->data[2]   // yaw
            };

            device_->setAbsolutePosition(motorWorkFlag, motor_value);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            RCLCPP_DEBUG(this->get_logger(), "Right eye target: pitch=%.2f, roll=%.2f, yaw=%.2f",
                       msg->data[0], msg->data[1], msg->data[2]);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting right eye position: %s", e.what());
        }
    }

    void neckTargetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!device_ || !device_->haveNeckLinked() || msg->data.size() != 3)
        {
            return;
        }

        try
        {
            // Neck angles: pan, tilt, roll
            bool activeFlag[3] = {true, true, true};
            float angles[3] = {
                msg->data[0],  // pan
                msg->data[1],  // tilt
                msg->data[2]   // roll
            };

            device_->setAbsolutePosition_Neck(activeFlag, angles, enumMovePattern_Saccade);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            RCLCPP_DEBUG(this->get_logger(), "Neck target: pan=%.2f, tilt=%.2f, roll=%.2f",
                       msg->data[0], msg->data[1], msg->data[2]);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting neck position: %s", e.what());
        }
    }

    void publishCurrentPositions()
    {
        if (!device_)
        {
            return;
        }

        try
        {
            // Get current motor positions
            BE_GeneralData data;
            if (device_->isBeDataReady())
            {
                device_->getBeData(data);
            }

            // Publish left eye current angles (motors 0-2)
            auto left_eye_msg = std_msgs::msg::Float32MultiArray();
            left_eye_msg.data = {data.motorData[0], data.motorData[1], data.motorData[2]};
            left_eye_pub_->publish(left_eye_msg);

            // Publish right eye current angles (motors 3-5)
            auto right_eye_msg = std_msgs::msg::Float32MultiArray();
            right_eye_msg.data = {data.motorData[3], data.motorData[4], data.motorData[5]};
            right_eye_pub_->publish(right_eye_msg);

            // Publish neck current angles if available
            if (device_->haveNeckLinked())
            {
                float neck_pos[3];
                device_->getInitPosition_Neck(enumNeckAllMotor, neck_pos);
                auto neck_msg = std_msgs::msg::Float32MultiArray();
                neck_msg.data = {neck_pos[0], neck_pos[1], neck_pos[2]};
                neck_pub_->publish(neck_msg);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error publishing current positions: %s", e.what());
        }
    }

    // BionicEyes device pointer
    CBionicEyes *device_;

    // Initial positions
    float left_eye_init_pos_[6];
    float right_eye_init_pos_[6];
    float neck_init_pos_[3];

    // ROS2 subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr left_eye_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr right_eye_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr neck_sub_;

    // ROS2 publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr left_eye_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr right_eye_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr neck_pub_;

    // Timer for publishing current positions
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BionicControlNode>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception in main loop: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
