#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "kinova_control/srv/key_trigger.hpp"

using namespace std::chrono_literals;

class MoveItService : public rclcpp::Node {
public:
    MoveItService() 
        : Node("moveit_service"), 
          move_group_interface_(
              std::shared_ptr<rclcpp::Node>(this, [](auto*) {}),
              "manipulator"
          ) 
    {
        service_ = create_service<kinova_control::srv::KeyTrigger>(
            "key_trigger_service",
            [this](
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<kinova_control::srv::KeyTrigger::Request> req,
                std::shared_ptr<kinova_control::srv::KeyTrigger::Response> res
            ) {
                (void)request_header;
                handle_key(req, res);
            });

        move_group_interface_.setPlanningTime(15.0);
        move_group_interface_.setMaxVelocityScalingFactor(0.5);
        move_group_interface_.setMaxAccelerationScalingFactor(0.5);
        RCLCPP_INFO(get_logger(), "Service ready. Always returns home before other poses!");
    }

private:
    rclcpp::Service<kinova_control::srv::KeyTrigger>::SharedPtr service_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;

    void handle_key(
        const std::shared_ptr<kinova_control::srv::KeyTrigger::Request> req,
        std::shared_ptr<kinova_control::srv::KeyTrigger::Response> res
    ) {
        res->success = false;
        
        if (req->key_pressed == "e") {  // Direct home command
            RCLCPP_INFO(get_logger(), "Executing direct home position");
            res->success = return_to_home();
        }
        else {
            // First return to home for all other commands
            RCLCPP_INFO(get_logger(), "Returning to home before executing %s pose", req->key_pressed.c_str());
            bool home_success = return_to_home();
            
            if(home_success) {
                if (req->key_pressed == "t") {   // Pose 1
                    res->success = execute_pose("pose1", return_to_pose1());
                }
                else if (req->key_pressed == "f") {   // Pose 2
                    res->success = execute_pose("pose2", return_to_pose2());
                }
                else if (req->key_pressed == "h") {   // Pose 3
                    res->success = execute_pose("pose3", return_to_pose3());
                }
                else if (req->key_pressed == "g") {   // Pose 4
                    res->success = execute_pose("pose4", return_to_pose4());
                }
                else if (req->key_pressed == "v") {   // Pose 5
                    res->success = execute_pose("pose5", return_to_pose5());
                }
                else {
                    RCLCPP_WARN(get_logger(), "Unknown key: %s", req->key_pressed.c_str());
                }
            }
            else {
                RCLCPP_ERROR(get_logger(), "Home position failed! Aborting %s pose", req->key_pressed.c_str());
            }
        }
    }

    bool execute_pose(const std::string& pose_name, bool pose_result) {
        if(pose_result) {
            RCLCPP_INFO(get_logger(), "%s executed successfully", pose_name.c_str());
            return true;
        }
        RCLCPP_ERROR(get_logger(), "%s execution failed", pose_name.c_str());
        return false;
    }

    // Keep existing return_to_home and return_to_poseX functions
    bool return_to_home() {
        std::vector<double> joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return execute_joint_motion(joints, "home");
    }

    bool return_to_pose1() {
        std::vector<double> joints = {
            -105.0 * (M_PI / 180.0), 
            -63.0 * (M_PI / 180.0), 
            143.0 * (M_PI / 180.0),
            72.0 * (M_PI / 180.0), 
            -135.0 * (M_PI / 180.0), 
            -55.0 * (M_PI / 180.0),
            29.0 * (M_PI / 180.0)
        };
        return execute_joint_motion(joints, "pose1");
    }

    bool return_to_pose2() {
        std::vector<double> joints = {
            -164.0 * (M_PI / 180.0), 
            91.0 * (M_PI / 180.0), 
            64.0 * (M_PI / 180.0),
            -120.0 * (M_PI / 180.0), 
            -99.0 * (M_PI / 180.0), 
            108.0 * (M_PI / 180.0),
            -113.0 * (M_PI / 180.0)
        };
        return execute_joint_motion(joints, "pose2");
    }

    bool return_to_pose3() {
        std::vector<double> joints = {
            130.0 * (M_PI / 180.0), 
            55.0 * (M_PI / 180.0), 
            152.0 * (M_PI / 180.0),
            -59.0 * (M_PI / 180.0), 
            28.0 * (M_PI / 180.0), 
            -64.0 * (M_PI / 180.0),
            103.0 * (M_PI / 180.0)
        };
        return execute_joint_motion(joints, "pose3");
    }

    bool return_to_pose4() {
        std::vector<double> joints = {
            5.0 * (M_PI / 180.0), 
            28.0 * (M_PI / 180.0), 
            -49.0 * (M_PI / 180.0),
            -40.0 * (M_PI / 180.0), 
            -51.0 * (M_PI / 180.0), 
            -120.0 * (M_PI / 180.0),
            94.0 * (M_PI / 180.0)
        };
        return execute_joint_motion(joints, "pose4");
    }

    bool return_to_pose5() {
        std::vector<double> joints = {
            104.0 * (M_PI / 180.0), 
            -65.0 * (M_PI / 180.0), 
            -67.0 * (M_PI / 180.0),
            -39.0 * (M_PI / 180.0), 
            -124.0 * (M_PI / 180.0), 
            -80.0 * (M_PI / 180.0),
            102.0 * (M_PI / 180.0)
        };
        return execute_joint_motion(joints, "pose5");
    }

    bool execute_joint_motion(const std::vector<double>& joints, const std::string& pose_name) {
        move_group_interface_.setStartStateToCurrentState();
        move_group_interface_.setJointValueTarget(joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        RCLCPP_INFO(get_logger(), "Planning %s...", pose_name.c_str());
        bool success = (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if(success) {
            RCLCPP_INFO(get_logger(), "Executing %s...", pose_name.c_str());
            success = (move_group_interface_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        
        if(!success) {
            RCLCPP_ERROR(get_logger(), "%s failed!", pose_name.c_str());
        }
        return success;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}