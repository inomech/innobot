#define BOOST_BIND_NO_PLACEHOLDERS

#include <inttypes.h>
#include <functional>
#include <memory>
#include <thread>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "control_msgs/action/gripper_command.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "innobot_msgs/action/move.hpp"
#include "innobot_msgs/action/calibrate.hpp"

namespace innobot_calibrate{
class CalibrationActionServer : public rclcpp::Node
{
public:

  using Calibrate = innobot_msgs::action::Calibrate; 
  using GoalHandleCalibrate = rclcpp_action::ServerGoalHandle<Calibrate>;

  CalibrationActionServer() : Node("arm_calibration_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)), goal_done_(false)
  {

    using namespace std::placeholders; 

    if (! this->has_parameter("base_frame"))
    {
      this->declare_parameter("base_frame", "world");
    }

    this->arm_move_server_ = rclcpp_action::create_server<Calibrate>(
      this, 
      "calibration_as",
      std::bind(&CalibrationActionServer::handle_goal, this, _1, _2),
      std::bind(&CalibrationActionServer::handle_cancel, this, _1),
      std::bind(&CalibrationActionServer::handle_accepted, this, _1)
    );

    WORLD_FRAME = "world";
    TCP_FRAME = "tcp";
    PLANNING_GROUP = "arm";


  } 

private:
  // Planning components
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters_;
  bool goal_done_;
  std::string WORLD_FRAME;
  std::string TCP_FRAME;
  std::string PLANNING_GROUP;


  rclcpp_action::Server<Calibrate>::SharedPtr arm_move_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Calibrate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received calibration goal" );
    (void)uuid;
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCalibrate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCalibrate> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&CalibrationActionServer::execute_move, this, _1), goal_handle}.detach();
  }

  tf2::Transform poseMsgToTransform(const geometry_msgs::msg::Pose& pose_in){
    
    tf2::Transform t_out; 
    tf2::Vector3 pos(pose_in.position.x, pose_in.position.y, pose_in.position.z); 
    t_out.setOrigin(pos);  
    tf2::Quaternion q;
    tf2::fromMsg(pose_in.orientation, q);
    t_out.setRotation(q);

    return t_out; 
  }
 
  
  void doPoses(geometry_msgs::msg::Pose& pose){

    bool success = false;
    
    moveit::core::RobotStatePtr robot_state = nullptr; 

    robot_state = moveit_cpp_->getCurrentState(2.0);

    if(!robot_state){
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get robot current state");
    }

    moveit_msgs::msg::RobotState robot_state_msg; 
    moveit::core::robotStateToRobotStateMsg(*robot_state, robot_state_msg, true);

    // start doMotionPlanning

    std::vector<double> position_tolerances(3, 0.01f);
    std::vector<double> orientation_tolerances(3, 0.01f);

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = WORLD_FRAME;
    goal_pose.pose = pose;

    moveit_msgs::msg::Constraints robot_goal = kinematic_constraints::constructGoalConstraints(
        TCP_FRAME, goal_pose, position_tolerances, orientation_tolerances);

    if (!moveit_cpp_->getPlanningSceneMonitor()->requestPlanningSceneState())
    {
      throw std::runtime_error("Failed to get planning scene");
    }

    // setting up planning configuration
    moveit_cpp::PlanningComponent planning_component(PLANNING_GROUP, moveit_cpp_);
    moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters;
    
    plan_parameters.load(this->shared_from_this());
    plan_parameters.planner_id = "RRTConnect";
    plan_parameters.planning_time = 20.0;
    plan_parameters.planning_attempts = 4;

    // set planning goal
    moveit::core::RobotState start_robot_state(moveit_cpp_->getRobotModel());
    moveit::core::robotStateMsgToRobotState(robot_state_msg, start_robot_state);

    planning_component_->setStartStateToCurrentState();
    planning_component_->setGoal(goal_pose, TCP_FRAME);

    // planning for goal
    auto plan_solution = planning_component_->plan();

    if (!plan_solution)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to plan");
    }

    success = moveit_cpp_->execute(PLANNING_GROUP, plan_solution.trajectory, true);

    if(success){

      RCLCPP_INFO(this->get_logger(), "Goal succeeded, logging robot state");  

      moveit::core::RobotStatePtr current_robot_state = nullptr;

      current_robot_state = moveit_cpp_->getCurrentState(2.0);

      moveit_msgs::msg::RobotState current_robot_state_msg; 
      moveit::core::robotStateToRobotStateMsg(*current_robot_state, current_robot_state_msg, true);

      for(unsigned int i=0; i < 6; i++){
        RCLCPP_INFO(this->get_logger(), "Joint state %i is: %s",i,std::to_string(current_robot_state_msg.joint_state.position[i]).c_str());    
      }

    }


  }

  void execute_move(const std::shared_ptr<GoalHandleCalibrate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing move goal");  
    
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Calibrate::Result>();

    RCLCPP_INFO(this->get_logger(), "Generating Moveit");  

    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this());

    // Planning component associated with a single motion group
    planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>("arm", moveit_cpp_);

    // Parameters set on this node
    plan_parameters_.load(this->shared_from_this());

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Move goal canceled");
      return;
    }

    std::vector<geometry_msgs::msg::Pose> cal_poses, tcp_place_poses, wrist_place_poses, poses;
    poses.clear();
    cal_poses.clear();


    geometry_msgs::msg::Pose desired_pose, temp_pose;
   
    temp_pose = goal->cal_pose;

    tf2::Transform tmp_trans = poseMsgToTransform(temp_pose);  


    tf2::Transform ee_at_cal_tf;

    tf2::Vector3 cal_position(temp_pose.position.x, temp_pose.position.y, temp_pose.position.z); 
    ee_at_cal_tf.setOrigin(cal_position);
    ee_at_cal_tf.setRotation(tmp_trans.getRotation() * tf2::Quaternion(tf2::Vector3(1,0,0), M_PI));

    tf2::toMsg(ee_at_cal_tf, desired_pose);


    doPoses(desired_pose);
    
    rclcpp::sleep_for(std::chrono::seconds(1));


    RCLCPP_INFO(this->get_logger(), "Setting result to true");

    result->success = true;

    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Move goal succeeded");
    }
    
  }
  

}; // class CalibrationActionServer

} // namespace innobot_calibrate

int main(int argc, char **argv)
{
  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // Create the CalibrationActionServer node
  auto arm_action_server = std::make_shared<innobot_calibrate::CalibrationActionServer>();

  rclcpp::spin(arm_action_server);

  rclcpp::shutdown();

  return 0;
}
