#define BOOST_BIND_NO_PLACEHOLDERS

#include <math.h>
#include <inttypes.h>
#include <functional>
#include <memory>
#include <thread>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "control_msgs/action/gripper_command.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
#include "innobot_msgs/action/pnp.hpp"

#define ROUNDING_PRECISION 1

namespace innobot_core{

class PickNPlace : public rclcpp::Node
{
public:

  using Pnp = innobot_msgs::action::Pnp; 
  using GoalHandlePnp = rclcpp_action::ServerGoalHandle<Pnp>;

  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

  PickNPlace() : Node("pick_n_place", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)), goal_done_(false)
  {

    using namespace std::placeholders; 

    if (! this->has_parameter("base_frame"))
    {
      this->declare_parameter("base_frame", "world");
    }

    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&PickNPlace::topic_callback, this, _1)
    );

    this->arm_move_server_ = rclcpp_action::create_server<Pnp>(
      this, 
      "pick_n_place_as",
      std::bind(&PickNPlace::handle_goal, this, _1, _2),
      std::bind(&PickNPlace::handle_cancel, this, _1),
      std::bind(&PickNPlace::handle_accepted, this, _1)
    );

    this->gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "gripper_controller/gripper_action");

  }

private:
  // Planning components
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  bool goal_done_;
  std::vector<double> joint_states_; 
  float gripper_position_; 

  rclcpp_action::Server<Pnp>::SharedPtr arm_move_server_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {

    // This relies on the fact that all the joints get loaded in this order - 1,3,5,2,4,6,right_finger    
    // TODO - make this more robust (e.g., ordering joint_states_ by reading the order of names published first)
    joint_states_ = {
      round( msg->position[0] * (180/M_PI) * ROUNDING_PRECISION ) / ROUNDING_PRECISION, 
      round( msg->position[3] * (180/M_PI) * ROUNDING_PRECISION ) / ROUNDING_PRECISION, 
      round( msg->position[1] * (180/M_PI) * ROUNDING_PRECISION ) / ROUNDING_PRECISION, 
      round( msg->position[4] * (180/M_PI) * ROUNDING_PRECISION ) / ROUNDING_PRECISION, 
      round( msg->position[2] * (180/M_PI) * ROUNDING_PRECISION ) / ROUNDING_PRECISION, 
      round( msg->position[5] * (180/M_PI) * ROUNDING_PRECISION ) / ROUNDING_PRECISION};  

  }

  void send_goal(float gripper_position)
  {
    using namespace std::placeholders;

    this->goal_done_ = false;

    if (!this->gripper_client_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->gripper_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = GripperCommand::Goal();

    goal_msg.command.position = gripper_position;
    goal_msg.command.max_effort = 0.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&PickNPlace::goal_response_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&PickNPlace::result_callback, this, _1);
    auto goal_handle_future = this->gripper_client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Pnp::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with goal_state: ", goal->pick_pose);
    (void)uuid;
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePnp> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePnp> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PickNPlace::execute_move, this, _1), goal_handle}.detach();
  }

  void verifyPose(moveit::core::RobotStatePtr desired_robot_state, bool& pose_success){

    std::vector<double> joint_values;

    desired_robot_state->copyJointGroupPositions("arm", joint_values);

    for(std::size_t i = 0; i < joint_values.size(); i++){

      joint_values[i] = round( joint_values[i] * (180/M_PI) * ROUNDING_PRECISION ) / ROUNDING_PRECISION;

      // RCLCPP_INFO(this->get_logger(), "%i angle: %f, %f", i, joint_values[i], joint_states_[i]);

    }

    if(joint_states_ == joint_values){

      pose_success = true;
      RCLCPP_INFO(this->get_logger(), "Robot in its position");

    }

    if(joint_states_ != joint_values){

      pose_success = false; 
      // RCLCPP_INFO(this->get_logger(), "Robot not yet in its position");

    }

  }

  // functions for pick and place application 

  tf2::Transform poseMsgToTransform(const geometry_msgs::msg::Pose& pose_in){
    
    // transform pose message from geometry_msgs::msg::Pose to tf2::Transform
    
    tf2::Transform t_out; 
    tf2::Vector3 pos(pose_in.position.x, pose_in.position.y, pose_in.position.z); 
    t_out.setOrigin(pos);  
    tf2::Quaternion q;
    tf2::fromMsg(pose_in.orientation, q);
    t_out.setRotation(q);

    return t_out; 
  }
 
  std::vector<geometry_msgs::msg::Pose> createManipulationPoses(double retreat_dis, double approach_dis, const tf2::Transform target_tf){
  
    // for given pick or place pose create the three manipulation poses, that is the start, approach, retreat where after approach follows grasp
  
    geometry_msgs::msg::Pose start_pose, target_pose, end_pose; 
    std::vector<geometry_msgs::msg::Pose> poses; 

    tf2::toMsg(tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, approach_dis)) * target_tf, start_pose);

    tf2::toMsg(target_tf, target_pose);

    tf2::toMsg(tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, retreat_dis)) * target_tf, end_pose);

    poses.clear();
    poses.push_back(start_pose);
    poses.push_back(target_pose);
    poses.push_back(end_pose);

    return poses;
  }

  void doPoses(std::vector<geometry_msgs::msg::Pose>& poses){

    bool success = false;

    for(unsigned int i = 0; i < poses.size(); i++)
    {
    
      moveit::core::RobotStatePtr robot_state = nullptr; 

      robot_state = moveit_cpp_->getCurrentState();

      if(!robot_state){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get robot current state");
      }

      moveit_msgs::msg::RobotState robot_state_msg; 
      moveit::core::robotStateToRobotStateMsg(*robot_state, robot_state_msg, true);

      // moveit_cpp::PlanningComponent::PlanSolution plan_solution; 

      // start of doMotionPlanning

      std::vector<double> position_tolerances(3, 0.01f);
      std::vector<double> orientation_tolerances(3, 0.01f);

      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "world";
      goal_pose.pose = poses[i];

      moveit_msgs::msg::Constraints robot_goal = kinematic_constraints::constructGoalConstraints(
          "tcp", goal_pose, position_tolerances, orientation_tolerances);

      if (!moveit_cpp_->getPlanningSceneMonitor()->requestPlanningSceneState())
      {
        throw std::runtime_error("Failed to get planning scene");
      }

      auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>("arm", moveit_cpp_);

      moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters;
      
      plan_parameters.load(this->shared_from_this());
      plan_parameters.planner_id = "RRTConnect";
      plan_parameters.planning_pipeline = "ompl";
      plan_parameters.planning_time = 20.0;
      plan_parameters.planning_attempts = 1;
      plan_parameters.max_velocity_scaling_factor = 0.75;
      plan_parameters.max_acceleration_scaling_factor = 0.75;

      // set planning goal
      moveit::core::RobotState start_robot_state(moveit_cpp_->getRobotModel());
      moveit::core::robotStateMsgToRobotState(robot_state_msg, start_robot_state);
      planning_components->setStartState(start_robot_state);
      planning_components->setGoal({ robot_goal });

      // planning for goal
      auto plan_solution = planning_components->plan(plan_parameters);

      if (!plan_solution)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to plan");
      }

      moveit::core::RobotStatePtr desired_robot_state = plan_solution.trajectory->getLastWayPointPtr(); 

      // Executes the latest planned trajectory 
      // planning_components->execute(false);

      // Executes the given trajectory 
      success = moveit_cpp_->execute("arm", plan_solution.trajectory, false);

      bool pose_success = false; 

      
      while(!pose_success){

        verifyPose(desired_robot_state, pose_success);

      }

      if(i == 1){

        send_goal(gripper_position_);

      }else if(i == 4){

        send_goal(0.08);

      }

      

      // end of doMotionPlanning
    }
  }

  void execute_move(const std::shared_ptr<GoalHandlePnp> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing move goal");  
    
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Pnp::Result>();

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

    double retreat_dis = 0.05; 
    double approach_dis = 0.05; 

    // start computePickToolPoses 

    geometry_msgs::msg::Pose box_pose, box_pose_place; 

    box_pose.position = goal->pick_pose.position; 
    box_pose.orientation = goal->pick_pose.orientation;

    gripper_position_ = goal->object_width; 

    tf2::Transform tcp_at_box_tf;
    

    tf2::Transform box_tf = poseMsgToTransform(box_pose);  
    
    box_pose_place.position = goal->place_pose.position; 
    box_pose_place.orientation = goal->place_pose.orientation;

    tf2::Transform tcp_at_box_tf_place;

    tf2::Transform box_tf_place = poseMsgToTransform(box_pose_place);  


    // start of createManipulationPoses

    tf2::Vector3 box_position(box_pose.position.x, box_pose.position.y, box_pose.position.z); 
    tcp_at_box_tf.setOrigin(box_position); 

    tcp_at_box_tf.setRotation(box_tf.getRotation() * tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI)); // M_PI because it needs to be oriented by 180 degrees from top down


    tf2::Vector3 box_position_place(box_pose_place.position.x, box_pose_place.position.y, box_pose_place.position.z); 
    tcp_at_box_tf_place.setOrigin(box_position_place); 

    tcp_at_box_tf_place.setRotation(box_tf_place.getRotation() * tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI));

    
    std::vector<geometry_msgs::msg::Pose> manipulation_poses; 

    std::vector<geometry_msgs::msg::Pose> pick_poses = createManipulationPoses(retreat_dis, approach_dis, tcp_at_box_tf);

    std::vector<geometry_msgs::msg::Pose> place_poses = createManipulationPoses(retreat_dis, approach_dis, tcp_at_box_tf_place);

    manipulation_poses.insert(manipulation_poses.end(), pick_poses.begin(), pick_poses.end());
    
    manipulation_poses.insert(manipulation_poses.end(), place_poses.begin(), place_poses.end());


    // execute all the manipulation poses one by one 
    doPoses(manipulation_poses);

    // TODO: remove unused code for calibration and come up with an easier interface to setting the manipulation poses
    
    // rclcpp::sleep_for(std::chrono::seconds(1));


    RCLCPP_INFO(this->get_logger(), "Setting result to true");

    result->success = true;

    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Move goal succeeded");
    }
    
  }
 
  // end of functions for pick and place application

  void goal_response_callback(std::shared_future<GoalHandleGripperCommand::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void result_callback(const GoalHandleGripperCommand::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received: " + result.result->reached_goal);
  
  }

}; // class PickNPlace

} // namespace innobot_core

int main(int argc, char **argv)
{
  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // Create the PickNPlace node
  auto arm_action_server = std::make_shared<innobot_core::PickNPlace>();

  rclcpp::spin(arm_action_server);

  rclcpp::shutdown();

  return 0;
}