#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "UR5_STOMP demonstration", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    ROS_INFO_NAMED( "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("End effector link: %s", move_group.getEndEffectorLink().c_str());
    
    std::cout << move_group.getCurrentPose();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    // const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    // kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
    


    robot_state::RobotState start_state(*move_group.getCurrentState());
    move_group.setStartState(start_state);

    //POSE
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 0.474989; //0.474989
    target_pose.orientation.x = -0.522257;//-0.522257
    target_pose.orientation.y = 0.47662; // 0.47662
    target_pose.orientation.z = -0.523895; //-0.523895
    target_pose.position.x = 0.000603714;//0.000603714
    target_pose.position.y = -0.335967;//-0.335967
    target_pose.position.z = 0.305978;//0.305978
    move_group.setPoseTarget(target_pose);


    // JOINTS
    //  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //  std::vector<double> joint_group_positions;
    //  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    //  joint_group_positions[0] = -2.13083290537;  //positivo esquerda , negativo direita , gira a base
    //  joint_group_positions[1] = -1.4019123091;  // radians positivo abaixo, negativo acima, primeira parte da base
    //  joint_group_positions[2] = 2.2600063779; // positivo pra frente, negativo pra trás, segunda parte da base ~braço
    //  joint_group_positions[3] = 2.28247727295;	
    //  joint_group_positions[4] = 0.557602172179;
    //  joint_group_positions[5] = 6.18913213329;
    //  move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.plan(my_plan);
    move_group.execute(my_plan);

    ros::shutdown(); 
        return 0;
} 
    // 