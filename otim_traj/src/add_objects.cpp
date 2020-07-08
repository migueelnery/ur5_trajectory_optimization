#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_objects");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    // Display debug information in teminal
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }


    // Define publisher to update work scene
    ros::Publisher pub_work_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(pub_work_scene.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // Define table plane
    shape_msgs::SolidPrimitive table;
    table.type = table.BOX;
    table.dimensions.resize(3);
    table.dimensions[0] = 1.7; // x
    table.dimensions[1] = 0.8; // y
    table.dimensions[2] = 0.03; // z
    // Define table position
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = -0.35;
    table_pose.position.y = 0;
    table_pose.position.z = -0.02;

    // Define collision objects
    moveit_msgs::CollisionObject collision_objects;
    collision_objects.id = "table";
    collision_objects.header.frame_id = "world";
    collision_objects.primitives.push_back(table);
    collision_objects.primitive_poses.push_back(table_pose);
    collision_objects.operation = collision_objects.ADD;

    // Define makerbot components
    shape_msgs::SolidPrimitive maker1;
    maker1.type = maker1.BOX;
    maker1.dimensions.resize(3);
    maker1.dimensions[0] = 0.4;
    maker1.dimensions[1] = 0.02;
    maker1.dimensions[2] = 0.5;

    shape_msgs::SolidPrimitive maker2;
    maker2.type = maker2.BOX;
    maker2.dimensions.resize(3);
    maker2.dimensions[0] = 0.4;
    maker2.dimensions[1] = 0.42;
    maker2.dimensions[2] = 0.02;

    //Define positions
    geometry_msgs::Pose maker1L_pose;
    maker1L_pose.orientation.w = 1.0;
    maker1L_pose.position.x = -0.91;
    maker1L_pose.position.y = -0.21;
    maker1L_pose.position.z = 0.25;

    geometry_msgs::Pose maker1R_pose;
    maker1R_pose.orientation.w = 1.0;
    maker1R_pose.position.x = -0.91;
    maker1R_pose.position.y = 0.22;
    maker1R_pose.position.z = 0.25;

    geometry_msgs::Pose maker2D_pose;
    maker2D_pose.orientation.w = 1.0;
    maker2D_pose.position.x = -0.91;
    maker2D_pose.position.y = 0.0;
    maker2D_pose.position.z = 0.15;

    geometry_msgs::Pose maker2U_pose;
    maker2U_pose.orientation.w = 1.0;
    maker2U_pose.position.x = -0.91;
    maker2U_pose.position.y = 0.0;
    maker2U_pose.position.z = 0.35;

    //Define collision objects
    moveit_msgs::CollisionObject maker1L_collision;
    maker1L_collision.id = "maker1L";
    maker1L_collision.header.frame_id = "world";
    maker1L_collision.primitives.push_back(maker1);
    maker1L_collision.primitive_poses.push_back(maker1L_pose);
    maker1L_collision.operation = collision_objects.ADD;   

    moveit_msgs::CollisionObject maker1R_collision;
    maker1R_collision.id = "maker1R";
    maker1R_collision.header.frame_id = "world";
    maker1R_collision.primitives.push_back(maker1);
    maker1R_collision.primitive_poses.push_back(maker1R_pose);
    maker1R_collision.operation = collision_objects.ADD;   

    moveit_msgs::CollisionObject maker2D_collision;
    maker2D_collision.id = "maker2D";
    maker2D_collision.header.frame_id = "world";
    maker2D_collision.primitives.push_back(maker2);
    maker2D_collision.primitive_poses.push_back(maker2D_pose);
    maker2D_collision.operation = collision_objects.ADD;   

    moveit_msgs::CollisionObject maker2U_collision;
    maker2U_collision.id = "maker2U";
    maker2U_collision.header.frame_id = "world";
    maker2U_collision.primitives.push_back(maker2);
    maker2U_collision.primitive_poses.push_back(maker2U_pose);
    maker2U_collision.operation = collision_objects.ADD;   

    // Define A Cylinder
    shape_msgs::SolidPrimitive object1;
    object1.type = object1.CYLINDER;
    object1.dimensions.resize(2);
    object1.dimensions[0] = 0.12; // height
    object1.dimensions[1] = 0.02; // radius
    // Define cylinder position
    geometry_msgs::Pose object1_pose;
    object1_pose.orientation.w = 1.0;
    object1_pose.position.x = -0.9; //-0.85
    object1_pose.position.y = 0.0; 
    object1_pose.position.z = 0.21;  //0.2


    // Define collision2 objects
    moveit_msgs::CollisionObject collision_objects2;
    collision_objects2.id = "object";
    collision_objects2.header.frame_id = "world";
    collision_objects2.primitives.push_back(object1);
    collision_objects2.primitive_poses.push_back(object1_pose);
    collision_objects2.operation = collision_objects.ADD;


    // Add all objects to environment
    ROS_INFO("Adding the all objects to the work scene.");
    moveit_msgs::PlanningScene work_scene;
    // work_scene.world.collision_objects.push_back(attached_objects.object);
    work_scene.world.collision_objects.push_back(collision_objects);
    work_scene.world.collision_objects.push_back(maker1L_collision);
    work_scene.world.collision_objects.push_back(maker1R_collision);
    work_scene.world.collision_objects.push_back(maker2D_collision);
    work_scene.world.collision_objects.push_back(maker2U_collision);
    work_scene.world.collision_objects.push_back(collision_objects2);
    work_scene.is_diff = true;
    pub_work_scene.publish(work_scene);
    ros::WallDuration(1).sleep();


    ros::shutdown();
    return 0;
}