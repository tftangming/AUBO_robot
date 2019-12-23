#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


#include <math.h>


//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/robot_state/conversions.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_plan");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    /* This sleep is ONLY to allow Rviz to come up */
    //sleep(10.0);


    moveit::planning_interface::MoveGroup group("manipulator_i5");

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // (Optional) Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO("The end-effector link: %s", group.getEndEffectorLink().c_str());

    //回到最初始位置
    std::map<std::string, double> joints;
    joints["shoulder_joint"] = 0;
    joints["upperArm_joint"] = 0;
    joints["foreArm_joint"] = 0;
    joints["wrist1_joint"] = 0;
    joints["wrist2_joint"] = 0;
    joints["wrist3_joint"] = 0;

    group.setJointValueTarget(joints);
    group.move();

    ros::shutdown();
    return 0;
}
