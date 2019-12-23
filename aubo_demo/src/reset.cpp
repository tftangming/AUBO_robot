#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


#include <math.h>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>

//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/robot_state/conversions.h>

using namespace std;
using namespace Eigen;
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

    //第一步：复位位置
    tf::Pose random_pose0;
    //random_pose.setOrigin(tf::Vector3(0.54882, -0.30854,  0.65841));
    //random_pose.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));
    random_pose0.setOrigin(tf::Vector3(1.5, 0,  0.8));
    //random_pose0.setRotation(tf::Quaternion(0.707, 0, 0, 0.707));
    random_pose0.setRotation(tf::Quaternion(1, 0, 0, 0));

    geometry_msgs::Pose target_pose0;
    tf::poseTFToMsg(random_pose0, target_pose0);
    group.setPoseTarget(target_pose0);


    moveit::planning_interface::MoveGroup::Plan plan1;
    bool success = static_cast<bool>(group.plan(plan1));

    ROS_INFO("Visualizing plan %s",success?"SUCCESS":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(2);
    group.move();
    return 0;
}
