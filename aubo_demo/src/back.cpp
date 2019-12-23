//
// Created by tangming on 19-10-10.
//
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
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

using namespace std;
using namespace Eigen;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_plan");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("manipulator_i5");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("The reference frame for the robot: %s", group.getPlanningFrame().c_str());  //world
    ROS_INFO("The end-effector link for this group: %s", group.getEndEffectorLink().c_str()); // ee_link

    //提取机械臂末端执行器当前位置信息
//    robot_model::RobotModelConstPtr kinematic_model = group.getRobotModel();
//    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
//    robot_state::RobotStatePtr 	kinematic_state = group.getCurrentState();
//    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");
//    ROS_INFO_STREAM("Translation of the ee_link: \n" << end_effector_state.translation() << "\n");
//    ROS_INFO_STREAM("Rotation of the ee_link: \n" << end_effector_state.rotation() << "\n");
//
//    vector<double> initial_pos {end_effector_state.translation()[0],end_effector_state.translation()[1],end_effector_state.translation()[2]};
    double length = 0.3775;
    vector<double> rcm {1.5, 0, 0.54};
//    cout << end_effector_state.translation()[0]<<" " << initial_pos[0]<<endl;
//    cout << end_effector_state.translation()[1]<<" " << initial_pos[1]<<endl;
//    cout << end_effector_state.translation()[2]<<" " << initial_pos[2]<<endl;
    //初始位置约为(1.5, 0.5, 0.8)
    //RCM点预设位置为(1.5, 0.5, 0.58)

    //输出末端执行器当前位姿
    const Eigen::Affine3d& end_effector_state = group.getCurrentState()->getGlobalLinkTransform("ee_link");
    ROS_INFO_STREAM("Translation of the ee_link: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation of the ee_link: \n" << end_effector_state.rotation() << "\n");

    tf::Pose start_pose0;
    geometry_msgs::PoseStamped initial_pos= group.getCurrentPose();
    vector<double> start_pos {initial_pos.pose.position.x,initial_pos.pose.position.y,initial_pos.pose.position.z};
    vector<double> start_rot {initial_pos.pose.orientation.w,initial_pos.pose.orientation.x,initial_pos.pose.orientation.y,initial_pos.pose.orientation.z};
    start_pose0.setOrigin(tf::Vector3(start_pos[0], start_pos[1],  start_pos[2]));
    start_pose0.setRotation(tf::Quaternion(start_rot[0], start_rot[1], start_rot[2], start_rot[3]));
    //向下, waypoints中包含了另一半的采样点（60°的扇形区域）->执行器末端沿纸面向上
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose;
    tf::poseTFToMsg(start_pose0, target_pose);
    double r=length-(start_pos[2]-rcm[2]);
    double alpha, beta, theta;
    target_pose.position.y = start_pos[1];
    beta=0;
    theta=0;
    for(int i=1;i<=30;i++)
    {
        //垂直地面向下进给
        double delta_z = 0.0015*i;
        target_pose.position.x = start_pos[0];
        target_pose.position.y = start_pos[1];
        target_pose.position.z = start_pos[2] + delta_z;
        target_pose.orientation.w = start_rot[0];
        target_pose.orientation.x = start_rot[1];
        target_pose.orientation.y = start_rot[2];
        target_pose.orientation.z = start_rot[3];
        waypoints.push_back(target_pose);

        //cout << target_pose.position.x << " "<<target_pose.position.y <<" " << target_pose.position.z<< endl;
    }
    //cout << target_pose.position.x << " "<< target_pose.position.y << " "<<target_pose.position.z<<endl;
    //输出当前末端执行器姿态
    geometry_msgs::PoseStamped current_pose;
    current_pose=group.getCurrentPose();
    ROS_INFO("current pose x: %f", current_pose.pose.position.x);
    ROS_INFO("current pose y: %f", current_pose.pose.position.y);
    ROS_INFO("current pose z: %f", current_pose.pose.position.z);
    const Eigen::Affine3d& end_effector_state2 = group.getCurrentState()->getGlobalLinkTransform("ee_link");
    ROS_INFO_STREAM("Translation of the ee_link: \n" << end_effector_state2.translation() << "\n");
    ROS_INFO_STREAM("Rotation of the ee_link: \n" << end_effector_state2.rotation() << "\n");

    //设置规划群组的目标位姿
    group.setPoseTarget(waypoints[waypoints.size()-1]);

    moveit_msgs::RobotTrajectory trajectory;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
    double fraction = 0.0;
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = group.computeCartesianPath(waypoints,
                                              0.01,  // eef_step
                                              0.0,   // jump_threshold
                                              trajectory);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        //执行规划
        moveit::planning_interface::MoveGroup::Plan plan;
        plan.trajectory_=trajectory;
        group.execute(plan);

        //显示轨迹
        display_trajectory.trajectory_start = plan.start_state_;
        display_trajectory.trajectory.push_back(plan.trajectory_);
        display_publisher.publish(display_trajectory);
        group.move();
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    //输入当前末端执行器姿态
    current_pose=group.getCurrentPose();
    ROS_INFO("current pose x: %f", current_pose.pose.position.x);
    ROS_INFO("current pose y: %f", current_pose.pose.position.y);
    ROS_INFO("current pose z: %f", current_pose.pose.position.z);
    const Eigen::Affine3d& end_effector_state3 = group.getCurrentState()->getGlobalLinkTransform("ee_link");
    ROS_INFO_STREAM("Translation of the ee_link: \n" << end_effector_state3.translation() << "\n");
    ROS_INFO_STREAM("Rotation of the ee_link: \n" << end_effector_state3.rotation() << "\n");

    ros::shutdown();
    return 0;
}





