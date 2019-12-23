//
// Created by tangming on 19-11-20.
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
    //waypoints.push_back(target_pose);
    double r=length-(start_pos[2]-rcm[2]);
    double alpha, beta, theta;
    target_pose.position.y = start_pos[1];
    beta=0;
    theta=0;
    for(int i=0;i<=15;i++)
    {
        //theta=(-3.0*i)/180*3.1415;
        double fie = 1.5*i/180*3.1415926;
        alpha = 3.1415926;
        beta = fie;
        //下摆
        double delta_x = length*sin(fie) - r*tan(fie);
        double delta_z = length*(1-cos(fie));
        target_pose.position.x = start_pos[0] + delta_x;
        target_pose.position.z = start_pos[2] - delta_z;

        target_pose.orientation.w = cos(alpha/2)*cos(beta/2)*cos(theta/2)+sin(alpha/2)*sin(beta/2)*sin(theta/2);
        target_pose.orientation.x = sin(alpha/2)*cos(beta/2)*cos(theta/2)-cos(alpha/2)*sin(beta/2)*sin(theta/2);
        target_pose.orientation.y = cos(alpha/2)*sin(beta/2)*cos(theta/2)+sin(alpha/2)*cos(beta/2)*sin(theta/2);
        target_pose.orientation.z = cos(alpha/2)*cos(beta/2)*sin(theta/2)-sin(alpha/2)*sin(beta/2)*cos(theta/2);
        waypoints.push_back(target_pose);
        cout << target_pose.position.x << " "<<target_pose.position.y <<" " << target_pose.position.z<< endl;
    }
    cout << target_pose.position.x << " "<< target_pose.position.y << " "<<target_pose.position.z<<endl;
    //输入当前末端执行器姿态
    geometry_msgs::PoseStamped current_pose;
    current_pose=group.getCurrentPose();
    ROS_INFO("current pose x: %f", current_pose.pose.position.x);
    ROS_INFO("current pose y: %f", current_pose.pose.position.y);
    ROS_INFO("current pose z: %f", current_pose.pose.position.z);
    const Eigen::Affine3d& end_effector_state = group.getCurrentState()->getGlobalLinkTransform("ee_link");
    ROS_INFO_STREAM("Translation of the ee_link: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation of the ee_link: \n" << end_effector_state.rotation() << "\n");

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
        sleep(1);
        group.move();
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
    const Eigen::Affine3d& end_effector_state_back = group.getCurrentState()->getGlobalLinkTransform("ee_link");
    ROS_INFO_STREAM("Translation of the ee_link: \n" << end_effector_state_back.translation() << "\n");
    ROS_INFO_STREAM("Rotation of the ee_link: \n" << end_effector_state_back.rotation() << "\n");
    std::vector<geometry_msgs::Pose> waypoints_back;
    for(int i=waypoints.size()-1;i>=0;i--)
    {
        waypoints_back.push_back(waypoints[i]);
    }
    //设置规划群组的目标位姿
    group.setPoseTarget(waypoints_back[waypoints_back.size()-1]);
    attempts = 0;
    fraction = 0.0;
    moveit_msgs::RobotTrajectory trajectory_back;
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = group.computeCartesianPath(waypoints_back,
                                              0.01,  // eef_step
                                              0.0,   // jump_threshold
                                              trajectory_back);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        //执行规划
        moveit::planning_interface::MoveGroup::Plan plan_back;
        plan_back.trajectory_=trajectory_back;
        group.execute(plan_back);

//        //显示轨迹
        display_trajectory.trajectory_start = plan_back.start_state_;
        display_trajectory.trajectory.push_back(plan_back.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(2);
        group.move();
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    //向右移动
    initial_pos= group.getCurrentPose();
    vector<double> start_pos2 {initial_pos.pose.position.x,initial_pos.pose.position.y,initial_pos.pose.position.z};
    vector<double> start_rot2 {initial_pos.pose.orientation.w,initial_pos.pose.orientation.x,initial_pos.pose.orientation.y,initial_pos.pose.orientation.z};
    start_pose0.setOrigin(tf::Vector3(start_pos2[0], start_pos2[1],  start_pos2[2]));
    start_pose0.setRotation(tf::Quaternion(start_rot2[0], start_rot2[1], start_rot2[2], start_rot2[3]));
    std::vector<geometry_msgs::Pose> waypoints2;
    tf::poseTFToMsg(start_pose0, target_pose);
    target_pose.position.y = start_pos2[1];
    beta=0;
    theta=0;
    for(int i=0;i<=15;i++)
    {
        //theta=(-3.0*i)/180*3.1415;
        double fie = 1.5*i/180*3.1415926;
        alpha = 3.1415926;
        beta = -fie;
        //下摆
        double delta_x = length*sin(fie) - r*tan(fie);
        double delta_z = length*(1-cos(fie));
        target_pose.position.x = start_pos[0] - delta_x;
        target_pose.position.z = start_pos[2] - delta_z;

        target_pose.orientation.w = cos(alpha/2)*cos(beta/2)*cos(theta/2)+sin(alpha/2)*sin(beta/2)*sin(theta/2);
        target_pose.orientation.x = sin(alpha/2)*cos(beta/2)*cos(theta/2)-cos(alpha/2)*sin(beta/2)*sin(theta/2);
        target_pose.orientation.y = cos(alpha/2)*sin(beta/2)*cos(theta/2)+sin(alpha/2)*cos(beta/2)*sin(theta/2);
        target_pose.orientation.z = cos(alpha/2)*cos(beta/2)*sin(theta/2)-sin(alpha/2)*sin(beta/2)*cos(theta/2);
        waypoints2.push_back(target_pose);
        cout << target_pose.position.x << " "<<target_pose.position.y <<" " << target_pose.position.z<< endl;
    }
    cout << target_pose.position.x << " "<< target_pose.position.y << " "<<target_pose.position.z<<endl;
    //输入当前末端执行器姿态
    geometry_msgs::PoseStamped current_pose2;
    current_pose2=group.getCurrentPose();
    ROS_INFO("current pose x: %f", current_pose2.pose.position.x);
    ROS_INFO("current pose y: %f", current_pose2.pose.position.y);
    ROS_INFO("current pose z: %f", current_pose2.pose.position.z);
    //设置规划群组的目标位姿
    group.setPoseTarget(waypoints2[waypoints2.size()-1]);

    moveit_msgs::RobotTrajectory trajectory2;
    maxtries = 100;   //最大尝试规划次数
    attempts = 0;     //已经尝试规划次数
    fraction = 0.0;
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = group.computeCartesianPath(waypoints2,
                                              0.01,  // eef_step
                                              0.0,   // jump_threshold
                                              trajectory2);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        //执行规划
        moveit::planning_interface::MoveGroup::Plan plan2;
        plan2.trajectory_=trajectory2;
        group.execute(plan2);

//        //显示轨迹
        display_trajectory.trajectory_start = plan2.start_state_;
        display_trajectory.trajectory.push_back(plan2.trajectory_);
        display_publisher.publish(display_trajectory);
        group.move();
        sleep(2);
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
    const Eigen::Affine3d& end_effector_state2 = group.getCurrentState()->getGlobalLinkTransform("ee_link");
    ROS_INFO_STREAM("Translation of the ee_link: \n" << end_effector_state2.translation() << "\n");
    ROS_INFO_STREAM("Rotation of the ee_link: \n" << end_effector_state2.rotation() << "\n");
    //折返到复位位置
    std::vector<geometry_msgs::Pose> waypoints2_back;
    for(int i=waypoints2.size()-1;i>=0;i--)
    {
        waypoints2_back.push_back(waypoints2[i]);
    }
    //设置规划群组的目标位姿
    group.setPoseTarget(waypoints2_back[waypoints2_back.size()-1]);
    attempts = 0;
    fraction = 0.0;
    moveit_msgs::RobotTrajectory trajectory2_back;
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = group.computeCartesianPath(waypoints2_back,
                                              0.01,  // eef_step
                                              0.0,   // jump_threshold
                                              trajectory2_back);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        //执行规划
        moveit::planning_interface::MoveGroup::Plan plan2_back;
        plan2_back.trajectory_=trajectory2_back;
        group.execute(plan2_back);
        //显示轨迹
        display_trajectory.trajectory_start = plan2_back.start_state_;
        display_trajectory.trajectory.push_back(plan2_back.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(2);
        group.move();
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    //向上移动
    initial_pos= group.getCurrentPose();
    vector<double> start_pos3 {initial_pos.pose.position.x,initial_pos.pose.position.y,initial_pos.pose.position.z};
    vector<double> start_rot3 {initial_pos.pose.orientation.w,initial_pos.pose.orientation.x,initial_pos.pose.orientation.y,initial_pos.pose.orientation.z};
    start_pose0.setOrigin(tf::Vector3(start_pos3[0], start_pos3[1],  start_pos3[2]));
    start_pose0.setRotation(tf::Quaternion(start_rot3[0], start_rot3[1], start_rot3[2], start_rot3[3]));
    //向下, waypoints中包含了另一半的采样点（60°的扇形区域）->执行器末端沿纸面向上
    std::vector<geometry_msgs::Pose> waypoints3;
    tf::poseTFToMsg(start_pose0, target_pose);
    target_pose.position.x = start_pos[0];
    beta=0;
    theta=0;
    for(int i=1;i<=15;i++)
    {
        //theta=(-3.0*i)/180*3.1415;
        double fie = 1.5*i/180*3.1415926;
        alpha = 3.1415926 + fie;
        //下摆
        double delta_y = length*sin(fie) - r*tan(fie);
        double delta_z = length*(1-cos(fie));
        target_pose.position.y = start_pos[1] - delta_y;
        target_pose.position.z = start_pos[2] - delta_z;

        target_pose.orientation.w = cos(alpha/2)*cos(beta/2)*cos(theta/2)+sin(alpha/2)*sin(beta/2)*sin(theta/2);
        target_pose.orientation.x = sin(alpha/2)*cos(beta/2)*cos(theta/2)-cos(alpha/2)*sin(beta/2)*sin(theta/2);
        target_pose.orientation.y = cos(alpha/2)*sin(beta/2)*cos(theta/2)+sin(alpha/2)*cos(beta/2)*sin(theta/2);
        target_pose.orientation.z = cos(alpha/2)*cos(beta/2)*sin(theta/2)-sin(alpha/2)*sin(beta/2)*cos(theta/2);
        waypoints3.push_back(target_pose);
        cout << target_pose.position.x << " "<<target_pose.position.y <<" " << target_pose.position.z<< endl;
    }
    cout << target_pose.position.x << " "<< target_pose.position.y << " "<<target_pose.position.z<<endl;

    //设置规划群组的目标位姿
    group.setPoseTarget(waypoints3[waypoints3.size()-1]);

    moveit_msgs::RobotTrajectory trajectory3;
    maxtries = 100;   //最大尝试规划次数
    attempts = 0;     //已经尝试规划次数
    fraction = 0.0;
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = group.computeCartesianPath(waypoints3,
                                              0.01,  // eef_step
                                              0.0,   // jump_threshold
                                              trajectory3);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        //执行规划
        moveit::planning_interface::MoveGroup::Plan plan3;
        plan3.trajectory_=trajectory3;
        group.execute(plan3);

        //显示轨迹
        display_trajectory.trajectory_start = plan3.start_state_;
        display_trajectory.trajectory.push_back(plan3.trajectory_);
        display_publisher.publish(display_trajectory);
        group.move();
        sleep(2);
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
    const Eigen::Affine3d&  end_effector_state3 = group.getCurrentState()->getGlobalLinkTransform("ee_link");
    ROS_INFO_STREAM("Translation of the ee_link: \n" << end_effector_state3.translation() << "\n");
    ROS_INFO_STREAM("Rotation of the ee_link: \n" << end_effector_state3.rotation() << "\n");
    //折返到复位位置
    std::vector<geometry_msgs::Pose> waypoints3_back;
    for(int i=waypoints3.size()-1;i>=0;i--)
    {
        waypoints3_back.push_back(waypoints3[i]);
    }
    //设置规划群组的目标位姿
    group.setPoseTarget(waypoints3_back[waypoints3_back.size()-1]);
    attempts = 0;
    fraction = 0.0;
    moveit_msgs::RobotTrajectory trajectory3_back;
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = group.computeCartesianPath(waypoints3_back,
                                              0.01,  // eef_step
                                              0.0,   // jump_threshold
                                              trajectory3_back);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        //执行规划
        moveit::planning_interface::MoveGroup::Plan plan3_back;
        plan3_back.trajectory_=trajectory3_back;
        group.execute(plan3_back);
        //显示轨迹
        display_trajectory.trajectory_start = plan3_back.start_state_;
        display_trajectory.trajectory.push_back(plan3_back.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(2);
        group.move();
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    //向下移动
    initial_pos= group.getCurrentPose();
    vector<double> start_pos4 {initial_pos.pose.position.x,initial_pos.pose.position.y,initial_pos.pose.position.z};
    vector<double> start_rot4 {initial_pos.pose.orientation.w,initial_pos.pose.orientation.x,initial_pos.pose.orientation.y,initial_pos.pose.orientation.z};
    start_pose0.setOrigin(tf::Vector3(start_pos4[0], start_pos4[1],  start_pos4[2]));
    start_pose0.setRotation(tf::Quaternion(start_rot4[0], start_rot4[1], start_rot4[2], start_rot4[3]));

    std::vector<geometry_msgs::Pose> waypoints4;
    tf::poseTFToMsg(start_pose0, target_pose);
    target_pose.position.x = start_pos[0];
    beta=0;
    theta=0;
    for(int i=1;i<=15;i++)
    {
        //theta=(-3.0*i)/180*3.1415;
        double fie = 1.5*i/180*3.1415926;
        alpha = 3.1415926-fie;
        //下摆
        double delta_y = length*sin(fie) - r*tan(fie);
        double delta_z = length*(1-cos(fie));
        target_pose.position.y = start_pos[1] + delta_y;
        target_pose.position.z = start_pos[2] - delta_z;

        target_pose.orientation.w = cos(alpha/2)*cos(beta/2)*cos(theta/2)+sin(alpha/2)*sin(beta/2)*sin(theta/2);
        target_pose.orientation.x = sin(alpha/2)*cos(beta/2)*cos(theta/2)-cos(alpha/2)*sin(beta/2)*sin(theta/2);
        target_pose.orientation.y = cos(alpha/2)*sin(beta/2)*cos(theta/2)+sin(alpha/2)*cos(beta/2)*sin(theta/2);
        target_pose.orientation.z = cos(alpha/2)*cos(beta/2)*sin(theta/2)-sin(alpha/2)*sin(beta/2)*cos(theta/2);
        waypoints4.push_back(target_pose);
        cout << target_pose.position.x << " "<<target_pose.position.y <<" " << target_pose.position.z<< endl;
    }
    cout << target_pose.position.x << " "<< target_pose.position.y << " "<<target_pose.position.z<<endl;

    //设置规划群组的目标位姿
    group.setPoseTarget(waypoints4[waypoints4.size()-1]);

    moveit_msgs::RobotTrajectory trajectory4;
    maxtries = 100;   //最大尝试规划次数
    attempts = 0;     //已经尝试规划次数
    fraction = 0.0;
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = group.computeCartesianPath(waypoints4,
                                              0.01,  // eef_step
                                              0.0,   // jump_threshold
                                              trajectory4);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        //执行规划
        moveit::planning_interface::MoveGroup::Plan plan4;
        plan4.trajectory_=trajectory4;
        group.execute(plan4);

        //显示轨迹
        display_trajectory.trajectory_start = plan4.start_state_;
        display_trajectory.trajectory.push_back(plan4.trajectory_);
        display_publisher.publish(display_trajectory);
        group.move();
        sleep(2);
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
    const Eigen::Affine3d&  end_effector_state4 = group.getCurrentState()->getGlobalLinkTransform("ee_link");
    ROS_INFO_STREAM("Translation of the ee_link: \n" << end_effector_state4.translation() << "\n");
    ROS_INFO_STREAM("Rotation of the ee_link: \n" << end_effector_state4.rotation() << "\n");
    //折返到复位位置
    std::vector<geometry_msgs::Pose> waypoints4_back;
    for(int i=waypoints4.size()-1;i>=0;i--)
    {
        waypoints4_back.push_back(waypoints4[i]);
    }
    //设置规划群组的目标位姿
    group.setPoseTarget(waypoints4_back[waypoints4_back.size()-1]);
    attempts = 0;
    fraction = 0.0;
    moveit_msgs::RobotTrajectory trajectory4_back;
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = group.computeCartesianPath(waypoints4_back,
                                              0.01,  // eef_step
                                              0.0,   // jump_threshold
                                              trajectory4_back);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        //执行规划
        moveit::planning_interface::MoveGroup::Plan plan4_back;
        plan4_back.trajectory_=trajectory4_back;
        group.execute(plan4_back);
        //显示轨迹
        display_trajectory.trajectory_start = plan4_back.start_state_;
        display_trajectory.trajectory.push_back(plan4_back.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(2);
        group.move();
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    ros::shutdown();
    return 0;
}


