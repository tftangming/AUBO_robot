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
    vector<double> rcm {1.5, 0, 0.58};
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
    double r=length-(start_pos[2]-0.55);
    double alpha, beta, theta;
    target_pose.position.y = start_pos[1];
    beta=0;
    theta=0;
    for(int i=1;i<=20;i++)
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
        sleep(1);

        //显示轨迹
        display_trajectory.trajectory_start = plan.start_state_;
        display_trajectory.trajectory.push_back(plan.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(3);
        group.move();
        sleep(3);
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


    vector<double> rot_start_pos {current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z};
    double r1 = sqrt(pow(rot_start_pos[0]-rcm[0],2)+pow(rot_start_pos[1]-rcm[1],2));
    double l1 = sqrt(pow(rot_start_pos[0]-rcm[0],2)+pow(rot_start_pos[1]-rcm[1],2)+pow(rot_start_pos[2]-rcm[2],2));
    double l2 = length-l1;
    double r2 = r1*l2/l1;

    tf::Pose start_pose2;
    geometry_msgs::PoseStamped initial_pos2= group.getCurrentPose();
    vector<double> start_pos2 {initial_pos2.pose.position.x,initial_pos2.pose.position.y,initial_pos2.pose.position.z};
    vector<double> start_rot2 {initial_pos2.pose.orientation.w,initial_pos2.pose.orientation.x,initial_pos2.pose.orientation.y,initial_pos2.pose.orientation.z};
    start_pose2.setOrigin(tf::Vector3(start_pos2[0], start_pos2[1],  start_pos2[2]));
    start_pose2.setRotation(tf::Quaternion(start_rot2[0], start_rot2[1], start_rot2[2], start_rot2[3]));
    std::vector<geometry_msgs::Pose> waypoints2;
    geometry_msgs::Pose target_pose2;
    tf::poseTFToMsg(start_pose2, target_pose2);
    //waypoints2.push_back(target_pose2);
    double alpha2_0,alpha2, beta2, theta2;
    target_pose2.position.z = start_pos2[2];
    alpha2_0 = 3.1415;
    theta2 = 0;
    double sigma = asin(r1/l1);
    cout <<"start_pos2:"<<start_pos2[0] <<" "<<start_pos2[1]<<" "<<start_pos2[2]<<endl;
    cout<<"rcm:"<<rcm[0]<<" "<<rcm[1]<<" "<<rcm[2]<<endl;
    cout<< "r1:"<<r1<<" ,l1:"<<l1<<endl;
    for(int i=1;i<=90;i++)
    {
        //theta=(-3.0*i)/180*3.1415;
        double fie2 = (2*i*3.1415)/360;
        //alpha2 = alpha2_0 + asin(sin(fie2)*sin(sigma));
        //cout << "alpha2:"<<alpha2 <<endl;
        //beta2 = atan2(cos(fie2),1.0/tan(sigma));
        //cout << "beta2:"<<beta2 <<endl;
        alpha2 = alpha2_0;
        beta2 = sigma;
        theta2 = fie2;
        //下摆
        target_pose2.position.x = rcm[0] + r1*cos(fie2);
        target_pose2.position.y = rcm[1] + r1*sin(fie2);

        target_pose2.orientation.w = cos(alpha2/2)*cos(beta2/2)*cos(theta2/2)+sin(alpha2/2)*sin(beta2/2)*sin(theta2/2);
        target_pose2.orientation.x = sin(alpha2/2)*cos(beta2/2)*cos(theta2/2)-cos(alpha2/2)*sin(beta2/2)*sin(theta2/2);
        target_pose2.orientation.y = cos(alpha2/2)*sin(beta2/2)*cos(theta2/2)+sin(alpha2/2)*cos(beta2/2)*sin(theta2/2);
        target_pose2.orientation.z = cos(alpha2/2)*cos(beta2/2)*sin(theta2/2)-sin(alpha2/2)*sin(beta2/2)*cos(theta2/2);
        waypoints2.push_back(target_pose2);
        cout << target_pose2.position.x << " "<<target_pose2.position.y <<" " << target_pose2.position.z<< endl;
    }
    //设置规划群组的目标位姿
    group.setPoseTarget(waypoints2[waypoints2.size()-1]);
    moveit_msgs::RobotTrajectory trajectory2;
    int maxtries2 = 100;   //最大尝试规划次数
    int attempts2 = 0;     //已经尝试规划次数
    double fraction2 = 0.0;
    while(fraction2 < 1.0 && attempts2 < maxtries2)
    {
        fraction2 = group.computeCartesianPath(waypoints2,
                                              0.01,  // eef_step
                                              0.0,   // jump_threshold
                                              trajectory2);
        attempts2++;

        if(attempts2 % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts2);
    }

    if(fraction2 == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        //执行规划
        moveit::planning_interface::MoveGroup::Plan plan2;
        plan2.trajectory_=trajectory2;
        group.execute(plan2);
        sleep(1);

        //显示轨迹
        display_trajectory.trajectory_start = plan2.start_state_;
        display_trajectory.trajectory.push_back(plan2.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(3);
        group.move();
        sleep(3);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction2, maxtries2);
    }
    //输入当前末端执行器姿态
    current_pose=group.getCurrentPose();
    ROS_INFO("current pose x: %f", current_pose.pose.position.x);
    ROS_INFO("current pose y: %f", current_pose.pose.position.y);
    ROS_INFO("current pose z: %f", current_pose.pose.position.z);
    const Eigen::Affine3d& end_effector_state4 = group.getCurrentState()->getGlobalLinkTransform("ee_link");
    ROS_INFO_STREAM("Translation of the ee_link: \n" << end_effector_state4.translation() << "\n");
    ROS_INFO_STREAM("Rotation of the ee_link: \n" << end_effector_state4.rotation() << "\n");

    ros::shutdown();
    return 0;
}


