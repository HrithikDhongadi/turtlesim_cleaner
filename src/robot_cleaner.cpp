#include "ros/ros.h"
#include "iostream"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

const double PI = 3.14159265359;




// Method to move robot straight
void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double relative_angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void setDesiredOrientation(double desired_angle_radians);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
double getDistance(double x1, double y1, double x2, double y2);
void gridClean();
void spiralClean();
void home();


int main(int argc, char **argv)
{
    //initiate a new node named "robot_cleaner"
    ros::init(argc, argv, "robot_cleaner");
    ros::NodeHandle n;

    double speed, angular_speed;
    double distance, angle;
    bool isForward, clockwise;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

    // std::cout<<"Enter Speed: ";
    // std::cin >> speed;
    // std::cout<<"Enter Distance: ";
    // std::cin >> distance;
    // std::cout<<"Forward? ";
    // std::cin >> isForward;

    // move(speed,distance, isForward);

    // std::cout<<"Enter Angular Velocity: ";
    // std::cin >> angular_speed;
    // std::cout<<"Enter Angle: ";
    // std::cin >> angle;
    // std::cout<<"Clockwise? ";
    // std::cin >> clockwise;

    // rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);

    // setDesiredOrientation(degrees2radians(120));
    ros::Rate loop_rate(0.5);
    // loop_rate.sleep();
    // setDesiredOrientation(degrees2radians(-60));
    // loop_rate.sleep();
    // setDesiredOrientation(degrees2radians(0));

    // turtlesim::Pose goal_pose;
    // goal_pose.x = 1;
    // goal_pose.y = 1;
    // goal_pose.theta = 0;
    // moveGoal(goal_pose, 0.01);
    // loop_rate.sleep();
    
    spiralClean();

    // ros::spin();
}


/* 
 
 * Makes the robot move with a certain linear velocity for  a
 * certain distance in a forward or backward straight direction
 */

void move(double speed, double distance, bool isForward)
{
    geometry_msgs::Twist vel_msg;
    // distance = speed * time


    // Set random linear velocity in x
    if(isForward){
        vel_msg.linear.x = abs(speed);
    }        
    else{
        vel_msg.linear.x =  -abs(speed);
    }
        
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    // Set random angular velocity in x

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    //t0 : current time
    double  t0 = ros::Time::now().toSec();
    double current_distance = 0;
    ros::Rate loop_rate(100);
    do{
        velocity_publisher.publish(vel_msg);
        double t1=ros::Time::now().toSec();
        current_distance = speed * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    }while(current_distance<distance);
    vel_msg.linear.x=0;
    velocity_publisher.publish(vel_msg);
    //loop
    //publish velocity
    //estimate the distance  = speed * (t1 - t0)
    //current distance moved by robot <= distance
}

void rotate(double angular_speed, double relative_angle, bool clockwise)
{
    geometry_msgs::Twist vel_msg;
    
    // Set random angular velocity in x
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    // Set random angular velocity in x
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    if(clockwise)
        vel_msg.angular.z = -abs(angular_speed);
    else
        vel_msg.angular.z = abs(angular_speed);

    double current_angle = 0.0;
    double t0 = ros::Time::now().toSec();
    ros::Rate loop_rate(100);

    do{
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    }while(current_angle<relative_angle);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees)
{
    return angle_in_degrees *PI/180.0; 
}

void setDesiredOrientation(double desired_angle_radians)
{
    double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
    bool clockwise = ((relative_angle_radians<0)?true:false);
    rotate(abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message)
{
    turtlesim_pose.x = pose_message->x;
    turtlesim_pose.y = pose_message->y;
    turtlesim_pose.theta = pose_message->theta;
}

double getDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance)
{
    geometry_msgs::Twist vel_msg;

    ros::Rate loop_rate(10);
    do{
        /********Proportional Controller********/
        // linear velocity in the x-axis
        vel_msg.linear.x = 1.5*(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y));
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        //angular velocity in the y-axis
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 4*(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);

        velocity_publisher.publish(vel_msg);
        
        ros::spinOnce();
        loop_rate.sleep();    
    }while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y));
    std::cout<<"end move goal"<<std::endl;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

void gridClean()
{
    ros::Rate loop(0.5);
    turtlesim::Pose pose;
    pose.x = 1;
    pose.y = 1;
    pose.theta = 0;
    moveGoal(pose, 0.1);
    loop.sleep();
    // setDesiredOrientation(0);
    // loop.sleep();

    move(2,9,true);
    loop.sleep();
    rotate(degrees2radians(90),degrees2radians(90), false);
    loop.sleep();
    move(2,9,true);

    rotate(degrees2radians(90),degrees2radians(90), false);
    move(2,1,true);
    loop.sleep();
    rotate(degrees2radians(90),degrees2radians(90), false);
    loop.sleep();
    move(2,9,true);

    rotate(degrees2radians(90),degrees2radians(90), true);
    move(2,1,true);
    loop.sleep();
    rotate(degrees2radians(90),degrees2radians(90), true);
    loop.sleep();
    move(2,9,true);

    rotate(degrees2radians(90),degrees2radians(90), false);
    move(2,1,true);
    loop.sleep();
    rotate(degrees2radians(90),degrees2radians(90), false);
    loop.sleep();
    move(2,9,true);
}

void spiralClean()
{
    geometry_msgs::Twist vel_msg;
    double count = 0;

    double constant_speed = 4;
    double vk = 1;
    double wk = 2;
    double rk = 0.5;
    ros::Rate loop(1);

    do{
        rk += 0.5;
        vel_msg.linear.x = rk;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = constant_speed; //((vk)/(0.5+rk));

        std::cout << "vel_msg.linear.x =" << vel_msg.linear.x << std::endl;
        std::cout << "vel_msg.linear.y =" << vel_msg.linear.y << std::endl;
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();

        loop.sleep();
    }while((turtlesim_pose.x<10.5)&&turtlesim_pose.y<10.5);
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);

    home();
}

void home()
{
    ros::Rate loop(1);
    turtlesim::Pose pose;

    pose.x = 1;
    pose.y = 1;
    pose.theta = 0;

    loop.sleep();
    setDesiredOrientation(degrees2radians(90));
    loop.sleep(); 

}
