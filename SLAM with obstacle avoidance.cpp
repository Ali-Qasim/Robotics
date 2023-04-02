#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <fstream>
#include <math.h>

#define PI 3.141592653589

using namespace std;

class MyRobot {
public:
    MyRobot();
    void Moving();
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odomMsg);
    void set_velocity(double velocity_x, double velocity_z);
    void stop();
    void avoid();
    double calcAngle(double pointX, double pointY);
    double my_x = 0.0;
    double my_y = 0.0;
    double my_th = 0.0;

    double lDist;   //left 'antenna'
    double rDist;   //right 'antenna'

    double points[4][2] = {{-1, 2}, {2, 0.5}, {1, -2}, {-2, -0.5}};    //array of waypoints
    ofstream myfile;

    double my_ls[360];//distance from obstacle on each degree

    ros::NodeHandle nh;
    ros::Publisher cmdPub;
    ros::Subscriber laserSub;
    ros::Subscriber odomSub;
    geometry_msgs::Twist my_move;
private:
};

MyRobot::MyRobot(){
    MyRobot::cmdPub = MyRobot::nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    MyRobot::laserSub = MyRobot::nh.subscribe("/scan",1000, &MyRobot::laser_callback,this);
    MyRobot::odomSub = MyRobot::nh.subscribe("/odom", 100, &MyRobot::odom_callback,this);
}

void MyRobot::Moving() {
    myfile.open("/home/local/CAMPUS/ms19569/M-Drive/turtlebot_ws/src/ce215_pkg/src/myodom.txt");
    ros::Rate rate(100);

    if (ros::ok) {//MOVEMENT IN HERE
        for (int i = 0; i < 4; i++) {    //num of points
            bool reached = false;
            stop();

            //set destination coords
            double destX = points[i][0];
            double destY = points[i][1];

            stop();   //reset velocity before starting each
            ros::Duration(0.2).sleep();
            ros::spinOnce();

            while (!reached) {
                //update odom and sensor data
                ros::spinOnce();

                //calculate angle to point
                ros::spinOnce();
                double angle = calcAngle(destX, destY) - my_th;

                //turn toward point
                if (!(my_th < angle + (PI / 45) && my_th > angle - (PI / 45))) {    //within 4 degree margin

                    if(angle>0){
                        set_velocity(0.0, 0.5);
                    } else {
                        set_velocity(0.0, -0.5);
                    }
                    ros::Duration(abs(angle*2)).sleep();
                }

                //avoid obstacles
                avoid();

                //move to point
                set_velocity(0.15, 0);
                ros::Duration(0.2).sleep();

                //if reached, break
                ros::spinOnce();
                double xDist = destX - my_x;
                double yDist = destY - my_y;

                if (hypot(xDist, yDist) < 0.3) {    //if within 0.1 of destination (radially)
                    reached = true;
                }

                rate.sleep();
            }
        }
    }
    myfile.close();
}

double MyRobot::calcAngle(double pointX, double pointY){    //calculates angle (from x-axis) from current point to dest. point
    double angle, dX, dY;
    dY = pointY - this->my_y;
    dX = pointX - this->my_x;
    angle = atan2(dY,dX);
    return angle;
}

void MyRobot::avoid() {
    ros::spinOnce();
    stop();

    if (lDist < rDist) {
        while (lDist < 0.4) {//rotate past object
            //update
            ros::spinOnce();

            set_velocity(0, 0.4);    //rotate
            ros::Duration(0.1).sleep();
        }
    } else {
        while (rDist < 0.4) {  //rotate past object
            //update
            ros::spinOnce();

            set_velocity(0, -0.4);    //rotate
            ros::Duration(0.1).sleep();
        }
    }

    //once obstacle is avoided, stop rotating and move past
    set_velocity(0.10, 0);
    ros::Duration(1).sleep();
}

void MyRobot::set_velocity(double velocity_x, double velocity_w)
{
    my_move.linear.x = velocity_x;
    my_move.linear.y = 0;
    my_move.angular.z = velocity_w; //angle
    cmdPub.publish(my_move);
}   //forward

void MyRobot::stop(){
    my_move.linear.x = 0;
    my_move.linear.y = 0;
    my_move.angular.z = 0;
    cmdPub.publish(my_move);
}

void MyRobot::laser_callback(const sensor_msgs::LaserScan::ConstPtr& laserMsg) {
    for (int i = 0; i < laserMsg->ranges.size(); i++) {
        if (laserMsg->ranges[i] > 0.4) MyRobot::my_ls[i] = 0.4;   //cap detected distance at 0.4
        else MyRobot::my_ls[i] = laserMsg->ranges[i];
    }

    lDist = this->my_ls[360 - 24];   //left
    rDist = this->my_ls[24];         //right

}

void MyRobot::odom_callback(const nav_msgs::Odometry::ConstPtr &odomMsg){

    MyRobot::my_x = odomMsg->pose.pose.position.x;
    MyRobot::my_y = odomMsg->pose.pose.position.y;

    tf::Quaternion q(
            odomMsg ->pose.pose.orientation.x,
            odomMsg ->pose.pose.orientation.y,
            odomMsg ->pose.pose.orientation.z,
            odomMsg ->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    MyRobot::my_th = yaw;

    myfile << this->my_x << "," << this->my_y << "," << endl;
}

int main(int argc,char**argv){
    ros::init(argc, argv, "myrobot_node");
    MyRobot mr;
    mr.Moving();

    ROS_INFO("completed!");

    return 0;
}