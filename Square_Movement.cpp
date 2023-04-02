
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

   double my_x = 0.0;
   double my_y = 0.0;
   double my_th = 0.0;

   int corner = 1; //counts which corner the robot is targetting

   double vel = 0.2;

   double my_ls[360]; //distance from obstacle on each degree
   
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

   ofstream myfile;

   myfile.open("/home/local/CAMPUS/ms19569/M-Drive/turtlebot_ws/src/ce215_pkg/src/myodom.txt");

   ros::Rate rate(100);

   double x = 0; //initial x-displacement
   double y = 0; //initial y-displacement
   double th = 0; //initial angle

   this->set_velocity(0, 0);
   ros::Duration(0.5).sleep();

   while (ros::ok()) {

       double dX = this->my_x - x; //displacement in x
       double dY = this->my_y - y; //displacement in y
       double dth = this->my_th - th; //change in angle

       double absDist = sqrt(dX * dX + dY * dY);//absolute displacement of robot

       if (absDist > 0.98) { //slow down before reaching corner

           this->set_velocity(0, 0);
           ros::Duration(0.3).sleep();

           //turn here
           while (my_th < (th + PI / 2)) {

               //update dth
               ros::spinOnce();
               rate.sleep();

               dth = this->my_th - th;

               if (my_th<0 && (this->corner > 2) ){

                   dth = th + PI/2; // making sure my_th gives positive angle values.

               }

               if (abs(dth) < (PI / 2)) { //rotate 90 degrees

                   set_velocity(0, PI / 8);

               } else {

                   this->set_velocity(0, 0);
                   ros::Duration(0.5).sleep();

                   break;

               }
           }

           if (corner == 4) break; else this->corner++;

           ros::spinOnce();
           rate.sleep();

           //reset starting coords and trajectory
           th = (PI/2 * (corner-1)) - PI; // setting corner angle
           th = corner > 2 ? th : -th; //invert for corner 3 and 4;

           this->my_th = corner > 2 ? -this->my_th : this->my_th;

           myfile << this->my_x << "," << this->my_y << "," << this->my_th << "CORNER" << endl;

           x = this->my_x;
           y = this->my_y;

           ros::Duration(0.5).sleep();
           this->set_velocity(vel, 0);
           ros::Duration(0.5).sleep();


       } else {
           if (dth > PI / 180 ||
               dth < -PI / 180) { // to make robot correct angle in opposite direction if more than 1 degrees off course
               if (dth > 0) {

                   this->set_velocity(0.1, -0.25);

               } else {

                   this->set_velocity(0.1, 0.25);

               }
           } else {

               this->set_velocity(vel, 0); // if angle is acceptable, set angular velocity = 0

           }
       }
       ros::spinOnce();
       rate.sleep();

       myfile << this->my_x << "," << this->my_y << "," << this->my_th << endl;
   }

   myfile.close();
}

void MyRobot::set_velocity(double velocity_x, double velocity_z)
{

   my_move.linear.x = velocity_x;
   my_move.linear.y = 0;
   my_move.angular.z = velocity_z;

   cmdPub.publish(my_move);

}
void MyRobot::stop()    //stop method, can delete
{

   my_move.linear.x = 0;
   my_move.linear.y = 0;
   my_move.angular.z = 0;

   cmdPub.publish(my_move)

}

void MyRobot::laser_callback(const sensor_msgs::LaserScan::ConstPtr& laserMsg) {

    for (int i = 0; i < laserMsg->ranges.size(); i++) { //distance from obstacles on each degree in 360 degrees

        if (laserMsg->ranges[i] > 3) MyRobot::my_ls[i] = 3.0; //if detected range is greater than 3 then set range to 3
        else MyRobot::my_ls[i] = laserMsg->ranges[i];

    }

    ROS_INFO("right %2f",MyRobot::my_ls[270]);
    ROS_INFO("left %2f",MyRobot::my_ls[90]);
}
void MyRobot::odom_callback(const nav_msgs::Odometry::ConstPtr &odomMsg){

   MyRobot::my_x = odomMsg->pose.pose.position.x;
   MyRobot::my_y = odomMsg->pose.pose.position.y;

//covert quaternion to Eular angles
   tf::Quaternion q(

           odomMsg ->pose.pose.orientation.x,
           odomMsg ->pose.pose.orientation.y,
           odomMsg ->pose.pose.orientation.z,
           odomMsg ->pose.pose.orientation.w);

   tf::Matrix3x3 m(q);
   
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   
   MyRobot::my_th = yaw;

   ROS_INFO("my_th: %f ", MyRobot::my_th);

   //ROS_INFO("my_th: %f ", MyRobot::my_th);
}
int main(int argc,char**argv){

   ros::init(argc, argv, "myrobot_node");

   MyRobot mr;
   mr.Moving();

   ROS_INFO("completed!");

   return 0;
}