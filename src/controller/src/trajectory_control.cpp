#include "controller/trajectory_read.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float64.h"
#include <math.h>
using namespace std;
#define PI 3.1416

std::vector<Trajectory> trajectoryData;
uint32_t controlCount = 1;
Car simCar;


void CarInit(void)
{
    simCar.x = 40;
    simCar.y = 28.25;
    simCar.theta = 0;
    simCar.speed = 0;
    simCar.steeringWheelAngle = 0;
    simCar.throttle = 0;
    simCar.brake = 0;
}

uint32_t FindClosestPoint(float x,float y)
{
    float mindist = 100000,temp;
    uint32_t mark = 0;
    for (int i = 0; i < trajectoryData.size(); i++)
    {   
        temp = (trajectoryData[i].x - x) * (trajectoryData[i].x - x) + (trajectoryData[i].y - y) * (trajectoryData[i].y - y);
        if (mindist > temp)
        {
            mindist = temp;
            mark = i;
        }
    }

    return mark;
}

float GetAngle(float x1, float y1, float x2, float y2)
{
	float x = x1 - x2;//t d
	float y = y1 - y2;//z y
    float angle = -1;
	if (y == 0 && x > 0) angle = 0;
	if (y == 0 && x < 0) angle = PI;
	if (x ==0 && y > 0) angle = PI/2;
	if (x == 0 && y < 0) angle = PI*3/2;
    if (angle == -1)
    {   
        if (x > 0) angle = atan(y/x) ;
        if (x < 0) angle = atan(y/x) + PI;
    }
    if (angle < 0)
    {
       angle += 2*PI;
    }

	return angle;
}

float CalSteerAngle(uint32_t mark,float x,float y,float heading)
{
    int windowSize = 10;
    float target_x, target_y;
    float angleDist = 0;
    float steerAngleOut = 0;
    float KP = 20;
    uint32_t point = 0;
    point = mark + windowSize;
    if (point >= (trajectoryData.size())) point = point-trajectoryData.size();
    
    target_x = trajectoryData[point].x;
    target_y = trajectoryData[point].y;
    angleDist = GetAngle(target_x,target_y,x,y) - heading;
    
    if (angleDist >= PI) angleDist -= 2*PI;
    if (angleDist < -PI) angleDist += 2*PI;
    steerAngleOut = KP * angleDist;
    return steerAngleOut;
}

float CalThrottleOut(uint32_t mark,float speed)
{
    float throttleOut = 0;
    float KP = 10; 
    throttleOut = KP * (trajectoryData[mark].speed - speed);
    if(throttleOut > 50) throttleOut = 50;
    if(throttleOut < 0) throttleOut = 0;
    return throttleOut;
}

void  CalControlOut(void)
{   
    static uint32_t mark;
    mark = FindClosestPoint(simCar.x, simCar.y);
    simCar.throttle = CalThrottleOut(mark,simCar.speed); 
    simCar.steeringWheelAngle = CalSteerAngle(mark,simCar.x,simCar.y,simCar.theta);
}

void sim_Pose2DCallback(const geometry_msgs::Pose2D& pose)
{
    simCar.x = pose.x;
    simCar.y = pose.y;
    simCar.theta = pose.theta;
}

void sim_SpeedCallback(const std_msgs::Float64& msg)
{
    simCar.speed = msg.data;
}

int main(int argc,char **argv)
{   
    
    geometry_msgs::Point controlData_msg;
    ros::init(argc,argv,"trajectory_control");
    ros::NodeHandle n;    
    trajectory_simdata_read(&trajectoryData);
    ros::Rate loop_rate(1000);
    ros::Subscriber sub1 = n.subscribe("sim_Pose2D",1000,sim_Pose2DCallback);
    ros::Subscriber sub2 = n.subscribe("sim_Speed",1000,sim_SpeedCallback);
    ros::Publisher pub1 = n.advertise<geometry_msgs::Point>("sim_Control",1000);
    CarInit();
    while(ros::ok())
    {
        controlCount++;
        if (controlCount >= 20)
        {
            controlCount = 0;
            CalControlOut();
            controlData_msg.x = simCar.steeringWheelAngle;
            controlData_msg.y = simCar.throttle;
            
            controlData_msg.z = simCar.brake;
            pub1.publish(controlData_msg);
        }
    ros::spinOnce();    
    loop_rate.sleep();   
    }


}