#include "ros/ros.h"
#include "can_driver/wire_control.h"

uint32_t dbg_can_count = 0;

int main(int argc,char **argv)
{   

    ros::init(argc,argv,"dbg_can");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);    
    ros::Publisher dbg_can_pub = n.advertise<can_driver::wire_control>("wire_control",1000);
    can_driver::wire_control wire_control_msg;
    while(ros::ok())
    {
        if(dbg_can_count<500)
        {
            dbg_can_count++;
            if(dbg_can_count<250){
                wire_control_msg.brkMode = 0x00;
                wire_control_msg.brkReq = 0x01;
                wire_control_msg.trgBrkAcc = 1;//减速度1m/s
                wire_control_msg.strReq = 0x01;
                wire_control_msg.trgStrAngleOrTorque = 125 - dbg_can_count;
                wire_control_msg.pdeSwitch = 0x01;
                wire_control_msg.accPdeSingal2 = 1;//单位是V
                wire_control_msg.prsReq = 0x01;
                wire_control_msg.trgPressure = 1;
            }
            else {
                wire_control_msg.brkMode = 0x01;
                wire_control_msg.brkReq = 0x01;
                wire_control_msg.trgBrkAcc = 2;//减速度1m/s
                wire_control_msg.strReq = 0x01;
                wire_control_msg.trgStrAngleOrTorque = -375 + dbg_can_count;
                wire_control_msg.pdeSwitch = 0x01;
                wire_control_msg.accPdeSingal2 = 1;//单位是V
                wire_control_msg.prsReq = 0x01;
                wire_control_msg.trgPressure = 1;
            }
            dbg_can_pub.publish(wire_control_msg);
        }
    }

}