#include "ros/ros.h"
#include "can_driver/wire_control.h"

int32_t dbg_can_count = 0;

int main(int argc,char **argv)
{

    ros::init(argc,argv,"dbg_can");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);
    ros::Publisher dbg_can_pub = n.advertise<can_driver::wire_control>("wire_control",1000);
    can_driver::wire_control wire_control_msg;
    dbg_can_count = 0;
    while(ros::ok())
    {
        if(dbg_can_count<500)
        {
            dbg_can_count++;
            if(dbg_can_count<250){
                /************制动**********/
                wire_control_msg.brkMode = 0x01;
                wire_control_msg.brkReq = 0x00;
                wire_control_msg.trgBrkAcc = 0;//减速度1m/s
                wire_control_msg.prsReq = 0x00;
                wire_control_msg.trgPressure = 0;
                /*********转向*************/
                wire_control_msg.strReq = 0x01;
                wire_control_msg.trgStrAngleOrTorque = (float)(dbg_can_count)/2;
                /*********油门************/
                wire_control_msg.pdeSwitch = 0x00;
                wire_control_msg.accPdeSingal2 = 0;//单位是V
                /*********工作模式*********/
                wire_control_msg.activeCtrlMode = 1;
                ROS_INFO("%f", wire_control_msg.trgStrAngleOrTorque);

            }
            else {
                /************制动**********/
                wire_control_msg.brkMode = 0x01;
                wire_control_msg.brkReq = 0x00;
                wire_control_msg.trgBrkAcc = 0;//减速度1m/s
                wire_control_msg.prsReq = 0x00;
                wire_control_msg.trgPressure = 0;
                /*********转向*************/
                wire_control_msg.strReq = 0x01;
                wire_control_msg.trgStrAngleOrTorque =250-(float)(dbg_can_count)/2;
                /*********油门************/
                wire_control_msg.pdeSwitch = 0x01;
                wire_control_msg.accPdeSingal2 = 0.5;//单位是V
                /*********工作模式*********/
                wire_control_msg.activeCtrlMode = 1;
                ROS_INFO("%f", wire_control_msg.trgStrAngleOrTorque);
            }
        dbg_can_pub.publish(wire_control_msg);
        }
        else if(dbg_can_count == 500)
        {
            dbg_can_count++;
            /************制动**********/
            wire_control_msg.brkMode = 0x01;
            wire_control_msg.brkReq = 0x00;
            wire_control_msg.trgBrkAcc = 0;//减速度1m/s
            wire_control_msg.prsReq = 0x00;
            wire_control_msg.trgPressure = 0;
            /*********转向*************/
            wire_control_msg.strReq = 0x01;
            wire_control_msg.trgStrAngleOrTorque = 0;
            /*********油门************/
            wire_control_msg.pdeSwitch = 0x00;
            wire_control_msg.accPdeSingal2 = 0;//单位是V
            /*********工作模式*********/
            wire_control_msg.activeCtrlMode = 0;
            dbg_can_pub.publish(wire_control_msg);
        }
        loop_rate.sleep();
    }

}