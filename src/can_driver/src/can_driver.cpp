#include "ros/ros.h"
#include "can_driver/can_includes.h" // can_card_library 

NTCAN_HANDLE  h0;
NTCAN_HANDLE  h1;
CMSG   CAN_net0_rxmsg[MAX_RX_MSG_CNT];
CMSG   CAN_net0_txmsg[MAX_TX_MSG_CNT];
int32_t CAN_net0_transmit_len = 0;
int32_t CAN_net0_receive_len = 0;
uint32_t CAN_transmit_count = 0;
uint32_t CAN_receive_count = 0;
uint32_t CAN_receive_len_all = 0;
CAN_ID386 CAN_ID386_obj;
CAN_ID31A CAN_ID31A_obj;
CAN_ID238 CAN_ID238_obj;
CAN_ID381 CAN_ID381_obj;
int can_start(){
    uint32_t CAN_Mode = NTCAN_MODE_MARK_INTERACTION;
    int net = 0;   
    int32_t i=0;   
    int txbufsize = MAX_TX_MSG_CNT;
    int rxbufsize = MAX_RX_MSG_CNT;
    int txtout = 10000;
    int rxtout = 10000;
    int idstart=0x000;
    int idend =0x7FF;
    NTCAN_RESULT ret;
    ret = canOpen(net,CAN_Mode,txbufsize,rxbufsize,txtout,rxtout,&h0);
    if (ret == NTCAN_SUCCESS)
    {
        ret = canSetBaudrate(h0,NTCAN_BAUD_500);
        for (i= idstart;i<idend;i++)
        {
         ret = canIdAdd(h0,i);   
        }        
    }
    return ret;
}

void wire_controlCallback(const can_driver::wire_control& msg)
{
    if (msg.brkMode == acc_brkMode)
    {
        CAN_ID238_obj.Compress(msg.activeCtrlMode,msg.brkReq,msg.trgBrkAcc,msg.strReq,0,msg.trgPressure,msg.trgStrAngleOrTorque);

    }
    else if(msg.brkMode == pre_brkMode)
    {
        CAN_ID238_obj.Compress(msg.activeCtrlMode,0,msg.trgBrkAcc,msg.strReq,msg.prsReq,msg.trgPressure,msg.trgStrAngleOrTorque);
    }
    CAN_ID31A_obj.Compress(msg.pdeSwitch,msg.accPdeSingal2);
    CAN_net0_txmsg[CAN_net0_transmit_len].id = CAN_ID238_obj.CAN_id;
    CAN_ID238_obj.ReadCANData(CAN_net0_txmsg[CAN_net0_transmit_len].data);
    ROS_INFO("CAN_ID%03X,%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X",CAN_net0_txmsg[CAN_net0_transmit_len].id,CAN_net0_txmsg[CAN_net0_transmit_len].data[0],CAN_net0_txmsg[CAN_net0_transmit_len].data[1],CAN_net0_txmsg[CAN_net0_transmit_len].data[2],CAN_net0_txmsg[CAN_net0_transmit_len].data[3],CAN_net0_txmsg[CAN_net0_transmit_len].data[4],CAN_net0_txmsg[CAN_net0_transmit_len].data[5],CAN_net0_txmsg[CAN_net0_transmit_len].data[6],CAN_net0_txmsg[CAN_net0_transmit_len].data[7]);
    CAN_net0_transmit_len++;
    CAN_net0_txmsg[CAN_net0_transmit_len].id = CAN_ID31A_obj.CAN_id;
    CAN_ID31A_obj.ReadCANData(CAN_net0_txmsg[CAN_net0_transmit_len].data);
    ROS_INFO("CAN_ID%03X,%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X",CAN_net0_txmsg[CAN_net0_transmit_len].id,CAN_net0_txmsg[CAN_net0_transmit_len].data[0],CAN_net0_txmsg[CAN_net0_transmit_len].data[1],CAN_net0_txmsg[CAN_net0_transmit_len].data[2],CAN_net0_txmsg[CAN_net0_transmit_len].data[3],CAN_net0_txmsg[CAN_net0_transmit_len].data[4],CAN_net0_txmsg[CAN_net0_transmit_len].data[5],CAN_net0_txmsg[CAN_net0_transmit_len].data[6],CAN_net0_txmsg[CAN_net0_transmit_len].data[7]);
    CAN_net0_transmit_len++;
}

// void can_transmitCallback(const can_driver::can_transmit&  msg)
// {
//     int i = 0; 
//     NTCAN_RESULT ret = 0;
//     txmsg[0].id = msg.frameId;
//     txmsg[0].len = 8;
//     for (i=0;i<txmsg[0].len;i++)
//     {
        
//         txmsg[0].data[i]= msg.data[i];
       
//     }
//     ret = canSend(h0,txmsg,&transmit_length);
//     if (ret == NTCAN_SUCCESS)
//     {
//         //ROS_INFO("I transimit :[%u]",msg.frameId);
//     }
//     else 
//     {
//         ROS_INFO("transimit error");
//     }
// }

int main(int argc, char** argv)
{

    uint8_t i = 0;
    uint8_t j = 0;
    NTCAN_RESULT CAN_ret = 0;  
    Status CAN_Status = stop;
    ros::init(argc,argv,"can_driver");   
    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<can_driver::can_receive>("can_receive",1000);

/********** start CAN ****************/
    CAN_ret = can_start();
    if (CAN_ret != NTCAN_SUCCESS)
    {
        ROS_INFO("ERROR");
    }
    else 
    {
        ROS_INFO("OK");
        CAN_Status = start;
    }
/********** end of start CAN ***************/
  if (CAN_Status == start)
        {
        ROS_INFO("start");
        ros::Subscriber sub = n.subscribe("wire_control",1000,wire_controlCallback);    // set topic
        ros::Rate loop_rate(1000);     // set loop rate
/********************  main loop  ************************/
        while (CAN_Status == start && ros::ok())
            {

/**************************************receive*********************************************/ 
            CAN_receive_count++;
            if (CAN_receive_count == CAN_RECEIVE_FREQ)
            {   
                CAN_receive_count = 0;
                CAN_net0_receive_len = MAX_RX_MSG_CNT;
                CAN_ret = canTake(h0, CAN_net0_rxmsg, &CAN_net0_receive_len);
                if (CAN_ret == NTCAN_SUCCESS){
                    if (CAN_net0_receive_len != 0 &&CAN_net0_receive_len < MAX_RX_MSG_CNT)
                    {
                        CAN_receive_len_all += CAN_net0_receive_len;
                        for(i=0;i<CAN_net0_receive_len;i++)
                        {
                            switch (CAN_net0_rxmsg[i].id)
                            {
                                case 0x386:{
                                        CAN_ID386_obj.GetCANData(CAN_net0_rxmsg[i].data);
                                        CAN_ID386_obj.Extract();
                                        ROS_INFO("Wheel Speed FL: %f",CAN_ID386_obj.wheelSpeed_FL);
                                    break;
                                }
                                case 0x381:{
                                        CAN_ID381_obj.GetCANData(CAN_net0_rxmsg[i].data);
                                        CAN_ID381_obj.Extract();
                                        ROS_INFO("steer Wheel Angle: %f",CAN_ID381_obj.steerWheelAngle);
                                }
                                default:break;

                            }
                    
                        }
                        CAN_net0_receive_len = 0;
                        ROS_INFO("%d",CAN_receive_len_all);                     
                    }
                }
                else{
                    ROS_INFO("can net0 receive error"); 
                }
            }
/*************************************transimit*********************************************/ 
            CAN_transmit_count++;
            if (CAN_transmit_count == CAN_TRANSMIT_FREQ)
            {   
                CAN_ret = canSend(h0,CAN_net0_txmsg,&CAN_net0_transmit_len);
                CAN_transmit_count = 0;
                if (CAN_ret != NTCAN_SUCCESS)
                {
                    ROS_INFO("can net0 transimit error");                
                }
            }
            ros::spinOnce();     
            loop_rate.sleep(); 
            }
        
        }
/********************  end of main loop  ***************************/

/********************  if start can error  ************************/        
    else{
            ROS_INFO("CAN net0 Something wrong");
            ros::spin();
    }
    return 0;

}






