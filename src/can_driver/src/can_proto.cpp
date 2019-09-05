
#include "can_driver/can_proto.h"


void CAN_PROTO::ReadCANData(uint8_t* data)
{
    int i;
    for (i=0;i<8;i++)
    {
        data[i] = CAN_data[i];
    }
}

void CAN_PROTO::GetCANData(uint8_t* data)
{
    int i;
    for (i=0;i<8;i++)
    {
        CAN_data[i] = data[i];
    }
}

void CAN_ID238::Compress(uint8_t data_activeCtrlMode ,uint8_t data_brkReq ,float data_trgBrkAcc, 
                        uint8_t data_strReq,uint8_t data_prsReq,float data_trgPressure,float data_trgStrAngleOrTorque)
{
    int i = 0;
    float ftemp = 0;
    uint16_t u16temp = 0;
    CAN_id = 0x238; 
    activeCtrlMode = data_activeCtrlMode;
    brkReq = data_brkReq;
    trgBrkAcc = data_trgBrkAcc;
    strReq = data_strReq;
    prsReq = data_prsReq;
    trgPressure = data_trgPressure;
    trgStrAngleOrTorque = data_trgStrAngleOrTorque;
    for(i=0; i<8; i++) { CAN_data[i] = 0;}
    CAN_data[0] |= prsReq;    
    CAN_data[0] |= (brkReq << 3);
    CAN_data[0] |= (activeCtrlMode << 6);
    CAN_data[5] = (uint8_t)(trgBrkAcc / trgBrkAcc_Ratio);
    if (trgStrAngleOrTorque >= -steerWheelAngle_limit && trgStrAngleOrTorque <= steerWheelAngle_limit)
    {
    CAN_data[0] |= (strReq << 1);
    ftemp = trgStrAngleOrTorque / trgStrAngleOrTorque_Ratio + 0x8000;
    u16temp = (uint16_t)(ftemp);
    CAN_data[6] = (uint8_t)((u16temp >> 8) & 0xFF);
    CAN_data[7] = (uint8_t)(u16temp & 0xFF);
    }
    else {
    CAN_data[0] |= (0x00 << 1);
    ROS_INFO("ID238: data is out of limit");
    }
    CAN_data[3] = (uint8_t)((uint16_t)(trgPressure / trgPressure_Ratio) & 0xFF);
    CAN_data[2] = ((uint8_t)(((uint16_t)(trgPressure / trgPressure_Ratio)) >> 8)) & 0x03;

}

void CAN_ID31A::Compress(uint8_t data_pdeSwitch,float data_accPdeSingal2)
{
    int i;
    accPdeSingal2 = data_accPdeSingal2;
    pdeSwitch = data_pdeSwitch;
    CAN_id = 0x31A;
    for(i=0;i<8;i++) {CAN_data[i] = 0;}
    if (pdeSwitch >= -accPdeSingal2_limit && pdeSwitch <= accPdeSingal2_limit)
    {
    CAN_data[0] |= data_pdeSwitch;
    CAN_data[1] = (uint8_t)(data_accPdeSingal2/accPdeSingal2_Ratio);
    }
    else {
    CAN_data[0] |= 0x00; 
    ROS_INFO("ID31A: data is out of limit");
    }
}

void CAN_ID386::Extract(void)
{
    wheelSpeed_FL = CAN_data[0] * wheelSpeed_Ratio;
    wheelSpeed_FR = CAN_data[2] * wheelSpeed_Ratio;
    wheelSpeed_RL = CAN_data[4] * wheelSpeed_Ratio;
    wheelSpeed_RR = CAN_data[6] * wheelSpeed_Ratio;
}

void CAN_ID381::Extract(void)
{   
    if (CAN_data[4] < 0x80)
    {
        steerWheelAngle = (CAN_data[4] << 8 + CAN_data[3])*steerWheelAngle_Ratio;
    }
    else if (CAN_data[4] > 0x80)
    {
        steerWheelAngle = ((CAN_data[4] << 8 + CAN_data[3]) - 256*256)*steerWheelAngle_Ratio;
    }
}


void CAN_ID541::Extract(void)
{
    brakeFlag = (CAN_data[7] >> 4) & 0x01;
} 

void CAN_ID111::Extract(void) 
{
    gear = CAN_data[1] & 0x0F;
    gearShiftFlag = (CAN_data[2]>>3) & 0x01;
}

void CAN_ID2B0::Extract(void)
{
    steerWheelSpeed = CAN_data[2];
}

void CAN_ID153::Compress(float data_torqueReduceFast,float data_torqueReduceSlow)
{    
    torqueReduceFast = data_torqueReduceFast;
    torqueReduceSlow = data_torqueReduceSlow;
    CAN_data[2] = (uint8_t)(torqueReduceFast/torqueReduceFast_Ratio);
    CAN_data[4] = (uint8_t)(torqueReduceSlow/torqueReduceSlow_Ratio);
}

void CAN_ID153::Extract(void)
{
    torqueReduceFast = CAN_data[2] * torqueReduceFast_Ratio;
    torqueReduceSlow = CAN_data[4] * torqueReduceSlow_Ratio;
}
