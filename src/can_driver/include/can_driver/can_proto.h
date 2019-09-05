#pragma once
#ifndef _CAN_PROTO_H_
#define _CAN_PROTO_H_

#include "ros/ros.h"


/****************ID238_Ratio**************/
#define trgBrkAcc_Ratio 0.1
#define trgStrAngleOrTorque_Ratio 0.1
#define trgPressure_Ratio 0.01

/****************ID31A_Ratio**************/
#define  accPdeSingal2_Ratio 0.02
#define  accPdeSingal2_limit 1
/****************ID386_Ratio*************/
#define wheelSpeed_Ratio 0.3137

/****************ID381_Ratio**************/
#define steerWheelAngle_Ratio 0.1
#define steerWheelAngle_limit 540

/****************ID111_Ratio**************/
#define gear_P 0x00
#define gear_R 0x07
#define gear_N 0x06
#define gear_D 0x05

/****************ID2B0_Ratio**************/
#define steerWheelSpeed_Ratio 0.01

/****************ID153_Ratio*************/
#define torqueReduceFast_Ratio 1
#define torqueReduceSlow_Ratio 1

class CAN_PROTO{
    
    public:
    uint32_t timeStamp;
    uint32_t CAN_id;
    uint8_t CAN_data[8];
    uint8_t RemoteFlag;
    uint8_t DataLen;//是否是远程帧
    uint8_t ExternFlag;//是否是扩展帧
    uint8_t SendType;

    void ReadCANData(uint8_t* data);
    void GetCANData(uint8_t* data);
};



class CAN_ID238 : public CAN_PROTO{
    
    public: 
        
    uint8_t activeCtrlMode;
    uint8_t brkReq;
    float trgBrkAcc;
    uint8_t strReq;
    uint8_t prsReq;
    float trgPressure;
    float trgStrAngleOrTorque;

    void Compress(uint8_t data_activeCtrlMode ,uint8_t data_brkReq ,float data_trgBrkAcc, 
                        uint8_t data_strReq,uint8_t data_prsReq,float data_trgPressure,float data_trgStrAngleOrTorque);  
};

class CAN_ID31A : public CAN_PROTO{
    
    public: 
    
    uint8_t pdeSwitch;
    float accPdeSingal2;

    void Compress(uint8_t data_pdeSwitch,float data_accPdeSingal2);

};

class CAN_ID386 : public CAN_PROTO{

    public: 

    float wheelSpeed_FL;
    float wheelSpeed_FR;
    float wheelSpeed_RL;
    float wheelSpeed_RR;

    void Extract(void);

};

class CAN_ID381 : public CAN_PROTO{

    public: 

    float steerWheelAngle;

    void Extract(void);

};

class CAN_ID541 : public CAN_PROTO{

    public:

    uint8_t brakeFlag;

    void Extract(void);
};

class CAN_ID111 : public CAN_PROTO{

    public: 

    uint8_t gear;
    uint8_t gearShiftFlag;

    void Extract(void);


};

class CAN_ID2B0 : public CAN_PROTO{

    public: 

    float steerWheelSpeed;

    void Extract(void);

};

class CAN_ID153 : public CAN_PROTO{

    public:

    float torqueReduceFast;
    float torqueReduceSlow;

    void Compress(float data_torqueReduceFast,float data_torqueReduceSlow);
    void Extract(void);



};


#endif 







