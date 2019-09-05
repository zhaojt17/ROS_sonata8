/******************** 
author: zhaojt

命名规则：（其实就是拍脑袋习惯）
function like: FunctionFirst
data like: dataFirst ID238_dataFirst
长意的连词符：　_
 
佛祖保佑～永无bug

*******************/

#ifndef _CAN_INCLUDE_H_
#define _CAN_INCLUDE_H_

#include "can_driver/ntcan.h"
#include "can_driver/can_proto.h"
#include "can_driver/wire_control.h"


/***********pcie esd-can 402  definations start *****************/

#define MAX_RX_MSG_CNT 256
#define MAX_TX_MSG_CNT 256
#define CAN_RECEIVE_FREQ 10
#define CAN_TRANSMIT_FREQ 10
#define acc_brkMode 0x00
#define pre_brkMode 0x01

enum Status {start = 0 , stop = 1 ,error = 2};

/***********pcie esd-can 402  definations end *****************/



/***********pcie esd-can 402  funtion  start *****************/

//static int8_t* get_error_str(int8_t *str_buf, NTCAN_RESULT ntstatus);


/***********pcie esd-can 402  funtion  end *****************/




#endif 