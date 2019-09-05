#!/usr/bin/env python
#coding=utf-8 
'''test_listener ROS Node'''
# license removed for brevity
import rospy
import numpy as np
from cvxopt import solvers,matrix 
import csv
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64


# car parameter defination
class Car(object):
    def __init__(self):
        self.x = 40.0
        self.y = 28.25
        self.theta = 0.0    #rad
        self.thetaOld = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.steeringWheelAngle = 0.0
        self.speed = 15.0
        self.wz = 0.0


class SimCarModel(Car):
    def __init__(self):
        super(SimCarModel,self).__init__()
        self.mass  = 2220.0
        self.cf = 90943.0
        self.cr = 148091.0
        self.lf = 1.135
        self.lr = 1.685
        self.Iz = 4193.5
        self.steering_ratio = 20  


car_obj = SimCarModel() #init car object


# Trajectory parameter defination
class Trajectory(object):
    def __init__(self,filePath):
        global car_obj 
        self.x = []
        self.y = []
        self.speed = []
        self.num = []
        # for py36        
        csvfile = open(filePath,'rt',encoding="utf-8") 
        # # for py27
        # csvfile = file(filePath,'rb')
        reader = csv.reader(csvfile)
        for line in reader:                 
            self.num.append(int(line[0]))
            self.x.append(float(line[1]))
            self.y.append(float(line[2]))
            self.speed.append(float(line[4]))        
        csvfile.close()


# shot model 
def FindClosestPoint(x,y,trajectory):
    dist = 100000.0
    temp = 0.0
    mark = 0 
    for i in range(len(trajectory.num)):
        temp = (trajectory.x[i] - x)*(trajectory.x[i] - x)+(trajectory.y[i] - y)*(trajectory.y[i] - y)
        if temp < dist:
            dist = temp
            mark = i
    return mark


def GetAngle(x1,y1,x2,y2):
    x = x1 - x2
    y = y1 - y2
    angle = -1.0
    if y == 0 and x > 0 :
        angle = 0.0
    if y == 0 and x < 0 :
        angle = 3.1416
    if x == 0 and y > 0 :
        angle = 0.5*3.1416
    if x == 0 and y < 0 :
        angle = 1.5*3.1416
    if angle == -1:
        if x > 0:
            angle = math.atan(y/x) 
        if x < 0:
            angle = math.atan(y/x) + 3.1416 
    if angle < 0:
        angle += 2*3.1416
    
    return angle


def CalSteerAngle(mark,x,y,Heading,trajectory):
    windowSize = 10
    point = windowSize + mark
    KP = 25
    if point >= len(trajectory.num):
        point = point - len(trajectory.num)
    target_x = trajectory.x[point]
    target_y = trajectory.y[point]
    angleDist = GetAngle(target_x,target_y,x,y) - Heading
    if angleDist >= 3.1416:
        angleDist = angleDist - 2 * 3.1416   
    if angleDist < -3.1416:
        angleDist = angleDist + 2 * 3.1416
    steerAngleOut = KP * angleDist
    return steerAngleOut


def CalThrottleOut(mark,speed,trajectory):
    KP = 10
    throttleOut = KP * (trajectory.speed[mark]-speed)
    if throttleOut > 50:
        throttleOut = 50
    if throttleOut < 0:
        throttleOut = 0
    return throttleOut


def CalControlOut(trajectory):
    global car_obj 
    mark = FindClosestPoint(car_obj.x,car_obj.y,trajectory)    
    car_obj.throttle = CalThrottleOut(mark,car_obj.speed,trajectory)
    car_obj.steeringWheelAngle = CalSteerAngle(mark,car_obj.x,car_obj.y,car_obj.theta,trajectory)


#MPC  model
class MPC_lateral(object):
    def __init__(self,car):
        self.error = np.zeros((4,1))           
        self.error_prediction = []
        self.error_record = []
        self.horizon = 5
        self.timestamp = 0.05
        self.matrix_a = np.zeros((4,4))
        self.matrix_ad = np.zeros((4,4))
        self.matrix_b = np.zeros((4,1))
        self.matrix_bd = np.zeros((4,1))
        self.matrix_c = np.zeros((4,1))
        self.headingrate = 0 #参考轨迹航向角的变化率
        self.matrix_headingrate = np.zeros((20,1)) #参考轨迹航向角的变化率
        self.matrix_ref = np.zeros((20,1)) #参考的误差规律
        self.matrix_ctr = np.zeros((5,1))
        self.matrix_q = np.array([[20,0,0,0],[0,1,0,0],[0,0,5,0],[0,0,0,1]])
        self.matrix_r = 30
        self.matrix_bb = np.zeros((20,5))
        self.matrix_qq = np.zeros((20,20))
        self.matrix_cc = np.zeros((20,20))
        self.matrix_rr = self.matrix_r*np.eye(5)
        self.matrix_aa = np.zeros((20,20))
        self.matrix_initial_state = np.zeros((20,1))
    def Get_model(self):
        global car_obj        
        if car_obj.speed <= 0.2:
            speed = 0.2
        else:
            speed = car_obj.speed
        a22 = -(car_obj.cf + car_obj.cr) / (car_obj.mass * speed) 
        a23 = (car_obj.cf + car_obj.cr) / (car_obj.mass)         
        a24 = (-car_obj.cf * car_obj.lf + car_obj.cr * car_obj.lr) / (car_obj.mass * speed)
        a42 = (-car_obj.cf * car_obj.lf + car_obj.cr * car_obj.lr) / (car_obj.Iz * speed)
        a43 = (car_obj.cf * car_obj.lf - car_obj.cr * car_obj.lr) / (car_obj.Iz)
        a44 = -(car_obj.cf * car_obj.lf *car_obj.lf + car_obj.cr * car_obj.lr * car_obj.lr) /(car_obj.Iz * speed)
        self.matrix_a = np.array([[0,1,0,0],[0,a22,a23,a24],[0,0,0,1],[0,a42,a43,a44]],dtype = float)               
        b2 = car_obj.cf/car_obj.mass
        b4 = car_obj.cf * car_obj.lf / (car_obj.Iz)
        self.matrix_b = np.array([[0],[b2],[0],[b4]],dtype = float)          
        c2 =  (-car_obj.cf * car_obj.lf + car_obj.cr * car_obj.lr) / (car_obj.mass * speed) - speed
        c4 = -(car_obj.cf * car_obj.lf *car_obj.lf + car_obj.cr * car_obj.lr * car_obj.lr) /(car_obj.Iz * speed)
        self.matrix_c = np.array([[0],[c2],[0],[c4]],dtype = float)                
    def Get_d_model(self,mark,trajectory):
        global car_obj      
        self.matrix_ad = (np.eye(4) + self.timestamp * self.matrix_a) 
        self.matrix_bd = self.timestamp * self.matrix_b
        target_mark1 = mark+6
        target_mark2 = mark+3
        if target_mark1 >= len(trajectory.x):
            target_mark1 = target_mark1 - len(trajectory.x)
        if target_mark2 >= len(trajectory.x):
            target_mark2 = target_mark2 - len(trajectory.x) 
        heading = GetAngle(trajectory.x[target_mark1],trajectory.y[target_mark1],trajectory.x[mark],trajectory.y[mark])          
        headingrate = (GetAngle(trajectory.x[target_mark1],trajectory.y[target_mark1],trajectory.x[target_mark2],trajectory.y[target_mark2])-GetAngle(trajectory.x[target_mark2],trajectory.y[target_mark2],trajectory.x[mark],trajectory.y[mark]))      
        error1 = -(car_obj.x - trajectory.x[mark])*math.sin(heading) + (car_obj.y- trajectory.y[mark])*math.cos(heading)  
        error3 = (car_obj.theta - heading)
        if error3 > 3.1416 :
            error3 = error3 - 3.1416*2
        if error3 <-3.1416 :
            error3 = error3 + 3.1416*2  
        error2 = car_obj.speed * math.sin(error3)  
        if headingrate > 3.1416:
            headingrate = headingrate - 3.1416*2
        if headingrate < -3.1416:            
            headingrate = headingrate + 3.1416*2
        self.headingrate = headingrate * car_obj.speed / 2.25        
        error4 = car_obj.wz - self.headingrate 
        self.error = np.array([[error1],[error2],[error3],[error4]]) 
        #print(self.error)
        self.matrix_headingrate = np.r_[self.matrix_c*self.timestamp*self.headingrate,self.matrix_c*self.timestamp*self.headingrate,self.matrix_c*self.timestamp*self.headingrate,self.matrix_c*self.timestamp*self.headingrate,self.matrix_c*self.timestamp*self.headingrate]
    def Get_matrix_bb(self):
        zero = np.zeros((4,1))
        a11 = np.dot(self.matrix_ad,self.matrix_bd)
        a21 = np.dot(self.matrix_ad,a11)
        a31 = np.dot(self.matrix_ad,a21)
        a41 = np.dot(self.matrix_ad,a31)
        a51 = np.dot(self.matrix_ad,a41)        
        c1 = np.c_[a11,zero,zero,zero,zero]
        c2 = np.c_[a21,a11,zero,zero,zero]
        c3 = np.c_[a31,a21,a11,zero,zero]
        c4 = np.c_[a41,a31,a21,a11,zero]
        c5 = np.c_[a51,a41,a31,a21,a11]
        self.matrix_bb = np.r_[c1,c2,c3,c4,c5]
    def Get_matrix_qq(self):
        zero = np.zeros((4,4))
        c1 = np.c_[self.matrix_q,zero,zero,zero,zero]
        c2 = np.c_[zero,self.matrix_q,zero,zero,zero]
        c3 = np.c_[zero,zero,self.matrix_q,zero,zero]
        c4 = np.c_[zero,zero,zero,self.matrix_q,zero]
        c5 = np.c_[zero,zero,zero,zero,self.matrix_q]
        self.matrix_qq = np.r_[c1,c2,c3,c4,c5]
    def Get_matrix_cc(self):
        zero = np.zeros((4,4))
        a11 = np.eye(4)
        a21 = np.dot(self.matrix_ad,a11)
        a31 = np.dot(self.matrix_ad,a21)
        a41 = np.dot(self.matrix_ad,a31)
        a51 = np.dot(self.matrix_ad,a41)        
        c1 = np.c_[a11,zero,zero,zero,zero]
        c2 = np.c_[a21,a11,zero,zero,zero]
        c3 = np.c_[a31,a21,a11,zero,zero]
        c4 = np.c_[a41,a31,a21,a11,zero]
        c5 = np.c_[a51,a41,a31,a21,a11]
        self.matrix_cc = np.r_[c1,c2,c3,c4,c5]

    def Get_matrix_aa(self):
        zero = np.zeros((4,4))
        a11 = self.matrix_ad
        a22 = np.dot(self.matrix_ad,a11)
        a33 = np.dot(self.matrix_ad,a22)
        a44 = np.dot(self.matrix_ad,a33)
        a55 = np.dot(self.matrix_ad,a44)        
        c1 = np.c_[a11,zero,zero,zero,zero]
        c2 = np.c_[zero,a22,zero,zero,zero]
        c3 = np.c_[zero,zero,a33,zero,zero]
        c4 = np.c_[zero,zero,zero,a44,zero]
        c5 = np.c_[zero,zero,zero,zero,a55]
        self.matrix_aa = np.r_[c1,c2,c3,c4,c5]

    def Get_matrix_initial_state(self):
        self.matrix_initial_state = np.r_[self.error,self.error,self.error,self.error,self.error]

    def Get_matrix_rr(self):
        self.matrix_rr = self.matrix_r*np.eye(5)

    def Cal(self,trajectory):
        global car_obj
        mark = FindClosestPoint(car_obj.x,car_obj.y,trajectory)
        #print(mark)
        self.Get_model()
        self.Get_d_model(mark,trajectory)
        self.Get_matrix_bb()        
        self.Get_matrix_qq()
        self.Get_matrix_cc()
        self.Get_matrix_aa()
        self.Get_matrix_rr()
        self.Get_matrix_initial_state()
        Ht = np.dot(np.dot(np.transpose(self.matrix_bb),self.matrix_qq),self.matrix_bb) + self.matrix_rr
        M = np.dot(self.matrix_aa,self.matrix_initial_state) + np.dot(self.matrix_cc,self.matrix_headingrate)               
        Gt = np.dot(np.dot(np.transpose(self.matrix_bb),self.matrix_qq),M-self.matrix_ref)       
        ''' 
        print(self.matrix_initial_state)
        print(Ht)
        print(Gt)
        '''
        sol = solvers.qp(2*matrix(Ht),matrix(Gt))       
        car_obj.steeringWheelAngle = np.array(sol['x'])[0] * car_obj.steering_ratio 
        if car_obj.steeringWheelAngle >= 3*3.1416:
            car_obj.steeringWheelAngle = 3*3.1416
        if car_obj.steeringWheelAngle <= -3*3.1416:
            car_obj.steeringWheelAngle = -3*3.1416
        #print(np.dot(self.matrix_bb,u)+np.dot(self.matrix_aa,self.matrix_initial_state)+np.dot(self.matrix_cc,self.matrix_headingrate))
        
# callback  
def sim_Pose2DCallback(data):
    global car_obj
    car_obj.x = data.x
    car_obj.y = data.y
    car_obj.theta = data.theta

def sim_SpeedCallback(data):
    global car_obj
    car_obj.speed = data.data

def sim_wzCallback(data):
    global car_obj
    car_obj.wz = data.data


# main function
def trajectory_control():
    rospy.init_node('MPC_trajectory_control')
    pub = rospy.Publisher('sim_Control', Point, queue_size=100)
    pub2 = rospy.Publisher('sim_data', Point, queue_size=100)
    controlData_msg = Point()    #创建控制消息
    simData_msg = Point()
    rate = rospy.Rate(50)
    trajectory = Trajectory('/home/zhaojt/ROS/test.csv')
    rospy.Subscriber("sim_Pose2D", Pose2D, sim_Pose2DCallback)
    rospy.Subscriber("sim_Speed", Float64, sim_SpeedCallback)
    rospy.Subscriber("sim_wz", Float64, sim_wzCallback)
    #创建控制器
    MPC_lateral_obj = MPC_lateral(car_obj)
    
    ''' test 
    a = np.array([[1,2],[3,4]])
    c = np.c_[a,a]    
    print(a)
    print(c)
    a[1][0] = 1
    print(a)   
    a = matrix(a)
    a[2]=5
    print(a)
    a = np.array(a)
    a = np.dot(a,a)
    print(a)
    '''
    while not rospy.is_shutdown():        
        '''
        CalControlOut(trajectory)
        '''
        MPC_lateral_obj.Cal(trajectory)
        
        controlData_msg.x = car_obj.steeringWheelAngle
        controlData_msg.y = car_obj.throttle
        controlData_msg.z = car_obj.brake 
        simData_msg.x = MPC_lateral_obj.error[1]
        simData_msg.y = MPC_lateral_obj.error[2]
        simData_msg.z = MPC_lateral_obj.error[3]
        pub2.publish(simData_msg)
        pub.publish(controlData_msg)
        rate.sleep()


if __name__ == '__main__':
    trajectory_control()


