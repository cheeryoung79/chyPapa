#!/usr/bin/env python
# coding: utf-8

# In[1]:


import math
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import datetime
import numpy as np


# In[2]:


#给定足坐标系返回舵机角度
def xyz(x,y,z):
    h=0.15
    hu=0.35
    hl=0.382
    dyz=math.sqrt(y**2+z**2)
    lyz=math.sqrt(dyz**2-h**2)
    gamma_yz=-math.atan(y/z)
    gamma_h_offset=-math.atan(h/lyz)
    gamma=gamma_yz-gamma_h_offset
    
    lxzp=math.sqrt(lyz**2+x**2)
    n=(lxzp**2-hl**2-hu**2)/(2*hu)
    beta=-math.acos(n/hl)

    
    alfa_xzp=-math.atan(x/lyz)
    alfa_off=math.acos((hu+n)/lxzp)
    alfa=alfa_xzp+alfa_off
    
    return gamma,alfa,beta

    #输出角度为弧度


# In[4]:


#trot步态
def gait_plan1(t,T):
    Ts=T/2  #周期为0.2s
    xs=-0.1 #起点x位置
    xf=0.1  #终点x位置
    zs=-0.482 #z起点位置
    h=0.1   #抬腿高度
    pi=3.14
    if t<=Ts:
        sigma=2*pi*t/Ts  
        x=(xf-xs)*((sigma-math.sin(sigma))/(2*pi))+xs
        z=h*(1-math.cos(sigma))/2+zs
    else:
        x=(-(xf-xs)/(T-Ts))*(t-Ts)+xf
        z=-0.482
    return x,z


# In[5]:


#walk步态
def gait_plan2(t,T):
    Ts=T/4  #周期为0.25s
    xs=-0.1 #起点x位置
    xf=0.1  #终点x位置
    zs=-0.482 #z起点位置
    h=0.15   #抬腿高度
    pi=3.14
    if t<=Ts:  
        sigma=2*pi*t/Ts  
        x=(xf-xs)*((sigma-math.sin(sigma))/(2*pi))+xs
        z=h*(1-math.cos(sigma))/2+zs
    else:
        x=(-(xf-xs)/(T-Ts))*(t-Ts)+xf
        z=-0.482

    return x,z


# In[8]:


#四足机器人站立姿态解算

def pose_control(row,pitch,yaw,pos_x,pos_y,pos_z):
    pi=3.14
    b=0.4
    l=0.8
    w=0.7
    h=0.732
    R=row*pi/180
    P=pitch*pi/180
    Y=yaw*pi/180
    pos= np.mat([pos_x,pos_y,pos_z]).T
    #pos = np.mat([0.0,  0.0,  0.3 ]).T
    rotx=np.mat([[ 1,        0,         0      ],
       [ 0,        math.cos(R),   -math.sin(R) ],
       [ 0,        math.sin(R),    math.cos(R) ]])
    roty=np.mat([[ math.cos(P),   0,        -math.sin(P) ],
           [ 0,        1,         0      ],
           [ math.sin(P),   0,         math.cos(P) ]])
    rotz=np.mat([[ math.cos(Y),  -math.sin(Y),    0      ],
           [ math.sin(Y),   math.cos(Y),    0      ],
           [ 0,        0,         1      ]])
    rot_mat = rotx * roty * rotz
    #结构参数
    body_struct = np.mat([[ l / 2,  -b / 2,  h],
                 [  l / 2, b / 2,   h],
                 [ -l / 2,  b / 2,    h],
                 [ -l / 2, -b / 2,   h]]).T

    footpoint_struct = np.mat([[ l/2,   -w/2,  0],
                     [  l/2,   w/2,  0],
                     [  -l/2,    w/2,  0],
                     [  -l/2,    -w/2,  0]]).T
    leg_pose = np.mat(np.zeros((3, 4)))
                      
    for i in range(4):
        leg_pose[:,i] = - pos - rot_mat * body_struct[:, i] + footpoint_struct[:, i]
                      
    rb_x = leg_pose[0, 0]
    rb_y = -leg_pose[1, 0]
    rb_z = -leg_pose[2, 0]
    rf_x = leg_pose[0, 1]
    rf_y = leg_pose[1, 1]
    rf_z = -leg_pose[2, 1]
    lb_x = leg_pose[0, 2]
    lb_y = leg_pose[1, 2]
    lb_z = -leg_pose[2, 2]
    lf_x = leg_pose[0, 3]
    lf_y = -leg_pose[1, 3]
    lf_z = -leg_pose[2, 3]

    return rb_x,rb_y,rb_z,rf_x,rf_y,rf_z,lb_x,lb_y,lb_z,lf_x,lf_y,lf_z


# In[9]:


pi=3.14

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
    sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot); #开启仿真
    #声明节点，关节初始化，12个关节
    [rec ,rb_rot_3]=sim.simxGetObjectHandle (clientID,'rb_rot_3',sim.simx_opmode_blocking)
    [rec ,rf_rot_3]=sim.simxGetObjectHandle (clientID,'rf_rot_3',sim.simx_opmode_blocking)
    [rec ,rb_rot_2]=sim.simxGetObjectHandle (clientID,'rb_rot_2',sim.simx_opmode_blocking)
    [rec ,rf_rot_2]=sim.simxGetObjectHandle (clientID,'rf_rot_2',sim.simx_opmode_blocking)
    [rec ,rb_rot_1]=sim.simxGetObjectHandle (clientID,'rb_rot_1',sim.simx_opmode_blocking)
    [rec ,rf_rot_1]=sim.simxGetObjectHandle (clientID,'rf_rot_1',sim.simx_opmode_blocking)
    [rec ,lb_rot_3]=sim.simxGetObjectHandle (clientID,'lb_rot_3',sim.simx_opmode_blocking)
    [rec ,lf_rot_3]=sim.simxGetObjectHandle (clientID,'lf_rot_3',sim.simx_opmode_blocking)
    [rec ,lb_rot_2]=sim.simxGetObjectHandle (clientID,'lb_rot_2',sim.simx_opmode_blocking)
    [rec ,lf_rot_2]=sim.simxGetObjectHandle (clientID,'lf_rot_2',sim.simx_opmode_blocking)
    [rec ,lb_rot_1]=sim.simxGetObjectHandle (clientID,'lb_rot_1',sim.simx_opmode_blocking)
    [rec ,lf_rot_1]=sim.simxGetObjectHandle (clientID,'lf_rot_1',sim.simx_opmode_blocking)
    #2个电机力矩参数
    rb_rot_1_force=500
    rb_rot_2_force=500
    rb_rot_3_force=500 
    #第一条腿
    rf_rot_1_force=500     
    rf_rot_2_force=500         
    rf_rot_3_force=500 
    #第二条腿
    lb_rot_1_force=500     
    lb_rot_2_force=500         
    lb_rot_3_force=500 
    #第三条腿   
    lf_rot_1_force=500     
    lf_rot_2_force=500         
    lf_rot_3_force=500 
    #第四条腿
    
    #12个电机角度参数
    rb_rot_1_pos=0
    rb_rot_2_pos=0  
    rb_rot_3_pos=0  
    #第一条腿 
    rf_rot_1_pos=0  
    rf_rot_2_pos=0  
    rf_rot_3_pos=0  
    #第二条腿
    lb_rot_1_pos=0  
    lb_rot_2_pos=0  
    lb_rot_3_pos=0  
    #第三条腿
    lf_rot_1_pos=0  
    lf_rot_2_pos=0  
    lf_rot_3_pos=0  
    #第四条腿

    #置电机力矩
    rec=sim.simxSetJointForce(clientID,rb_rot_3, rb_rot_3_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,rf_rot_3, rf_rot_3_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,rb_rot_2, rb_rot_2_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,rf_rot_2, rf_rot_2_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,rb_rot_1, rb_rot_1_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,rf_rot_1, rf_rot_1_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,lb_rot_3, lb_rot_3_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,lf_rot_3, lf_rot_3_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,lb_rot_2, lb_rot_2_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,lf_rot_2, lf_rot_2_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,lb_rot_1, lb_rot_1_force,sim.simx_opmode_blocking)
    rec=sim.simxSetJointForce(clientID,lf_rot_1, lf_rot_1_force,sim.simx_opmode_blocking)
    
    #    row=0;   pitch=0; yaw=0;
    #    pos_x=0; pos_y=0.2; pos_z=-0.2;
    time.sleep(1)   #延时1s
    #t=datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')   #获取系统当前时间 
    t=datetime.datetime.now()
    #startTime=t(5)*60+t(6); #当前时间 [年 月 日 时 分 秒]
    startTime=t
    currentTime=datetime.datetime.now()-datetime.datetime.now() #当前时间
    gait_state=2  #步态标志位
    #  [lb_x,lb_y,lb_z,rb_x,rb_y,rb_z,rf_x,rf_y,rf_z,lf_x,lf_y,lf_z] = pose_control(row,pitch,yaw,pos_x,pos_y,pos_z);
    #  [lb_rot_1_pos,lb_rot_2_pos,lb_rot_3_pos]=xyz(lb_x,lb_y,lb_z);
    #[lf_rot_1_pos,lf_rot_2_pos,lf_rot_3_pos]=xyz(lf_x,lf_y,lf_z);
    #[rb_rot_1_pos,rb_rot_2_pos,rb_rot_3_pos]=xyz(rb_x,rb_y,rb_z);
    #[rf_rot_1_pos,rf_rot_2_pos,rf_rot_3_pos]=xyz(rf_x,rf_y,rf_z);
    while currentTime.seconds < 100:
        #t=datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')   #获取系统当前时间 
        t=datetime.datetime.now()
        currentTime=t-startTime #当前时间，从进入主循环开始 
        if currentTime.seconds < 5:
            if gait_state==2:   #walk步态
                lb_x= -0.1
                rb_x= -0.1
                lf_x=0.1     
                rf_x=0.1
                lb_z=-0.482
                rf_z=-0.482
                lf_z=-0.482
                rb_z=-0.482
       
            if gait_state==1:   #trot步态
                lb_x= -0.1
                rb_x= 0.1
                lf_x=0.1
                rf_x=-0.1
                lb_z=-0.482
                rf_z=-0.482
                lf_z=-0.482
                rb_z=-0.482
       
            [rec,vrep_time]=sim.simxGetFloatSignal(clientID,'time',sim.simx_opmode_oneshot) #获取仿真时间
        else:
            [rec,vrep_realtime]=sim.simxGetFloatSignal(clientID,'time',sim.simx_opmode_oneshot) #获取仿真时间
            if gait_state==2: #慢步步态
                T=1
                time1=vrep_realtime-vrep_time          
                time2=vrep_realtime-vrep_time+0.25    
                time3=vrep_realtime-vrep_time+0.5       
                time4=vrep_realtime-vrep_time+0.75      
                T1=time1%T 
                T2=time2%T    #mod 函数为取模运算
                T3=time3%T 
                T4=time4%T 
                [lb_x,lb_z]=gait_plan2(T1,T)
                [rf_x,rf_z]=gait_plan2(T2,T)
                [rb_x,rb_z]=gait_plan2(T3,T)
                [lf_x,lf_z]=gait_plan2(T4,T) 
            if gait_state==2: #对角快步步态
                T=0.4   #步态周期s
                time1=vrep_realtime-vrep_time       
                T1=time1%T
                time2=vrep_realtime-vrep_time+0.2
                T2=time2%T
                [lb_x,lb_z]=gait_plan1(T1,T)
                [rf_x,rf_z]=gait_plan1(T1,T)
                [rb_x,rb_z]=gait_plan1(T2,T)
                [lf_x,lf_z]=gait_plan1(T2,T)
            
        #单腿逆运动学，Y值设定为默认值
        [lb_rot_1_pos,lb_rot_2_pos,lb_rot_3_pos]=xyz(lb_x,-0.15,lb_z)                
        [lf_rot_1_pos,lf_rot_2_pos,lf_rot_3_pos]=xyz(lf_x,-0.15,lf_z)     
        [rb_rot_1_pos,rb_rot_2_pos,rb_rot_3_pos]=xyz(rb_x,-0.15,rb_z)     
        [rf_rot_1_pos,rf_rot_2_pos,rf_rot_3_pos]=xyz(rf_x,-0.15,rf_z);
        #电机控制函数
        rec=sim.simxSetJointTargetPosition(clientID,lb_rot_1,-lb_rot_1_pos,sim.simx_opmode_oneshot)     
        rec=sim.simxSetJointTargetPosition(clientID,lb_rot_2,lb_rot_2_pos,sim.simx_opmode_oneshot)
        rec=sim.simxSetJointTargetPosition(clientID,lb_rot_3,lb_rot_3_pos,sim.simx_opmode_oneshot)
        rec=sim.simxSetJointTargetPosition(clientID,rf_rot_1,rf_rot_1_pos,sim.simx_opmode_oneshot)
        rec=sim.simxSetJointTargetPosition(clientID,rf_rot_2,rf_rot_2_pos,sim.simx_opmode_oneshot)
        rec=sim.simxSetJointTargetPosition(clientID,rf_rot_3,rf_rot_3_pos,sim.simx_opmode_oneshot)
        rec=sim.simxSetJointTargetPosition(clientID,rb_rot_1,-rb_rot_1_pos,sim.simx_opmode_oneshot)
        rec=sim.simxSetJointTargetPosition(clientID,rb_rot_2,rb_rot_2_pos,sim.simx_opmode_oneshot)
        rec=sim.simxSetJointTargetPosition(clientID,rb_rot_3,rb_rot_3_pos,sim.simx_opmode_oneshot)
        rec=sim.simxSetJointTargetPosition(clientID,lf_rot_1,lf_rot_1_pos,sim.simx_opmode_oneshot)
        rec=sim.simxSetJointTargetPosition(clientID,lf_rot_2,lf_rot_2_pos,sim.simx_opmode_oneshot)
        rec=sim.simxSetJointTargetPosition(clientID,lf_rot_3,lf_rot_3_pos,sim.simx_opmode_oneshot)
            
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking) #仿真停止
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
        
print('Program ended')


# In[ ]:




