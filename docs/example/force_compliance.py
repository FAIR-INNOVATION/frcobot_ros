import erarpc

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

J1=[-105.3,-68.0,-127.9,-75.5,90.8,77.8]
P1=[-208.9,-274.5,334.6,178.8,-1.3,86.7]
eP1=[0.000,0.000,0.000,0.000]
dP1=[0.000,0.000,0.000,0.000,0.000,0.000]
J2=[-105.3,-97.9,-101.5,-70.3,90.8,77.8]
P2=[-264.8,-480.5,341.8,179.2,0.3,86.7]
eP2=[0.000,0.000,0.000,0.000]
dP2=[0.000,0.000,0.000,0.000,0.000,0.000]

#恒力控制参数
status = 1  #恒力控制开启标志，0-关，1-开
sensor_num = 1 #力传感器编号
is_select = [1,1,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [-10.0,-10.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
gain = [0.0005,0.0,0.0,0.0,0.0,0.0]  #最大阈值
adj_sign = 0  #自适应启停状态，0-关闭，1-开启
ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
max_dis = 1000.0  #最大调整距离
max_ang = 0.0  #最大调整角度

#柔顺控制
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
p = 0.00005  #位置调节系数或柔顺系数
force = 30.0 #柔顺开启力阈值，单位N
robot.FT_ComplianceStart(p,force)
count = 15  #循环次数
while(count):
    robot.MoveL(J1,P1,9,0,100.0,180.0,100.0,-1.0,eP1,0,1,dP1)   #笛卡尔空间直线运动
    robot.MoveL(J2,P2,9,0,100.0,180.0,100.0,-1.0,eP2,0,0,dP2)
    count = count - 1
robot.FT_ComplianceStop()
status = 0
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)