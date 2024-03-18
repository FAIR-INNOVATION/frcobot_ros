import erarpc

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

#恒力控制
status = 1  #恒力控制开启标志，0-关，1-开
sensor_num = 1 #力传感器编号
is_select = [0,0,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
gain = [0.0005,0.0,0.0,0.0,0.0,0.0]  #最大阈值
adj_sign = 0  #自适应启停状态，0-关闭，1-开启
ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
max_dis = 100.0  #最大调整距离
max_ang = 0.0  #最大调整角度

J1=[-68.987,-96.414,-111.45,-61.105,92.884,11.089]
P1=[62.795,-511.979,291.697,-179.545,3.027,-170.039]
eP1=[0.000,0.000,0.000,0.000]
dP1=[0.000,0.000,0.000,0.000,0.000,0.000]
J2=[-107.596,-109.154,-104.735,-56.176,90.739,11.091]
P2=[-294.768,-503.708,233.158,179.799,0.713,151.309]
eP2=[0.000,0.000,0.000,0.000]
dP2=[0.000,0.000,0.000,0.000,0.000,0.000]

robot.MoveJ(J1,P1,9,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)    #关节空间运动PTP,工具号9，实际测试根据现场数据及工具号使用
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
robot.MoveL(J2,P2,9,0,100.0,180.0,20.0,-1.0,eP2,0,0,dP2)   #笛卡尔空间直线运动
status = 0
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)