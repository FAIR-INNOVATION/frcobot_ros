import erarpc

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

#恒力参数
status = 1  #恒力控制开启标志，0-关，1-开
sensor_num = 1 #力传感器编号
is_select = [0,0,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #最大阈值
adj_sign = 0  #自适应启停状态，0-关闭，1-开启
ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
max_dis = 100.0  #最大调整距离
max_ang = 5.0  #最大调整角度

#螺旋线探索参数
rcs = 0  #参考坐标系，0-工具坐标系，1-基坐标系
dr = 0.7  #每圈半径进给量，单位mm
fFinish = 1.0 #力或力矩阈值（0~100），单位N或Nm
t = 60000.0 #最大探索时间，单位ms
vmax = 3.0 #线速度最大值，单位mm/s

#直线插入参数
rcs = 0  #参考坐标系，0-工具坐标系，1-基坐标系
force_goal = 10.0  #力或力矩阈值（0~100），单位N或Nm
lin_v = 0.0 #直线速度，单位mm/s
lin_a = 0.0 #直线加速度，单位mm/s^2,暂不使用
disMax = 100.0 #最大插入距离，单位mm
linorn = 1 #插入方向，1-正方向，2-负方向

#旋转插入参数
rcs = 0  #参考坐标系，0-工具坐标系，1-基坐标系
angVelRot = 2.0  #旋转角速度，单位°/s
forceInsertion = 1.0 #力或力矩阈值（0~100），单位N或Nm
angleMax= 45 #最大旋转角度，单位°
orn = 1 #力的方向，1-fz,2-mz
angAccmax = 0.0 #最大旋转角加速度，单位°/s^2,暂不使用
rotorn = 1 #旋转方向，1-顺时针，2-逆时针

is_select = [0,0,1,1,1,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
robot.FT_SpiralSearch(rcs,dr,fFinish,t,vmax)
status = 0
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

is_select = [1,1,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
gain = [0.00005,0.0,0.0,0.0,0.0,0.0]  #最大阈值
force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
status = 1
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn)
status = 0
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

s_select = [0,0,1,1,1,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #最大阈值
status = 1
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
robot.FT_RotInsertion(rcs,angVelRot,forceInsertion,angleMax,orn,angAccmax,rotorn)
status = 0
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

is_select = [1,1,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [0.0,0.0,-30.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
status = 1
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn)
status = 0
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)