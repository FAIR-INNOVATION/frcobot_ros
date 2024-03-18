import erarpc
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

#恒力控制
status = 1  #恒力控制开启标志，0-关，1-开
sensor_num = 1 #力传感器编号
is_select = [1,0,0,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [-2.0,0.0,0.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
gain = [0.0002,0.0,0.0,0.0,0.0,0.0]  #最大阈值
adj_sign = 0  #自适应启停状态，0-关闭，1-开启
ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
max_dis = 15.0  #最大调整距离
max_ang = 0.0  #最大调整角度

#表面定位参数
rcs = 0 #参考坐标系，0-工具坐标系，1-基坐标系
direction = 1 #移动方向，1-正方向，2-负方向
axis = 1 #移动轴，1-X,2-Y,3-Z
lin_v = 3.0  #探索直线速度，单位mm/s
lin_a = 0.0  #探索直线加速度，单位mm/s^2
disMax = 50.0 #最大探索距离，单位mm
force_goal = 2.0 #动作终止力阈值，单位N


P1=[-230.959,-364.017,226.179,-179.004,0.002,89.999]
robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)       #关节空间点到点运动
#x方向寻找中心
#第1个表面
robot.FT_CalCenterStart()
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
status = 0
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)       #关节空间点到点运动
robot.WaitMs(1000)
#第2个表面
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
direction = 2 #移动方向，1-正方向，2-负方向
robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
status = 0
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
#计算x方向中心位置
xcenter= robot.FT_CalCenterEnd()
print(xcenter)
xcenter = [xcenter[1],xcenter[2],xcenter[3],xcenter[4],xcenter[5],xcenter[6]]
robot.MoveCart(xcenter,9,0,60.0,50.0,50.0,0.0,-1)

# time.sleep(1)

#y方向寻找中心
#第1个表面
robot.FT_CalCenterStart()
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
direction = 1 #移动方向，1-正方向，2-负方向
axis = 2 #移动轴，1-X,2-Y,3-Z
disMax = 150.0 #最大探索距离，单位mm
lin_v = 6.0  #探索直线速度，单位mm/s
robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
status = 0
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)       #关节空间点到点运动
robot.WaitMs(1000)
#第2个表面
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
direction = 2 #移动方向，1-正方向，2-负方向
robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
status = 0
robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
#计算y方向中心位置
ycenter=robot.FT_CalCenterEnd()
print(ycenter)
ycenter = [ycenter[1],ycenter[2],ycenter[3],ycenter[4],ycenter[5],ycenter[6]]
robot.MoveCart(ycenter,9,0,60.0,50.0,50.0,-1.0,-1)