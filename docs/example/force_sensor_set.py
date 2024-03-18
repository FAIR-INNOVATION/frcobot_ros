import erarpc
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

company = 17    #传感器厂商，17-坤维科技
device = 0      #传感器设备号
softversion = 0 #软件版本号
bus = 1         #末端总线位置
robot.FT_SetConfig(company, device, softversion, bus)   #配置力传感器
config = robot.FT_GetConfig() #获取力传感器配置信息，厂商编号下发比反馈大1
print(config)

time.sleep(1)

robot.FT_Activate(0)  #传感器复位
time.sleep(1)
robot.FT_Activate(1)  #传感器激活
time.sleep(1)

robot.SetLoadWeight(0.0)    #末端负载设置为零
time.sleep(1)
robot.SetLoadCoord(0.0,0.0,0.0)  #末端负载质心设置为零
time.sleep(1)
robot.FT_SetZero(0)   #传感器去除零点
time.sleep(1)
origin = robot.FT_GetForceTorqueOrigin()   #查询传感器原始数据
print(origin)
robot.FT_SetZero(1)   #传感器零点矫正,注意此时末端不能安装工具，只有力传感器
time.sleep(1)
rcs = robot.FT_GetForceTorqueRCS()  #查询传感器坐标系下数据
print(rcs)
