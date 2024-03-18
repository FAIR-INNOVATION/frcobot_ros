import erarpc
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')


#测试外设指令
robot.SetGripperConfig(4,0,0,1)
time.sleep(1)
config = robot.GetGripperConfig()
print(config)
robot.ActGripper(1,0)
time.sleep(1)
robot.ActGripper(1,1)
time.sleep(2)
robot.MoveGripper(1,100,48,46,30000,0)
time.sleep(3)
robot.MoveGripper(1,0,50,0,30000,0)
robot.GetGripperMotionDone()