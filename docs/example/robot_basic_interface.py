import erarpc
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

ret = robot.GetSDKVersion()    #查询SDK版本号
if ret[0] == 0:  #0-无故障，返回格式：[errcode,data],errcode-故障码，data-数据
    print("SDK version is:",ret[1])
else:
    print("some things happened, the errcode is: ", ret[0])

ret = robot.GetControllerIP()    #查询控制器IP
if ret[0] == 0:
    print("controller ip is:",ret[1])
else:
    print("some things happened, the errcode is: ", ret[0])

#机器人进入或退出拖动示教模式
robot.Mode(1) #机器人切入手动模式
time.sleep(1)
robot.DragTeachSwitch(1)  #机器人切入拖动示教模式，必须在手动模式下才能切入拖动示教模式
time.sleep(1)
ret = robot.IsInDragTeach()    #查询是否处于拖动示教模式，1-拖动示教模式，0-非拖动示教模式
if ret[0] == 0:
    print("drag state is:",ret[1])
else:
    print("some things happened, the errcode is: ", ret[0])
time.sleep(3)
robot.DragTeachSwitch(0)  #机器人切入非拖动示教模式，必须在手动模式下才能切入非拖动示教模式
time.sleep(1)
ret = robot.IsInDragTeach()    #查询是否处于拖动示教模式，1-拖动示教模式，0-非拖动示教模式
if ret[0] == 0:
    print("drag state is:",ret[1])
else:
    print("some things happened, the errcode is: ", ret[0])
time.sleep(3)

#机器人上使能或下使能
robot.RobotEnable(0)   #机器人下使能
time.sleep(3)
robot.RobotEnable(1)   #机器人上使能，机器人上电后默认自动上使能

#机器人手自动模式切换
robot.Mode(0)   #机器人切入自动运行模式
time.sleep(1)  
robot.Mode(1)   #机器人切入手动模式
