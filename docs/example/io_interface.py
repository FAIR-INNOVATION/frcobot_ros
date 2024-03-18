import erarpc

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')


# 测试控制箱DO
for i in range(0,16):
    robot.SetDO(i,1,0,0)      #打开控制箱DO
robot.WaitMs(1000)
for i in range(0,16):
    robot.SetDO(i,0,0,0)      #关闭控制箱DO
robot.WaitMs(1000)

# 测试末端DO
for i in range(0,2):
    robot.SetToolDO(i,1,0,0)    #打开工具DO
robot.WaitMs(1000)
for i in range(0,2):
    robot.SetToolDO(i,0,0,0)    #关闭工具DO
    

# 测试控制箱AO
robot.SetAO(0,0.0,0)
robot.SetAO(1,100.0,0)
robot.WaitMs(1000)
robot.SetAO(0,0.0,0)
robot.SetAO(1,0.0,0)

# 测试末端AO
robot.SetToolAO(0,100.0,0)
robot.WaitMs(1000)
robot.SetToolAO(0,0.0,0)

# 测试DI
di = robot.GetDI(0,0)
print(di)
robot.WaitDI(0,1,0,2)       # 一直等待
robot.WaitMultiDI(1,3,3,10000,2)   #一直等待
tool_di = robot.GetToolDI(1,0)
print(tool_di)
robot.WaitToolDI(1,1,0,2)      #一直等待

# 测试AI
ai = robot.GetAI(0,1)
print(ai)
robot.WaitAI(0,0,50,0,2)         #一直等待
robot.WaitToolAI(0,0,50,0,2)     #一直等待
tool_ai = robot.GetToolAI(0,1)
print(tool_ai)


