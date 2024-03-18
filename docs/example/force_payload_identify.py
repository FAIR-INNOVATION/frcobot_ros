import erarpc
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

#负载辨识，此时末端安装要辨识的工具，工具安装在力传感器下方,末端竖直向下
robot.FT_SetRCS(0)    #设置参考坐标系为工具坐标系，0-工具坐标系，1-基坐标系
time.sleep(1)
tool_id = 10  #传感器坐标系编号
tool_coord = [0.0,0.0,35.0,0.0,0.0,0.0]   # 传感器相对末端法兰位姿
tool_type = 1  # 0-工具，1-传感器
tool_install = 0 # 0-安装末端，1-机器人外部
robot.SetToolCoord(tool_id,tool_coord,tool_type,tool_install)     #设置传感器坐标系，传感器相对末端法兰位姿
time.sleep(1)
robot.FT_PdIdenRecord(tool_id)   #记录辨识数据
time.sleep(1)
weight = robot.FT_PdIdenCompute()  #计算负载重量，单位kg
print(weight)

#负载质心辨识，机器人需要示教三个不同的姿态，然后记录辨识数据，最后计算负载质心
P1=[-160.619,-586.138,384.988,-170.166,-44.782,169.295]
robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)         #关节空间点到点运动
time.sleep(1)
robot.FT_PdCogIdenRecord(tool_id,1)                               #记录辨识数据
time.sleep(1)
P2=[-87.615,-606.209,556.119,-102.495,10.118,178.985]
robot.MoveCart(P2,9,0,100.0,100.0,100.0,-1.0,-1)
time.sleep(1)      
robot.FT_PdCogIdenRecord(tool_id,2)
time.sleep(1)
P3=[41.479,-557.243,484.407,-125.174,46.995,-132.165]
robot.MoveCart(P3,9,0,100.0,100.0,100.0,-1.0,-1)
time.sleep(1)
robot.FT_PdCogIdenRecord(tool_id,3)
time.sleep(1)
cog = robot.FT_PdCogIdenCompute()
print(cog)
