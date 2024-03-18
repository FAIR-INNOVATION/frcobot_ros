import erarpc


# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

P1=[-378.9,-340.3,107.2,179.4,-1.3,125.0]
name = 'tpd2023'   #轨迹名
blend = 1   #是否平滑，1-平滑，0-不平滑
ovl = 100.0   #速度缩放

robot.LoadTPD(name)  #轨迹预加载
robot.MoveCart(P1,1,0,100.0,100.0,100.0,-1.0,-1)       #运动到起始点
robot.MoveTPD(name, blend, ovl)  #轨迹复现
