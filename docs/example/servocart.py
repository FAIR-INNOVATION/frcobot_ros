import erarpc
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

mode = 2  #工具坐标系增量运动
n_pos = [0.0,0.0,0.5,0.0,0.0,0.0]   #笛卡尔空间位姿增量
gain = [0.0,0.0,1.0,0.0,0.0,0.0]
acc = 0.0
vel = 0.0
t = 0.008
lookahead_time = 0.0
P = 0.0
count = 100
while(count):
    robot.ServoCart(mode, n_pos, gain, acc, vel, t, lookahead_time, P)
    count = count - 1
    time.sleep(0.008)
    # robot.WaitMs(1)
