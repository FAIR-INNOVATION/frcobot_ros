import erarpc
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

joint_pos = robot.GetActualJointPosDegree(0)
print(joint_pos)
joint_pos = [joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5],joint_pos[6]]
acc = 0.0
vel = 0.0
t = 0.008
lookahead_time = 0.0
P = 0.0
count = 100
while(count):
    robot.ServoJ(joint_pos, acc, vel, t, lookahead_time, P)
    joint_pos[0] = joint_pos[0] + 0.1
    count = count - 1
    time.sleep(0.008)
    # robot.WaitMs(1)