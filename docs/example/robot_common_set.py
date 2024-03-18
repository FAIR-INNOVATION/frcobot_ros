import erarpc

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

robot.SetSpeed(20)

for i in range(1,21):
    robot.SetSysVarValue(i,i+0.5)
robot.WaitMs(1000)
for i in range(1,21):
    sys_var = robot.GetSysVarValue(i)
    print(sys_var)

robot.SetLoadWeight(3.0)
robot.SetLoadCoord(3.0,4.0,5.0)
t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
robot.SetToolCoord(10,t_coord,0,0)

robot.SetToolList(10,t_coord,0,0)

etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
etool = [21.0,22.0,23.0,24.0,25.0,26.0]
robot.SetExToolCoord(10,etcp,etool)
robot.SetExToolList(10,etcp,etool)

w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
robot.SetWObjCoord(11,w_coord)
robot.SetWObjList(11,w_coord)
robot.SetRobotInstallPos(3)
robot.SetRobotInstallAngle(10.0,11.0)
