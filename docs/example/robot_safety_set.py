import erarpc

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

level = [1.0,2.0,3.0,4.0,5.0,6.0]
robot.SetAnticollision(0,level,1)
level = [50.0,20.0,30.0,40.0,50.0,60.0]
robot.SetAnticollision(1,level,1)
robot.SetCollisionStrategy(1)

p_limit = [170.0,80.0,150.0,80.0,170.0,160.0]
n_limit = [-170.0,-260.0,-150.0,-260.0,-170.0,-160.0]
robot.SetLimitPositive(p_limit)
robot.SetLimitNegative(n_limit)

robot.ResetAllError()

robot.FrictionCompensationOnOff(1)

lcoeff = [0.9,0.9,0.9,0.9,0.9,0.9]
wcoeff = [0.4,0.4,0.4,0.4,0.4,0.4]
ccoeff = [0.6,0.6,0.6,0.6,0.6,0.6]
fcoeff = [0.5,0.5,0.5,0.5,0.5,0.5]
robot.SetFrictionValue_level(lcoeff)
robot.SetFrictionValue_wall(wcoeff)
robot.SetFrictionValue_ceiling(ccoeff)
robot.SetFrictionValue_freedom(fcoeff)


