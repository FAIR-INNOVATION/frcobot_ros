import erarpc
import time


# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

type = 1  # 数据类型，1-关节位置
name = 'tpd2023'  # 轨迹名
period = 4  #采样周期，2ms或4ms或8ms
di_choose = 0 # di输入配置
do_choose = 0 # do输出配置
robot.SetTPDParam(type, name, period, di_choose, do_choose)    #配置TPD参数

robot.Mode(1)  # 机器人切入手动模式
time.sleep(1)  
robot.DragTeachSwitch(1)  #机器人切入拖动示教模式
robot.SetTPDStart(type, name, period, di_choose, do_choose)   # 开始记录示教轨迹
time.sleep(30)
robot.SetWebTPDStop()  # 停止记录示教轨迹
robot.DragTeachSwitch(0)  #机器人切入非拖动示教模式

# robot.SetTPDDelete('tpd2023')   # 删除TPD轨迹
