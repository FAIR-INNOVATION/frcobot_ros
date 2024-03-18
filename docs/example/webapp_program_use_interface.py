import erarpc
import time
import _thread

def print_program_state(name,rb):
    while(1):
        pstate = robot.GetProgramState()    #查询程序运行状态,1-程序停止或无程序运行，2-程序运行中，3-程序暂停
        linenum = robot.GetCurrentLine()    #查询当前作业程序执行的行号
        name = robot.GetLoadedProgram()     #查询已加载的作业程序名
        print("the robot program state is:",pstate[1])
        print("the robot program line number is:",linenum[1])
        print("the robot program name is:",name[1])
        time.sleep(1)

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = erarpc.RPC('192.168.58.2')

#机器人webapp程序使用接口
robot.Mode(0)   #机器人切入自动运行模式
robot.ProgramLoad('/fruser/testPTP.lua')   #加载要执行的机器人程序，testPTP.lua程序需要先在webapp上编写好
robot.ProgramRun()     #执行机器人程序
_thread.start_new_thread(print_program_state,("print_state",robot))
time.sleep(5)         #休息10s
robot.ProgramPause()   #暂停正在执行的机器人程序
time.sleep(5)
robot.ProgramResume()  #恢复暂停执行的机器人程序
time.sleep(5)
robot.ProgramStop()    #停止正在执行的机器人程序
time.sleep(2)
