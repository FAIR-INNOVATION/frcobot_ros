#ifndef  FRROBOT_H_
#define  FRROBOT_H_

#include "RobotTypes.h"

class FRRobot
{
public:
    /**
	 *@brief  机器人接口类构造函数
	 */
    FRRobot();
    
	/**
	 *@brief  与机器人控制器建立通信
	 *@param  [in] ip  控制器IP地址，出场默认为192.168.58.2
	 *@return 错误码
	 */
    errno_t  RPC(const char *ip);
	
	/**
	 *@brief  查询SDK版本号
	 *@param  [out] version   SDK版本号
     *@return  错误码
     */	 
	errno_t  GetSDKVersion(char *version);
	
	/**
	 *@brief  获取控制器IP
	 *@param  [out] ip  控制器IP
	 *@return  错误码
	 */
	errno_t  GetControllerIP(char *ip);
	
	/**
	 *@brief  控制机器人进入或退出拖动示教模式
	 *@param  [in] state 0-退出拖动示教模式，1-进入拖动示教模式
	 *@return  错误码
	 */
	errno_t  DragTeachSwitch(uint8_t state);
	
	/**
	 *@brief  查询机器人是否处于拖动示教模式
	 *@param  [out] state 0-非拖动示教模式，1-拖动示教模式
	 *@return  错误码
	 */
	errno_t  IsInDragTeach(uint8_t *state);
	
	/**
	 *@brief  控制机器人上使能或下使能，机器人上电后默认自动上使能
	 *@param  [in] state  0-下使能，1-上使能
	 *@return  错误码
	 */
	errno_t  RobotEnable(uint8_t state);
    
	/**
	 *@brief 控制机器人手自动模式切换
	 *@param [in] mode 0-自动模式，1-手动模式
	 *@return 错误码
	 */
    errno_t  Mode(int mode);
	
	/**
	 *@brief  jog点动
	 *@param  [in]  ref 0-关节点动，2-基坐标系下点动，4-工具坐标系下点动，8-工件坐标系下点动
	 *@param  [in]  nb 1-关节1(或x轴)，2-关节2(或y轴)，3-关节3(或z轴)，4-关节4(或绕x轴旋转)，5-关节5(或绕y轴旋转)，6-关节6(或绕z轴旋转)
	 *@param  [in]  dir 0-负方向，1-正方向
	 *@param  [in]  vel 速度百分比，[0~100]
	 *@param  [in]  acc 加速度百分比， [0~100]
	 *@param  [in]  max_dis 单次点动最大角度，单位[°]或距离，单位[mm]
	 *@return  错误码
	 */
	errno_t  StartJOG(uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis);
	
	/**
	 *@brief  jog点动减速停止
	 *@param  [in]  ref  1-关节点动停止，3-基坐标系下点动停止，5-工具坐标系下点动停止，9-工件坐标系下点动停止
	 *@return  错误码
	 */
	errno_t  StopJOG(uint8_t ref);
	
	/**
	 *@brief jog点动立即停止
	 *@return  错误码
	 */
	errno_t  ImmStopJOG(); 
	
	/**
	 *@brief  关节空间运动
	 *@param  [in] joint_pos  目标关节位置,单位deg
	 *@param  [in] desc_pos   目标笛卡尔位姿
	 *@param  [in] tool  工具坐标号，范围[1~15]
	 *@param  [in] user  工件坐标号，范围[1~15]
	 *@param  [in] vel  速度百分比，范围[0~100]
	 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 *@param  [in] ovl  速度缩放因子，范围[0~100]
	 *@param  [in] epos  扩展轴位置，单位mm
	 *@param  [in] blendT [-1.0]-运动到位(阻塞)，[0~500.0]-平滑时间(非阻塞)，单位ms
	 *@param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 *@param  [in] offset_pos  位姿偏移量
	 *@return  错误码
	 */
	errno_t  MoveJ(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos);
	
	/**
	 *@brief  笛卡尔空间直线运动
	 *@param  [in] joint_pos  目标关节位置,单位deg
	 *@param  [in] desc_pos   目标笛卡尔位姿
	 *@param  [in] tool  工具坐标号，范围[1~15]
	 *@param  [in] user  工件坐标号，范围[1~15]
	 *@param  [in] vel  速度百分比，范围[0~100]
	 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 *@param  [in] ovl  速度缩放因子，范围[0~100]
	 *@param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm	 
	 *@param  [in] epos  扩展轴位置，单位mm
	 *@param  [in] search  0-不焊丝寻位，1-焊丝寻位
	 *@param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 *@param  [in] offset_pos  位姿偏移量
	 *@return  错误码
	 */	
	errno_t  MoveL(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos);

	/**
	 *@brief  笛卡尔空间圆弧运动
	 *@param  [in] joint_pos_p  路径点关节位置,单位deg
	 *@param  [in] desc_pos_p   路径点笛卡尔位姿
	 *@param  [in] ptool  工具坐标号，范围[1~15]
	 *@param  [in] puser  工件坐标号，范围[1~15]
	 *@param  [in] pvel  速度百分比，范围[0~100]
	 *@param  [in] pacc  加速度百分比，范围[0~100],暂不开放
	 *@param  [in] epos_p  扩展轴位置，单位mm
	 *@param  [in] poffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 *@param  [in] offset_pos_p  位姿偏移量
	 *@param  [in] joint_pos_t  目标点关节位置,单位deg
	 *@param  [in] desc_pos_t   目标点笛卡尔位姿
	 *@param  [in] ttool  工具坐标号，范围[1~15]
	 *@param  [in] tuser  工件坐标号，范围[1~15]
	 *@param  [in] tvel  速度百分比，范围[0~100]
	 *@param  [in] tacc  加速度百分比，范围[0~100],暂不开放
	 *@param  [in] epos_t  扩展轴位置，单位mm
	 *@param  [in] toffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 *@param  [in] offset_pos_t  位姿偏移量	 
	 *@param  [in] ovl  速度缩放因子，范围[0~100]	 
	 *@param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm	 
	 *@return  错误码
	 */		
	errno_t  MoveC(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p,JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t,float ovl, float blendR);
	
	/**
	 *@brief  笛卡尔空间整圆运动
	 *@param  [in] joint_pos_p  路径点1关节位置,单位deg
	 *@param  [in] desc_pos_p   路径点1笛卡尔位姿
	 *@param  [in] ptool  工具坐标号，范围[1~15]
	 *@param  [in] puser  工件坐标号，范围[1~15]
	 *@param  [in] pvel  速度百分比，范围[0~100]
	 *@param  [in] pacc  加速度百分比，范围[0~100],暂不开放
	 *@param  [in] epos_p  扩展轴位置，单位mm
	 *@param  [in] joint_pos_t  路径点2关节位置,单位deg
	 *@param  [in] desc_pos_t   路径点2笛卡尔位姿
	 *@param  [in] ttool  工具坐标号，范围[1~15]
	 *@param  [in] tuser  工件坐标号，范围[1~15]
	 *@param  [in] tvel  速度百分比，范围[0~100]
	 *@param  [in] tacc  加速度百分比，范围[0~100],暂不开放
	 *@param  [in] epos_t  扩展轴位置，单位mm
	 *@param  [in] ovl  速度缩放因子，范围[0~100]	
	 *@param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 *@param  [in] offset_pos  位姿偏移量	 	 
	 *@return  错误码
	 */		
	errno_t  Circle(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, float ovl, uint8_t offset_flag, DescPose *offset_pos);
	
	/**
	 *@brief  笛卡尔空间螺旋线运动
	 *@param  [in] joint_pos  目标关节位置,单位deg
	 *@param  [in] desc_pos   目标笛卡尔位姿
	 *@param  [in] tool  工具坐标号，范围[1~15]
	 *@param  [in] user  工件坐标号，范围[1~15]
	 *@param  [in] vel  速度百分比，范围[0~100]
	 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 *@param  [in] epos  扩展轴位置，单位mm
	 *@param  [in] ovl  速度缩放因子，范围[0~100]	 
	 *@param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
	 *@param  [in] offset_pos  位姿偏移量
	 *@param  [in] spiral_param  螺旋参数
	 *@return  错误码
	 */
	errno_t  NewSpiral(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, ExaxisPos *epos, float ovl, uint8_t offset_flag, DescPose *offset_pos, SpiralParam spiral_param);	
	
	/**
	 *@brief  关节空间伺服模式运动
	 *@param  [in] joint_pos  目标关节位置,单位deg
	 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放，默认为0
	 *@param  [in] vel  速度百分比，范围[0~100]，暂不开放，默认为0
	 *@param  [in] cmdT  指令下发周期，单位s，建议范围[0.001~0.0016]
	 *@param  [in] filterT 滤波时间，单位s，暂不开放，默认为0
	 *@param  [in] gain  目标位置的比例放大器，暂不开放，默认为0
	 *@return  错误码
	 */
	errno_t  ServoJ(JointPos *joint_pos, float acc, float vel, float cmdT, float filterT, float gain);

	/**
	 *@brief  笛卡尔空间伺服模式运动
	 *@param  [in]  mode  0-绝对运动(基坐标系)，1-增量运动(基坐标系)，2-增量运动(工具坐标系)
	 *@param  [in]  desc_pos  目标笛卡尔位姿或位姿增量
	 *@param  [in]  pos_gain  位姿增量比例系数，仅在增量运动下生效，范围[0~1]
	 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放，默认为0
	 *@param  [in] vel  速度百分比，范围[0~100]，暂不开放，默认为0
	 *@param  [in] cmdT  指令下发周期，单位s，建议范围[0.001~0.0016]
	 *@param  [in] filterT 滤波时间，单位s，暂不开放，默认为0
	 *@param  [in] gain  目标位置的比例放大器，暂不开放，默认为0
	 *@return  错误码
	 */
	errno_t  ServoCart(int mode, DescPose *desc_pose, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain);
   
	/**
	 *@brief  笛卡尔空间点到点运动
	 *@param  [in]  desc_pos  目标笛卡尔位姿或位姿增量
	 *@param  [in] tool  工具坐标号，范围[1~15]
	 *@param  [in] user  工件坐标号，范围[1~15]
	 *@param  [in] vel  速度百分比，范围[0~100]
	 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 *@param  [in] ovl  速度缩放因子，范围[0~100]
	 *@param  [in] blendT [-1.0]-运动到位(阻塞)，[0~500.0]-平滑时间(非阻塞)，单位ms	
     *@param  [in] config  关节空间配置，[-1]-参考当前关节位置解算，[0~7]-参考特定关节空间配置解算，默认为-1	 
	 *@return  错误码
	 */
	errno_t  MoveCart(DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config);
	
	/**
	 *@brief  样条运动开始
	 *@return  错误码
	 */
	errno_t  SplineStart();

    /**
	 *@brief  关节空间样条运动
	 *@param  [in] joint_pos  目标关节位置,单位deg
	 *@param  [in] desc_pos   目标笛卡尔位姿
	 *@param  [in] tool  工具坐标号，范围[1~15]
	 *@param  [in] user  工件坐标号，范围[1~15]
	 *@param  [in] vel  速度百分比，范围[0~100]
	 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 *@param  [in] ovl  速度缩放因子，范围[0~100]	
	 *@return  错误码
	 */
	errno_t  SplinePTP(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl);
	
	/**
	 *@brief  样条运动结束
	 *@return  错误码
	 */
	errno_t  SplineEnd();
	
	/**
	 *@brief 新样条运动开始
	 *@param  [in] type   0-圆弧过渡，1-给定点位为路径点
	 *@return  错误码
	 */
	errno_t  NewSplineStart(int type);
	
	/**
	 *@brief 新样条指令点
	 *@param  [in] joint_pos  目标关节位置,单位deg
	 *@param  [in] desc_pos   目标笛卡尔位姿
	 *@param  [in] tool  工具坐标号，范围[1~15]
	 *@param  [in] user  工件坐标号，范围[1~15]
	 *@param  [in] vel  速度百分比，范围[0~100]
	 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放
	 *@param  [in] ovl  速度缩放因子，范围[0~100]
	 *@param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
	 *@return  错误码
	 */	 
	errno_t  NewSplinePoint(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR);
	
	/**
	 *@brief 新样条运动结束
	 *@return  错误码
	 */
	errno_t  NewSplineEnd();
	
	/**
	 *@brief 终止运动
	 *@return  错误码
	 */
	errno_t  StopMotion();
	
	/**
	 *@brief  点位整体偏移开始
	 *@param  [in]  flag  0-基坐标系下/工件坐标系下偏移，2-工具坐标系下偏移
	 *@param  [in] offset_pos  位姿偏移量
	 *@return  错误码
	 */
	errno_t  PointsOffsetEnable(int flag, DescPose *offset_pos);
	
	/**
	 *@brief  点位整体偏移结束
	 *@return  错误码
	 */
	errno_t  PointsOffsetDisable();
	
	/**
	 *@brief  设置控制箱数字量输出
	 *@param  [in] id  io编号，范围[0~15]
	 *@param  [in] status 0-关，1-开
	 *@param  [in] smooth 0-不平滑， 1-平滑
	 *@param  [in] block  0-阻塞，1-非阻塞
	 *@return  错误码
	 */
	errno_t  SetDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

	/**
	 *@brief  设置工具数字量输出
	 *@param  [in] id  io编号，范围[0~1]
	 *@param  [in] status 0-关，1-开
	 *@param  [in] smooth 0-不平滑， 1-平滑
	 *@param  [in] block  0-阻塞，1-非阻塞
	 *@return  错误码
	 */
	errno_t  SetToolDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

	/**
	 *@brief  设置控制箱模拟量输出
	 *@param  [in] id  io编号，范围[0~1]
	 *@param  [in] value 电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]
	 *@param  [in] block  0-阻塞，1-非阻塞
	 *@return  错误码
	 */
	errno_t  SetAO(int id, float value, uint8_t block);

	/**
	 *@brief  设置工具模拟量输出
	 *@param  [in] id  io编号，范围[0]
	 *@param  [in] value 电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]
	 *@param  [in] block  0-阻塞，1-非阻塞
	 *@return  错误码
	 */
	errno_t  SetToolAO(int id, float value, uint8_t block);

	/**
	 *@brief  获取控制箱数字量输入
	 *@param  [in] id  io编号，范围[0~15]
	 *@param  [in] block  0-阻塞，1-非阻塞
	 *@param  [out] result  0-低电平，1-高电平
	 *@return  错误码
	 */	
	errno_t  GetDI(int id, uint8_t block, uint8_t *result);

	/**
	 *@brief  获取工具数字量输入
	 *@param  [in] id  io编号，范围[0~1]
	 *@param  [in] block  0-阻塞，1-非阻塞
	 *@param  [out] result  0-低电平，1-高电平
	 *@return  错误码
	 */	
	errno_t  GetToolDI(int id, uint8_t block, uint8_t *result);
	
	/**
	 *@brief 等待控制箱数字量输入
	 *@param  [in] id  io编号，范围[0~15]
	 *@param  [in]  status 0-关，1-开
	 *@param  [in]  max_time  最大等待时间，单位ms
	 *@param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
	 *@return  错误码
	 */
	errno_t  WaitDI(int id, uint8_t status, int max_time, int opt);

	/**
	 *@brief 等待控制箱多路数字量输入
	 *@param  [in] mode 0-多路与，1-多路或
	 *@param  [in] id  io编号，bit0~bit7对应DI0~DI7，bit8~bit15对应CI0~CI7
	 *@param  [in]  status 0-关，1-开
	 *@param  [in]  max_time  最大等待时间，单位ms
	 *@param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
	 *@return  错误码
	 */
	errno_t  WaitMultiDI(int mode, int id, uint8_t status, int max_time, int opt);

	/**
	 *@brief 等待工具数字量输入
	 *@param  [in] id  io编号，范围[0~1]
	 *@param  [in]  status 0-关，1-开
	 *@param  [in]  max_time  最大等待时间，单位ms
	 *@param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
	 *@return  错误码
	 */
	errno_t  WaitToolDI(int id, uint8_t status, int max_time, int opt);
	
	/**
	 *@brief  获取控制箱模拟量输入
	 *@param  [in] id  io编号，范围[0~1]
	 *@param  [in] block  0-阻塞，1-非阻塞
	 *@param  [out] result  输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
	 *@return  错误码
	 */	
	errno_t  GetAI(int id, uint8_t block, float *result);	

	/**
	 *@brief  获取工具模拟量输入
	 *@param  [in] id  io编号，范围[0]
	 *@param  [in] block  0-阻塞，1-非阻塞
	 *@param  [out] result  输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
	 *@return  错误码
	 */	
	errno_t  GetToolAI(int id, uint8_t block, float *result);	
	
	/**
	 *@brief 等待控制箱模拟量输入
	 *@param  [in] id  io编号，范围[0~1]
	 *@param  [in]  sign 0-大于，1-小于
	 *@param  [in]  value 输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
	 *@param  [in]  max_time  最大等待时间，单位ms
	 *@param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
	 *@return  错误码
	 */
	errno_t  WaitAI(int id, int sign, float value, int max_time, int opt);	
	
	/**
	 *@brief 等待工具模拟量输入
	 *@param  [in] id  io编号，范围[0]
	 *@param  [in]  sign 0-大于，1-小于
	 *@param  [in]  value 输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
	 *@param  [in]  max_time  最大等待时间，单位ms
	 *@param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
	 *@return  错误码
	 */
	errno_t  WaitToolAI(int id, int sign, float value, int max_time, int opt);	

    /**
	 *@brief  设置全局速度
	 *@param  [in]  vel  速度百分比，范围[0~100]
	 *@return  错误码
	 */
	errno_t  SetSpeed(int vel);
	
	/**
	 *@brief  设置系统变量值
	 *@param  [in]  id  变量编号，范围[1~20]
	 *@param  [in]  value 变量值
	 *@return  错误码
	 */
	errno_t  SetSysVarValue(int id, float value);
	
	/**
	 *@brief  设置工具坐标系
	 *@param  [in] id 坐标系编号，范围[1~15]
	 *@param  [in] coord  工具中心点相对于末端法兰中心位姿
	 *@param  [in] type  0-工具坐标系，1-传感器坐标系
	 *@param  [in] install 安装位置，0-机器人末端，1-机器人外部
	 *@return  错误码
	 */
	errno_t  SetToolCoord(int id, DescPose *coord, int type, int install);
	
	/**
	 *@brief  设置工具坐标系列表
	 *@param  [in] id 坐标系编号，范围[1~15]
	 *@param  [in] coord  工具中心点相对于末端法兰中心位姿
	 *@param  [in] type  0-工具坐标系，1-传感器坐标系
	 *@param  [in] install 安装位置，0-机器人末端，1-机器人外部
	 *@return  错误码
	 */
	errno_t  SetToolList(int id, DescPose *coord, int type, int install);	
	
	/**
	 *@brief  设置外部工具坐标系
	 *@param  [in] id 坐标系编号，范围[1~15]
	 *@param  [in] etcp  工具中心点相对末端法兰中心位姿
	 *@param  [in] etool  待定
	 *@return  错误码
	 */
	errno_t  SetExToolCoord(int id, DescPose *etcp, DescPose *etool);
	
	/**
	 *@brief  设置外部工具坐标系列表
	 *@param  [in] id 坐标系编号，范围[1~15]
	 *@param  [in] etcp  工具中心点相对末端法兰中心位姿
	 *@param  [in] etool  待定
	 *@return  错误码
	 */
	errno_t  SetExToolList(int id, DescPose *etcp, DescPose *etool);	
	
	/**
	 *@brief  设置工件坐标系
	 *@param  [in] id 坐标系编号，范围[1~15]
	 *@param  [in] coord  工件坐标系相对于末端法兰中心位姿
	 *@return  错误码
     */	 
	errno_t  SetWObjCoord(int id, DescPose *coord);
	
	/**
	 *@brief  设置工件坐标系列表
	 *@param  [in] id 坐标系编号，范围[1~15]
	 *@param  [in] coord  工件坐标系相对于末端法兰中心位姿
	 *@return  错误码
     */	 
	errno_t  SetWObjList(int id, DescPose *coord);	
	
	/**
	 *@brief  设置末端负载重量
	 *@param  [in] weight  负载重量，单位kg
	 *@return  错误码
	 */
	errno_t  SetLoadWeight(float weight);
	
	/**
	 *@brief  设置末端负载质心坐标
	 *@param  [in] coord 质心坐标，单位mm
	 *@return  错误码
	 */
	errno_t  SetLoadCoord(DescTran *coord);

	/**
	 *@brief  设置机器人安装方式
	 *@param  [in] install  安装方式，0-正装，1-侧装，2-倒装
	 *@return  错误码
	 */
	errno_t  SetRobotInstallPos(uint8_t install);	

	/**
	 *@brief  设置机器人安装角度，自由安装
	 *@param  [in] yangle  倾斜角
	 *@param  [in] zangle  旋转角
	 *@return  错误码
	 */
	errno_t  SetRobotInstallAngle(double yangle, double zangle);	

	/**
	 *@brief  等待指定时间
	 *@param  [in]  t_ms  单位ms
	 *@return  错误码
	 */
	errno_t  WaitMs(int t_ms);
	
	/**
	 *@brief 设置碰撞等级
	 *@param  [in]  mode  0-等级，1-百分比
	 *@param  [in]  level 碰撞阈值，等级对应范围[],百分比对应范围[0~1]
	 *@param  [in]  config 0-不更新配置文件，1-更新配置文件
	 *@return  错误码
	 */
	errno_t  SetAnticollision(int mode, float level[6], int config);
	
	/**
	 *@brief  设置碰撞后策略
	 *@param  [in] strategy  0-报错停止，1-继续运行
     *@return  错误码	 
	 */
	errno_t  SetCollisionStrategy(int strategy);
	
	/**
	 *@brief  设置正限位
	 *@param  [in] limit 六个关节位置，单位deg
	 *@return  错误码
	 */
	errno_t  SetLimitPositive(float limit[6]);
	
	/**
	 *@brief  设置负限位
	 *@param  [in] limit 六个关节位置，单位deg
	 *@return  错误码
	 */
	errno_t  SetLimitNegative(float limit[6]);	
	
	/**
	 *@brief  错误状态清除
	 *@return  错误码
	 */
	errno_t  ResetAllError();
	
	/**
	 *@brief  关节摩擦力补偿开关
	 *@param  [in]  state  0-关，1-开
	 *@return  错误码
	 */
	errno_t  FrictionCompensationOnOff(uint8_t state);
	
	/**
	 *@brief  设置关节摩擦力补偿系数-正装
	 *@param  [in]  coeff 六个关节补偿系数，范围[0~1]
	 *@return  错误码
	 */
	errno_t  SetFrictionValue_level(float coeff[6]);

	/**
	 *@brief  设置关节摩擦力补偿系数-侧装
	 *@param  [in]  coeff 六个关节补偿系数，范围[0~1]
	 *@return  错误码
	 */
	errno_t  SetFrictionValue_wall(float coeff[6]);

	/**
	 *@brief  设置关节摩擦力补偿系数-倒装
	 *@param  [in]  coeff 六个关节补偿系数，范围[0~1]
	 *@return  错误码
	 */
	errno_t  SetFrictionValue_ceiling(float coeff[6]);

	/**
	 *@brief  设置关节摩擦力补偿系数-自由安装
	 *@param  [in]  coeff 六个关节补偿系数，范围[0~1]
	 *@return  错误码
	 */
	errno_t  SetFrictionValue_freedom(float coeff[6]);
	
	/**
	 *@brief  获取机器人安装角度
	 *@param  [out] yangle 倾斜角
	 *@param  [out] zangle 旋转角
	 *@return  错误码
	 */
	errno_t  GetRobotInstallAngle(float *yangle, float *zangle);
	
	/**
	 *@brief  获取系统变量值
	 *@param  [in] id 系统变量编号，范围[1~20]
	 *@param  [out] value  系统变量值
	 *@return  错误码
	 */
	errno_t  GetSysVarValue(int id, float *value);
	
	/**
	 *@brief  获取当前关节位置(角度)
	 *@param  [in] flag 0-阻塞，1-非阻塞
	 *@param  [out] jPos 六个关节位置，单位deg
	 *@return  错误码
	 */
	errno_t  GetActualJointPosDegree(uint8_t flag, JointPos *jPos);

	/**
	 *@brief  获取当前关节位置(弧度)
	 *@param  [in] flag 0-阻塞，1-非阻塞
	 *@param  [out] jPos 六个关节位置，单位rad
	 *@return  错误码
	 */	
	errno_t  GetActualJointPosRadian(uint8_t flag, JointPos *jPos);
	
	/**
	 *@brief  获取当前工具位姿
	 *@param  [in] flag  0-阻塞，1-非阻塞
	 *@param  [out] desc_pos  工具位姿
	 *@return  错误码
	 */
	errno_t  GetActualTCPPose(uint8_t flag, DescPose *desc_pos);
	
	/**
	 *@brief  获取当前工具坐标系编号
	 *@param  [in] flag  0-阻塞，1-非阻塞
	 *@param  [out] id  工具坐标系编号
	 *@return  错误码
	 */
	errno_t  GetActualTCPNum(uint8_t flag, int *id);
	
	/**
	 *@brief  获取当前工件坐标系编号
	 *@param  [in] flag  0-阻塞，1-非阻塞
	 *@param  [out] id  工件坐标系编号
	 *@return  错误码
	 */
	errno_t  GetActualWObjNum(uint8_t flag, int *id);	
	
	/**
	 *@brief  获取当前末端法兰位姿
	 *@param  [in] flag  0-阻塞，1-非阻塞
	 *@param  [out] desc_pos  法兰位姿
	 *@return  错误码
	 */
	errno_t  GetActualToolFlangePose(uint8_t flag, DescPose *desc_pos);	
	
	/**
	 *@brief  逆运动学求解
	 *@param  [in] type 0-绝对位姿(基坐标系)，1-增量位姿(基坐标系)，2-增量位姿(工具坐标系)
	 *@param  [in] desc_pos 笛卡尔位姿
	 *@param  [in] config 关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解
	 *@param  [out] joint_pos 关节位置
	 *@return  错误码
	 */
	errno_t  GetInverseKin(int type, DescPose *desc_pos, int config, JointPos *joint_pos);

	/**
	 *@brief  逆运动学求解，参考指定关节位置求解
	 *@param  [in] desc_pos 笛卡尔位姿
	 *@param  [in] joint_pos_ref 参考关节位置
	 *@param  [out] joint_pos 关节位置
	 *@return  错误码
	 */	
	errno_t  GetInverseKinRef(DescPose *desc_pos, JointPos *joint_pos_ref, JointPos *joint_pos);

	/**
	 *@brief  逆运动学求解，参考指定关节位置判断是否有解
	 *@param  [in] desc_pos 笛卡尔位姿
	 *@param  [in] joint_pos_ref 参考关节位置
	 *@param  [out] result 0-无解，1-有解
	 *@return  错误码
	 */	
	errno_t  GetInverseKinHasSolution(DescPose *desc_pos, JointPos *joint_pos_ref, uint8_t *result);

    /**
	 *@brief  正运动学求解
	 *@param  [in] joint_pos 关节位置
	 *@param  [out] desc_pos 笛卡尔位姿
	 *@return  错误码
	 */
    errno_t  GetForwardKin(JointPos *joint_pos, DescPose *desc_pos);
	
	/**
	 *@brief 获取当前关节转矩
	 *@param  [in] flag 0-阻塞，1-非阻塞
	 *@param  [out] torques 关节转矩
	 *@return  错误码
	 */
	errno_t  GetJointTorques(uint8_t flag, float torques[6]);
	
	/**
	 *@brief  获取当前负载的重量
	 *@param  [in] flag 0-阻塞，1-非阻塞
	 *@param  [out] weight 负载重量，单位kg
	 *@return  错误码
	 */
	errno_t  GetTargetPayload(uint8_t flag, float *weight);
	
	/**
	 *@brief  获取当前负载的质心
	 *@param  [in] flag 0-阻塞，1-非阻塞
	 *@param  [out] cog 负载质心，单位mm
	 *@return  错误码
	 */	
	errno_t  GetTargetPayloadCog(uint8_t flag, DescTran *cog);
	
	/**
	 *@brief  获取当前工具坐标系
	 *@param  [in] flag 0-阻塞，1-非阻塞
	 *@param  [out] desc_pos 工具坐标系位姿
	 *@return  错误码
	 */
	errno_t  GetTCPOffset(uint8_t flag, DescPose *desc_pos);
	
	/**
	 *@brief  获取当前工件坐标系
	 *@param  [in] flag 0-阻塞，1-非阻塞
	 *@param  [out] desc_pos 工件坐标系位姿
	 *@return  错误码
	 */	
	errno_t  GetWObjOffset(uint8_t flag, DescPose *desc_pos);
	
	/**
	 *@brief  获取关节软限位角度
	 *@param  [in] flag 0-阻塞，1-非阻塞	 
	 *@param  [out] negative  负限位角度，单位deg
	 *@param  [out] positive  正限位角度，单位deg
	 *@return  错误码
	 */
	errno_t  GetJointSoftLimitDeg(uint8_t flag, float negative[6], float positive[6]);
	
	/**
	 *@brief  获取系统时间
	 *@param  [out] t_ms 单位ms
	 *@return  错误码
	 */
	errno_t  GetSystemClock(float *t_ms);
	
	/**
	 *@brief  获取机器人当前关节位置
	 *@param  [out]  config  关节空间配置，范围[0~7]
	 *@return  错误码
	 */
	errno_t  GetRobotCurJointsConfig(int *config);

	/**
	 *@brief  获取机器人当前速度
	 *@param  [out]  vel  速度，单位mm/s
	 *@return  错误码
	 */	
	errno_t  GetDefaultTransVel(float *vel);

	/**
	 *@brief  查询机器人运动是否完成
	 *@param  [out]  state  0-未完成，1-完成
	 *@return  错误码
	 */	
	errno_t  GetRobotMotionDone(uint8_t *state);
	
	/**
	 *@brief  设置轨迹记录参数
	 *@param  [in] type  记录数据类型，1-关节位置
	 *@param  [in] name  轨迹文件名
	 *@param  [in] period_ms  数据采样周期，固定值2ms或4ms或8ms
	 *@param  [in] di_choose  DI选择,bit0~bit7对应控制箱DI0~DI7，bit8~bit9对应末端DI0~DI1，0-不选择，1-选择
	 *@param  [in] do_choose  DO选择,bit0~bit7对应控制箱DO0~DO7，bit8~bit9对应末端DO0~DO1，0-不选择，1-选择
	 *@return  错误码
	 */
	errno_t  SetTPDParam(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose);
	
	/**
	 *@brief  开始轨迹记录
	 *@param  [in] type  记录数据类型，1-关节位置
	 *@param  [in] name  轨迹文件名
	 *@param  [in] period_ms  数据采样周期，固定值2ms或4ms或8ms
	 *@param  [in] di_choose  DI选择,bit0~bit7对应控制箱DI0~DI7，bit8~bit9对应末端DI0~DI1，0-不选择，1-选择
	 *@param  [in] do_choose  DO选择,bit0~bit7对应控制箱DO0~DO7，bit8~bit9对应末端DO0~DO1，0-不选择，1-选择
	 *@return  错误码
	 */
	errno_t  SetTPDStart(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose);	
	
	/**
	 *@brief  停止轨迹记录
	 *@return  错误码
	 */
	errno_t  SetWebTPDStop();
	
	/**
	 *@brief  删除轨迹记录
	 *@param  [in] name  轨迹文件名
	 *@return  错误码
	 */	
	errno_t  SetTPDDelete(char name[30]);
	
	/**
	 *@brief  轨迹预加载
	 *@param  [in] name  轨迹文件名
	 *@return  错误码
	 */		
	errno_t  LoadTPD(char name[30]);
	
	/**
	 *@brief  轨迹复现
	 *@param  [in] name  轨迹文件名
	 *@param  [in] blend 0-不平滑，1-平滑
	 *@param  [in] ovl  速度缩放百分比，范围[0~100]
	 *@return  错误码
	 */
	errno_t  MoveTPD(char name[30], uint8_t blend, float ovl);
	
	/**
	 *@brief  设置开机自动加载默认的作业程序
	 *@param  [in] flag  0-开机不自动加载默认程序，1-开机自动加载默认程序
	 *@param  [in] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
	 *@return  错误码
	 */
	errno_t  LoadDefaultProgConfig(uint8_t flag, char program_name[64]);
	
	/**
	 *@brief  加载指定的作业程序
	 *@param  [in] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
	 *@return  错误码
	 */
	errno_t  ProgramLoad(char program_name[64]);
	
	/**
	 *@brief  获取已加载的作业程序名
	 *@param  [out] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
	 *@return  错误码
	 */
	errno_t  GetLoadedProgram(char program_name[64]);	
	
	/**
	 *@brief  获取当前机器人作业程序执行的行号
	 *@param  [out] line  行号
	 *@return  错误码
	 */	
	errno_t  GetCurrentLine(int *line);
	
	/**
	 *@brief  运行当前加载的作业程序
	 *@return  错误码
	 */
	errno_t  ProgramRun();
	
	/**
	 *@brief  暂停当前运行的作业程序
	 *@return  错误码
	 */ 
	errno_t  ProgramPause();
	
	/**
	 *@brief  恢复当前暂停的作业程序
	 *@return  错误码
	 */ 
	errno_t  ProgramResume();	
	
	/**
	 *@brief  终止当前运行的作业程序
	 *@return  错误码
	 */ 
	errno_t  ProgramStop();		
	
	/**
	 *@brief  获取机器人作业程序执行状态
	 *@param  [out]  state 1-程序停止或无程序运行，2-程序运行中，3-程序暂停
	 *@return  错误码
	 */
	errno_t  GetProgramState(uint8_t *state);
	
	/**
	 *@brief  配置夹爪
	 *@param  [in] company  夹爪厂商，待定
	 *@param  [in] device  设备号，暂不使用，默认为0
	 *@param  [in] softvesion  软件版本号，暂不使用，默认为0
	 *@param  [in] bus 设备挂在末端总线位置，暂不使用，默认为0
	 *@return  错误码
	 */
	errno_t  SetGripperConfig(int company, int device, int softvesion, int bus);
	
	/**
	 *@brief  获取夹爪配置
	 *@param  [in] company  夹爪厂商，待定
	 *@param  [in] device  设备号，暂不使用，默认为0
	 *@param  [in] softvesion  软件版本号，暂不使用，默认为0
	 *@param  [in] bus 设备挂在末端总线位置，暂不使用，默认为0
	 *@return  错误码
	 */
	errno_t  GetGripperConfig(int *company, int *device, int *softvesion, int *bus);

	/**
	 *@brief  激活夹爪
	 *@param  [in] index  夹爪编号
	 *@param  [in] act  0-复位，1-激活
	 *@return  错误码
	 */
	errno_t  ActGripper(int index, uint8_t act);

	/**
	 *@brief  控制夹爪
	 *@param  [in] index  夹爪编号
	 *@param  [in] pos  位置百分比，范围[0~100]
	 *@param  [in] vel  速度百分比，范围[0~100]
	 *@param  [in] force  力矩百分比，范围[0~100]
	 *@param  [in] max_time  最大等待时间，范围[0~30000]，单位ms
	 *@param  [in] block  0-阻塞，1-非阻塞
	 *@return  错误码
	 */
	errno_t  MoveGripper(int index, int pos, int vel, int force, int max_time, uint8_t block);

	/**
	 *@brief  获取夹爪运动状态
	 *@param  [out] fault  0-无错误，1-有错误
	 *@param  [out] staus  0-运动未完成，1-运动完成
	 *@return  错误码
	 */
	errno_t  GetGripperMotionDone(uint8_t *fault, uint8_t *status);
	
	/**
	 *@brief  配置力传感器
	 *@param  [in] company  力传感器厂商，17-坤维科技
	 *@param  [in] device  设备号，暂不使用，默认为0
	 *@param  [in] softvesion  软件版本号，暂不使用，默认为0
	 *@param  [in] bus 设备挂在末端总线位置，暂不使用，默认为0
	 *@return  错误码
	 */
	errno_t  FT_SetConfig(int company, int device, int softvesion, int bus);

	/**
	 *@brief  获取力传感器配置
	 *@param  [in] company  力传感器厂商，待定
	 *@param  [in] device  设备号，暂不使用，默认为0
	 *@param  [in] softvesion  软件版本号，暂不使用，默认为0
	 *@param  [in] bus 设备挂在末端总线位置，暂不使用，默认为0
	 *@return  错误码
	 */
	errno_t  FT_GetConfig(int *company, int *device, int *softvesion, int *bus);

	/**
	 *@brief  力传感器激活
	 *@param  [in] act  0-复位，1-激活
	 *@return  错误码
	 */
	errno_t  FT_Activate(uint8_t act);
	
	/**
	 *@brief  力传感器校零
	 *@param  [in] act  0-去除零点，1-零点矫正
	 *@return  错误码
	 */
	errno_t  FT_SetZero(uint8_t act);	

	/**
	 *@brief  设置力传感器参考坐标系
	 *@param  [in] ref  0-工具坐标系，1-基坐标系
	 *@return  错误码
	 */
	errno_t  FT_SetRCS(uint8_t ref);	
	
	/**
	 *@brief  负载重量辨识记录
	 *@param  [in] id  传感器坐标系编号，范围[1~14]
	 *@return  错误码
	 */
	errno_t  FT_PdIdenRecord(int id);	
	
	/**
	 *@brief  负载重量辨识计算
	 *@param  [out] weight  负载重量，单位kg
	 *@return  错误码
	 */	
	errno_t  FT_PdIdenCompute(float *weight);
	
	/**
	 *@brief  负载质心辨识记录
	 *@param  [in] id  传感器坐标系编号，范围[1~14]
	 *@param  [in] index 点编号，范围[1~3]
	 *@return  错误码
	 */
	errno_t  FT_PdCogIdenRecord(int id, int index);		
	
	/**
	 *@brief  负载质心辨识计算
	 *@param  [out] cog  负载质心，单位mm
	 *@return  错误码
	 */	
	errno_t  FT_PdCogIdenCompute(DescTran *cog);	

	/**
	 *@brief  获取参考坐标系下力/扭矩数据
	 *@param  [out] ft  力/扭矩，fx,fy,fz,tx,ty,tz
	 *@return  错误码
	 */	
	errno_t  FT_GetForceTorqueRCS(ForceTorque *ft);	

	/**
	 *@brief  获取力传感器原始力/扭矩数据
	 *@param  [out] ft  力/扭矩，fx,fy,fz,tx,ty,tz
	 *@return  错误码
	 */	
	errno_t  FT_GetForceTorqueOrigin(ForceTorque *ft);	

	/**
	 *@brief  碰撞守护
	 *@param  [in] flag 0-关闭碰撞守护，1-开启碰撞守护
	 *@param  [in] sensor_id 力传感器编号
	 *@param  [in] select  选择六个自由度是否检测碰撞，0-不检测，1-检测
	 *@param  [in] ft  碰撞力/扭矩，fx,fy,fz,tx,ty,tz
	 *@param  [in] max_threshold 最大阈值
	 *@param  [in] min_threshold 最小阈值
	 *@note   力/扭矩检测范围：(ft-min_threshold, ft+max_threshold)
	 *@return  错误码
	 */	
	errno_t  FT_Guard(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float max_threshold[6], float min_threshold[6]);	
	
	/**
	 *@brief  恒力控制
	 *@param  [in] flag 0-关闭恒力控制，1-开启恒力控制
	 *@param  [in] sensor_id 力传感器编号
	 *@param  [in] select  选择六个自由度是否检测碰撞，0-不检测，1-检测
	 *@param  [in] ft  碰撞力/扭矩，fx,fy,fz,tx,ty,tz
	 *@param  [in] ft_pid 力pid参数，力矩pid参数
	 *@param  [in] adj_sign 自适应启停控制，0-关闭，1-开启
	 *@param  [in] ILC_sign ILC启停控制， 0-停止，1-训练，2-实操
	 *@param  [in] 最大调整距离，单位mm
	 *@param  [in] 最大调整角度，单位deg
	 *@return  错误码
	 */	
	errno_t  FT_Control(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float ft_pid[6], uint8_t adj_sign, uint8_t ILC_sign, float max_dis, float max_ang);	

	/**
	 *@brief  螺旋线探索
	 *@param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
	 *@param  [in] dr 每圈半径进给量
	 *@param  [in] ft 力/扭矩阈值，fx,fy,fz,tx,ty,tz，范围[0~100]
	 *@param  [in] max_t_ms 最大探索时间，单位ms
	 *@param  [in] max_vel 最大线速度，单位mm/s
	 *@return  错误码
	 */	
    errno_t  FT_SpiralSearch(int rcs, float dr, float ft, float max_t_ms, float max_vel);	
	
	/**
	 *@brief  旋转插入
	 *@param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
	 *@param  [in] angVelRot 旋转角速度，单位deg/s
	 *@param  [in] ft  力/扭矩阈值，fx,fy,fz,tx,ty,tz，范围[0~100]
	 *@param  [in] max_angle 最大旋转角度，单位deg
	 *@param  [in] orn 力/扭矩方向，1-沿z轴方向，2-绕z轴方向
	 *@param  [in] max_angAcc 最大旋转加速度，单位deg/s^2，暂不使用，默认为0
	 *@param  [in] rotorn  旋转方向，1-顺时针，2-逆时针
	 *@return  错误码
	 */	
    errno_t  FT_RotInsertion(int rcs, float angVelRot, float ft, float max_angle, uint8_t orn, float max_angAcc, uint8_t rotorn);		
	
	/**
	 *@brief  直线插入
	 *@param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
	 *@param  [in] ft  力/扭矩阈值，fx,fy,fz,tx,ty,tz，范围[0~100]
	 *@param  [in] lin_v 直线速度，单位mm/s
	 *@param  [in] lin_a 直线加速度，单位mm/s^2，暂不使用
	 *@param  [in] max_dis 最大插入距离，单位mm
	 *@param  [in] linorn  插入方向，0-负方向，1-正方向
	 *@return  错误码
	 */	
    errno_t  FT_LinInsertion(int rcs, float ft, float lin_v, float lin_a, float max_dis, uint8_t linorn);		

	/**
	 *@brief  表面定位
	 *@param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
     *@param  [in] dir  移动方向，1-正方向，2-负方向 
	 *@param  [in] axis 移动轴，1-x轴，2-y轴，3-z轴
	 *@param  [in] lin_v 探索直线速度，单位mm/s
	 *@param  [in] lin_a 探索直线加速度，单位mm/s^2，暂不使用，默认为0
	 *@param  [in] max_dis 最大探索距离，单位mm
	 *@param  [in] ft  动作终止力/扭矩阈值，fx,fy,fz,tx,ty,tz	 
	 *@return  错误码
	 */	
    errno_t  FT_FindSurface(int rcs, uint8_t dir, uint8_t axis, float lin_v, float lin_a, float max_dis, float ft);	
	
	/**
	 *@brief  计算中间平面位置开始
	 *@return  错误码
	 */	
	errno_t  FT_CalCenterStart();
	
	/**
	 *@brief  计算中间平面位置结束
	 *@param  [out] pos 中间平面位姿
	 *@return  错误码
	 */		
	errno_t  FT_CalCenterEnd(DescPose *pos);
	
	/**
	 *@brief  柔顺控制开启
	 *@param  [in] p 位置调节系数或柔顺系数
	 *@param  [in] force 柔顺开启力阈值，单位N
	 *@return  错误码
	 */	
	errno_t  FT_ComplianceStart(float p, float force);	
	
	/**
	 *@brief  柔顺控制关闭
	 *@return  错误码
	 */	
	errno_t  FT_ComplianceStop();	
	
    /**
	 *@brief  机器人接口类析构函数
	 */    
    ~FRRobot();
};

#endif
