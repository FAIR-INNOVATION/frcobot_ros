#ifndef  FRROBOT_H_
#define  FRROBOT_H_

#include "RobotTypes.h"

class FRRobot
{
public:
    /**
	 *@brief  Robot interface class constructor
	 */
    FRRobot();
    
	/**
    *@brief  Establish communication with the robot controller
    *@param  [in] ip  Controller IP address. The default value is 192.168.58.2
    *@return Error code
	 */
    errno_t  RPC(const char *ip);
	
	/**
    *@brief  Query the SDK version number
    *@param  [out] version  SDK version
    *@return  Error code
     */	 
	errno_t  GetSDKVersion(char *version);
	
	/**
    *@brief  Obtain Controller IP address
    *@param  [out] ip  Controller IP
    *@return  Error code
	 */
	errno_t  GetControllerIP(char *ip);
	
	/**
    *@brief  Control the robot to enter or exit the drag teaching mode
    *@param  [in] state 0-exit drag mode，1-enter the drag mode
    *@return  Error code
	 */
	errno_t  DragTeachSwitch(uint8_t state);
	
	/**
    *@brief  Check whether the robot is in drag mode
    *@param  [out] state 0-non-drag teaching mode，1-drag the teaching mode
    *@return  Error code
	 */
	errno_t  IsInDragTeach(uint8_t *state);
	
	/**
    *@brief  Enable or disable the function on or off the robot. By default, the function is enabled automatically after the robot is powered on
    *@param  [in] state  0-down-enable，1-upper enable
    *@return  Error code
	 */
	errno_t  RobotEnable(uint8_t state);
    
	/**
    *@brief Control robot hand/automatic mode
    *@param [in] mode 0-automatic mode，1-manual mode
    *@return Error code
	 */
    errno_t  Mode(int mode);
	
	/**
    *@brief  Jog point movement
    *@param  [in]  ref 0- node movement, 2- base coordinate system, 4- tool coordinate system, 8- workpiece coordinate system
    *@param  [in]  nb 1-joint 1(or axis x), 2-joint 2(or axis y), 3-joint 3(or axis z), 4-joint 4(or rotation about axis x), 5-joint 5(or rotation about axis y), 6-joint 6(or rotation about axis z)
    *@param  [in]  dir 0-negative correlation, 1-positive correlation
    *@param  [in]  vel The percentage of velocity,[0~100]
    *@param  [in]  acc The percentage of acceleration, [0~100]
    *@param  [in]  max_dis Maximum Angle of single click, unit: [°] or distance, unit: [mm]
    *@return  Error code
	 */
	errno_t  StartJOG(uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis);
	
	/**
    *@brief  Jog point dynamic deceleration stop
    *@param  [in]  ref  1- point stop, 3- point stop in base coordinate system, 5- point stop in tool coordinate system, 9- point stop in workpiece coordinate system
    *@return  Error code
	 */
	errno_t  StopJOG(uint8_t ref);
	
	/**
    *@brief The jog stops immediately
    *@return  Error code
	 */
	errno_t  ImmStopJOG(); 
	
	/**
    *@brief  Joint space motion
    *@param  [in] joint_pos  Target joint location, unit: deg
    *@param  [in] desc_pos   Target Cartesian position
    *@param  [in] tool  Tool coordinate number, range [1~15]
    *@param  [in] user  Workpiece coordinate number, range [1~15]
    *@param  [in] vel  Percentage of speed, range [0~100]
    *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] ovl  Velocity scaling factor, range[0~100]
    *@param  [in] epos  Position of expansion shaft, unit: mm
    *@param  [in] blendT [-1.0]- movement in place (blocking), [0~500.0]- smoothing time (non-blocking), in ms
    *@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    *@param  [in] offset_pos  The pose offset
    *@return  Error code
	 */
	errno_t  MoveJ(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos);
	
	/**
    *@brief  Rectilinear motion in Cartesian space
    *@param  [in] joint_pos  Target joint location, unit: deg
    *@param  [in] desc_pos   Target Cartesian position
    *@param  [in] tool  Tool coordinate number, range [1~15]
    *@param  [in] user  Workpiece coordinate number, range [1~15]
    *@param  [in] vel  Percentage of speed, range [0~100]
    *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] ovl  Velocity scaling factor, range[0~100]
    *@param  [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm    
    *@param  [in] epos  Position of expansion shaft, unit: mm
    *@param  [in] search  0- no wire seeking, 1- wire seeking
    *@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    *@param  [in] offset_pos  The pose offset
    *@return  Error code
	 */	
	errno_t  MoveL(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos);

	/**
    *@brief  Circular arc motion in Cartesian space
    *@param  [in] joint_pos_p  Waypoint joint position, unit: deg
    *@param  [in] desc_pos_p   Waypoint Cartesian position
    *@param  [in] ptool  Tool coordinate number, range [1~15]
    *@param  [in] puser  Workpiece coordinate number, range [1~15]
    *@param  [in] pvel  Percentage of speed, range [0~100]
    *@param  [in] pacc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] epos_p  Position of expansion shaft, unit: mm
    *@param  [in] poffset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    *@param  [in] offset_pos_p  The pose offset
    *@param  [in] joint_pos_t  Target joint position, unit: deg
    *@param  [in] desc_pos_t   Target point Cartesian position
    *@param  [in] ttool  Tool coordinate number, range [1~15]
    *@param  [in] tuser  Workpiece coordinate number, range [1~15]
    *@param  [in] tvel  Percentage of speed, range [0~100]
    *@param  [in] tacc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] epos_t  Position of expansion shaft, unit: mm
    *@param  [in] toffset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    *@param  [in] offset_pos_t  The pose offset   
    *@param  [in] ovl  Velocity scaling factor, range[0~100]    
    *@param  [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm    
    *@return  Error code
	 */		
	errno_t  MoveC(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p,JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t,float ovl, float blendR);
	
	/**
    *@brief  Circular motion in Cartesian space
    *@param  [in] joint_pos_p  Path point 1 joint position, unit: deg
    *@param  [in] desc_pos_p   Waypoint 1 Cartesian position
    *@param  [in] ptool  Tool coordinate number, range [1~15]
    *@param  [in] puser  Workpiece coordinate number, range [1~15]
    *@param  [in] pvel  Percentage of speed, range [0~100]
    *@param  [in] pacc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] epos_p  Position of expansion shaft, unit: mm
    *@param  [in] joint_pos_t  Joint position at waypoint 2, unit: deg
    *@param  [in] desc_pos_t   Waypoint 2 Cartesian position
    *@param  [in] ttool  Tool coordinate number, range [1~15]
    *@param  [in] tuser  Workpiece coordinate number, range [1~15]
    *@param  [in] tvel  Percentage of speed, range [0~100]
    *@param  [in] tacc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] epos_t  Position of expansion shaft, unit: mm
    *@param  [in] ovl  Velocity scaling factor, range[0~100]   
    *@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    *@param  [in] offset_pos  The pose offset     
    *@return  Error code
	 */		
	errno_t  Circle(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, float ovl, uint8_t offset_flag, DescPose *offset_pos);
	
	/**
    *@brief  Spiral motion in Cartesian space
    *@param  [in] joint_pos  Target joint location, unit: deg
    *@param  [in] desc_pos   Target Cartesian position
    *@param  [in] tool  Tool coordinate number, range [1~15]
    *@param  [in] user  Workpiece coordinate number, range [1~15]
    *@param  [in] vel  Percentage of speed, range [0~100]
    *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] epos  Position of expansion shaft, unit: mm
    *@param  [in] ovl  Velocity scaling factor, range[0~100]    
    *@param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    *@param  [in] offset_pos  The pose offset
    *@param  [in] spiral_param  Spiral parameter
    *@return  Error code
	 */
	errno_t  NewSpiral(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, ExaxisPos *epos, float ovl, uint8_t offset_flag, DescPose *offset_pos, SpiralParam spiral_param);	
	
	/**
    *@brief  Joint space servo mode motion
    *@param  [in] joint_pos  Target joint location, unit: deg
    *@param  [in] acc  Acceleration percentage range[0~100], not open yet, default: 0
    *@param  [in] vel  The value ranges from 0 to 100. The value is not available. The default value is 0
    *@param  [in] cmdT Instruction delivery period, unit: s, recommended range [0.001~0.0016]
    *@param  [in] filterT Filtering time (unit: s), temporarily disabled. The default value is 0
    *@param  [in] gain  The proportional amplifier at the target position, not yet open, defaults to 0
    *@return  Error code
	 */
	errno_t  ServoJ(JointPos *joint_pos, float acc, float vel, float cmdT, float filterT, float gain);

	/**
    *@brief  Cartesian space servo mode motion
    *@param  [in]  mode  0- absolute motion (base coordinates), 1- incremental motion (base coordinates), 2- incremental motion (tool coordinates)
    *@param  [in]  desc_pos  Target Cartesian pose or pose increment
    *@param  [in]  pos_gain  Proportional coefficient of pose increment, effective only for incremental motion, range [0~1]
    *@param  [in] acc  Acceleration percentage range[0~100], not open yet, default: 0
    *@param  [in] vel  The value ranges from 0 to 100. The value is not available. The default value is 0
    *@param  [in] cmdT Instruction delivery period, unit: s, recommended range [0.001~0.0016]
    *@param  [in] filterT Filtering time (unit: s), temporarily disabled. The default value is 0
    *@param  [in] gain  The proportional amplifier at the target position, not yet open, defaults to 0
    *@return  Error code
	 */
	errno_t  ServoCart(int mode, DescPose *desc_pose, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain);
   
	/**
    *@brief  Point to point motion in Cartesian space
    *@param  [in]  desc_pos  Target Cartesian pose or pose increment
    *@param  [in] tool  Tool coordinate number, range [1~15]
    *@param  [in] user  Workpiece coordinate number, range [1~15]
    *@param  [in] vel  Percentage of speed, range [0~100]
    *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] ovl  Velocity scaling factor, range[0~100]
    *@param  [in] blendT [-1.0]- movement in place (blocking), [0~500.0]- smoothing time (non-blocking), in ms
    *@param  [in] config  Joint space configuration, [-1]- refer to the current joint position, [0~7]- refer to the specific joint space configuration, the default is -1 
    *@return  Error code
	 */
	errno_t  MoveCart(DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config);
	
	/**
    *@brief  The spline motion begins
    *@return  Error code
	 */
	errno_t  SplineStart();

    /**
    *@brief  Joint space spline movement
    *@param  [in] joint_pos  Target joint location, unit: deg
    *@param  [in] desc_pos   Target Cartesian position
    *@param  [in] tool  Tool coordinate number, range [1~15]
    *@param  [in] user  Workpiece coordinate number, range [1~15]
    *@param  [in] vel  Percentage of speed, range [0~100]
    *@param  [in] acc  Acceleration percentage, range [0~100], not open for now
    *@param  [in] ovl  Velocity scaling factor, range[0~100]   
    *@return  Error code
	 */
	errno_t  SplinePTP(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl);
	
	/**
    *@brief  The spline movement is complete
    *@return  Error code
	 */
	errno_t  SplineEnd();
	
	/**
	 *@brief New spline motion starts
	 *@param  [in] type   0-arc transition, 1-the given point is the path point
	 *@return  Error code
	 */
	errno_t  NewSplineStart(int type);
	
	/**
	 *@brief New Spline Cue Points
	 *@param  [in] joint_pos  Target joint position, unit: deg
	 *@param  [in] desc_pos   Target Cartesian pose
	 *@param  [in] tool  Tool coordinate number, range [1~15]
	 *@param  [in] user  Workpiece coordinate number, range [1~15]
	 *@param  [in] vel  Speed percentage, range [0~100]
	 *@param  [in] acc  Acceleration percentage, range [0~100], not open yet
	 *@param  [in] ovl  Speed scaling factor, range [0~100]
	 *@param  [in] blendR [-1.0]-movement in place (blocking), [0~1000.0]-smooth radius (non-blocking), unit: mm
	 *@return  Error code
	 */	 
	errno_t  NewSplinePoint(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int lastFlag);
	
	/**
	 *@brief New spline motion ends
	 *@return  Error code
	 */
	errno_t  NewSplineEnd();
	
	/**
	 *@brief Stop motion
	 *@return  Error code
	 */
	errno_t  StopMotion();
	
	/**
    *@brief  The whole point shift begins
    *@param  [in]  flag 0- offset in base coordinate system/workpiece coordinate system, 2- offset in tool coordinate system
    *@param  [in] offset_pos  The pose offset
    *@return  Error code
	 */
	errno_t  PointsOffsetEnable(int flag, DescPose *offset_pos);
	
	/**
    *@brief  The whole point shift ends
    *@return  Error code
	 */
	errno_t  PointsOffsetDisable();
	
	/**
    *@brief  Set the control box digital output
    *@param  [in] id  I/O number and range[0~15]
    *@param  [in] status 0- off, 1- on
    *@param  [in] smooth 0- Not smooth, 1- smooth
    *@param  [in] block  0- blocking, 1- non-blocking
    *@return  Error code
	 */
	errno_t  SetDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

	/**
    *@brief  Set tool digital output
    *@param  [in] id  I/O number and range[0~1]
    *@param  [in] status 0- off, 1- on
    *@param  [in] smooth 0- not smooth, 1- smooth
    *@param  [in] block  0- blocking, 1- non-blocking
    *@return  Error code
	 */
	errno_t  SetToolDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

	/**
    *@brief  Set control box analog output
    *@param  [in] id  I/O number and range[0~1]
    *@param  [in] value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@return  Error code
	 */
	errno_t  SetAO(int id, float value, uint8_t block);

	/**
    *@brief  Set tool analog output
    *@param  [in] id  I/O number, range [0]
    *@param  [in] value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@return  Error code
	 */
	errno_t  SetToolAO(int id, float value, uint8_t block);

	/**
    *@brief  Get the control box digital input
    *@param  [in] id  I/O number range[0~15]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@param  [out] result  0- low, 1- high
    *@return  Error code
	 */	
	errno_t  GetDI(int id, uint8_t block, uint8_t *result);

	/**
    *@brief  Get tool numeric input
    *@param  [in] id  I/O number, range[0~1]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@param  [out] result  0- low, 1- high
    *@return  Error code
	 */	
	errno_t  GetToolDI(int id, uint8_t block, uint8_t *result);
	
	/**
    *@brief Wait for the control box digital input
    *@param  [in] id  I/O number，range[0~15]
    *@param  [in]  status 0- off, 1- on
    *@param  [in]  max_time  Maximum waiting time, expressed in ms
    *@param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    *@return  Error code
	 */
	errno_t  WaitDI(int id, uint8_t status, int max_time, int opt);

	/**
    *@brief Wait for control box multiplex digital input
    *@param  [in] mode 0- multiplexed and, 1- multiplexed or
    *@param  [in] id  I/O numbers. bit0 to bit7 corresponds to DI0 to DI7, and bit8 to bit15 corresponds to CI0 to CI7
    *@param  [in]  status 0- off, 1- on
    *@param  [in]  max_time  Maximum waiting time, expressed in ms
    *@param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    *@return  Error code
	 */
	errno_t  WaitMultiDI(int mode, int id, uint8_t status, int max_time, int opt);

	/**
    *@brief Wait for the tool number to enter
    *@param  [in] id  I/O numbers，range[0~1]
    *@param  [in]  status 0- off, 1- on
    *@param  [in]  max_time  Maximum waiting time, expressed in ms
    *@param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    *@return  Error code
	 */
	errno_t  WaitToolDI(int id, uint8_t status, int max_time, int opt);
	
	/**
    *@brief  Get control box analog input
    *@param  [in] id  I/O numbers，range[0~1]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@param  [out] result  Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    *@return  Error code
	 */	
	errno_t  GetAI(int id, uint8_t block, float *result);	

	/**
    *@brief  Get the tool analog input
    *@param  [in] id  I/O numbers，range[0~1]
    *@param  [in] block  0- blocking, 1- non-blocking
    *@param  [out] result  Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    *@return  Error code
	 */	
	errno_t  GetToolAI(int id, uint8_t block, float *result);	
	
	/**
    *@brief Wait for control box analog input
    *@param  [in] id  I/O numbers，range[0~1]
    *@param  [in]  sign 0-greater than，1-less than
    *@param  [in]  value Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    *@param  [in]  max_time Maximum waiting time, expressed in ms
    *@param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    *@return  Error code
	 */
	errno_t  WaitAI(int id, int sign, float value, int max_time, int opt);	
	
	/**
    *@brief Wait for tool analog input
    *@param  [in] id  I/O numbers，range[0~1]
    *@param  [in]  sign 0-greater than，1-less than
    *@param  [in]  value Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    *@param  [in]  max_time  Maximum waiting time, expressed in ms
    *@param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    *@return  Error code
	 */
	errno_t  WaitToolAI(int id, int sign, float value, int max_time, int opt);	

    /**
    *@brief  Set global speed
    *@param  [in]  vel  Percentage of velocity, range[0~100]
    *@return  Error code
	 */
	errno_t  SetSpeed(int vel);
	
	/**
    *@brief  Set the value of a system variable
    *@param  [in]  id  Variable number, range[1~20]
    *@param  [in]  value Variable value
    *@return  Error code
	 */
	errno_t  SetSysVarValue(int id, float value);
	
	/**
    *@brief  Set tool coordinate system
    *@param  [in] id Frame number, range[1~15]
    *@param  [in] coord  Tool center position relative to end flange center position
    *@param  [in] type  0- tool coordinates, 1- sensor coordinates
    *@param  [in] install Installation position, 0- robot end, 1- robot outside
    *@return  Error code
	 */
	errno_t  SetToolCoord(int id, DescPose *coord, int type, int install);
	
	/**
    *@brief  Set the tool coordinate list
    *@param  [in] id Frame number, range[1~15]
    *@param  [in] coord  Tool center position relative to end flange center position
    *@param  [in] type  0- tool coordinates, 1- sensor coordinates
    *@param  [in] install Installation position, 0- robot end, 1- robot outside
    *@return  Error code
	 */
	errno_t  SetToolList(int id, DescPose *coord, int type, int install);	
	
	/**
    *@brief  Set the external tool coordinate system
    *@param  [in] id Frame number, range[1~15]
    *@param  [in] etcp  Tool center position relative to end flange center position
    *@param  [in] etool  To be determined
    *@return  Error code
	 */
	errno_t  SetExToolCoord(int id, DescPose *etcp, DescPose *etool);
	
	/**
    *@brief  Set the list of external tool coordinate systems
    *@param  [in] id Frame number, range[1~15]
    *@param  [in] etcp  Tool center position relative to end flange center position
    *@param  [in] etool  To be determined
    *@return  Error code
	 */
	errno_t  SetExToolList(int id, DescPose *etcp, DescPose *etool);	
	
	/**
	 *@brief  Set workpiece coordinate system
	 *@param  [in] id Coordinate system number, range [1~15]
	 *@param  [in] coord  The workpiece coordinate system relative to the end flange center pose
	 *@return  Error code
     */	 
	errno_t  SetWObjCoord(int id, DescPose *coord);
	
	/**
    *@brief  Set the list of work coordinate systems
    *@param  [in] id Frame number, range[1~15]
    *@param  [in] coord  Tool center position relative to end flange center position
    *@return  Error code
     */	 
	errno_t  SetWObjList(int id, DescPose *coord);	
	
	/**
    *@brief  Set the end load weight
    *@param  [in] weight  Load weight, unit: kg
    *@return  Error code
	 */
	errno_t  SetLoadWeight(float weight);
	
	/**
    *@brief  Set the end-load centroid coordinates
    *@param  [in] coord Centroid coordinates, unit: mm
    *@return  Error code
	 */
	errno_t  SetLoadCoord(DescTran *coord);

	/**
    *@brief  Set the robot installation mode
    *@param  [in] install  Installation mode: 0- formal installation, 1- side installation, 2- inverted installation
    *@return  Error code
	 */
	errno_t  SetRobotInstallPos(uint8_t install);	

	/**
    *@brief  Set the robot installation Angle, free installation
    *@param  [in] yangle  Angle of inclination
    *@param  [in] zangle  Angle of rotation
    *@return  Error code
	 */
	errno_t  SetRobotInstallAngle(double yangle, double zangle);	

	/**
    *@brief  Wait for the specified time
    *@param  [in]  t_ms  unit: ms
    *@return  Error code
	 */
	errno_t  WaitMs(int t_ms);
	
	/**
    *@brief Set collision level
    *@param  [in]  mode  0- grade, 1- percentage
    *@param  [in]  level Collision threshold, grade range [], percentage range [0~1]
    *@param  [in]  config 0- Do not update the configuration file. 1- Update the configuration file
    *@return  Error code
	 */
	errno_t  SetAnticollision(int mode, float level[6], int config);
	
	/**
    *@brief  Set the post-collision policy
    *@param  [in] strategy  0- Error stop, 1- Continue running
    *@return  Error code   
	 */
	errno_t  SetCollisionStrategy(int strategy);
	
	/**
    *@brief  Set the positive limit
    *@param  [in] limit Six joint positions, unit: deg
    *@return  Error code
	 */
	errno_t  SetLimitPositive(float limit[6]);
	
	/**
    *@brief  Set the negative limit
    *@param  [in] limit Six joint positions, unit: deg
    *@return  Error code
	 */
	errno_t  SetLimitNegative(float limit[6]);	
	
	/**
    *@brief  Error status clearing
    *@return  Error code
	 */
	errno_t  ResetAllError();
	
	/**
    *@brief  Joint friction compensation switch
    *@param  [in]  state  0- off, 1- on
    *@return  Error code
	 */
	errno_t  FrictionCompensationOnOff(uint8_t state);
	
	/**
    *@brief  Set joint friction compensation coefficient - formal
    *@param  [in]  coeff Six joint compensation coefficients, range [0~1]
    *@return  Error code
	 */
	errno_t  SetFrictionValue_level(float coeff[6]);

	/**
    *@brief  Set joint friction compensation coefficient - side mount
    *@param  [in]  coeff Six joint compensation coefficients, range [0~1]
    *@return  Error code
	 */
	errno_t  SetFrictionValue_wall(float coeff[6]);

	/**
    *@brief  Set joint friction compensation coefficient - inverted
    *@param  [in]  coeff Six joint compensation coefficients, range [0~1]
    *@return  Error code
	 */
	errno_t  SetFrictionValue_ceiling(float coeff[6]);

	/**
    *@brief  Set joint friction compensation coefficient - free mount
    *@param  [in]  coeff Six joint compensation coefficients, range [0~1]
    *@return  Error code
	 */
	errno_t  SetFrictionValue_freedom(float coeff[6]);
	
	/**
    *@brief  Obtain robot mounting Angle
    *@param  [out] yangle Angle of inclination
    *@param  [out] zangle Angle of rotation
    *@return  Error code
	 */
	errno_t  GetRobotInstallAngle(float *yangle, float *zangle);
	
	/**
    *@brief  Get the system variable value
    *@param  [in] id System variable number, range[1~20]
    *@param  [out] value  System variable value
    *@return  Error code
	 */
	errno_t  GetSysVarValue(int id, float *value);
	
	/**
    *@brief  Get the current joint position (Angle)
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] jPos Six joint positions, unit: deg
    *@return  Error code
	 */
	errno_t  GetActualJointPosDegree(uint8_t flag, JointPos *jPos);

	/**
    *@brief  Get the current joint position (radians)
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] jPos Six joint positions, unit: rad
    *@return  Error code
	 */	
	errno_t  GetActualJointPosRadian(uint8_t flag, JointPos *jPos);

	/**
	 *@brief  Get joint feedback velocity-deg/s
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] speed [x,y,z,rx,ry,rz] speed
	 *@return  Error code 
	 */	
	errno_t  GetActualJointSpeedsDegree(uint8_t flag, float speed[6]);

	/**
	 *@brief  Get joint feedback velocity-rad/s
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] speed [x,y,z,rx,ry,rz] speed
	 *@return  Error code
	 */		
	errno_t  GetActualJointSpeedsRadian(uint8_t flag, float speed[6]);

	/**
	 *@brief  Get TCP instruction speed
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] tcp_speed linear speed
	 *@param  [out] ori_speed attitude speed
	 *@return  Error code 
	 */
	errno_t  GetTargetTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed);

	/**
	 *@brief  Get TCP instruction speed
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] tcp_speed linear speed
	 *@param  [out] ori_speed attitude speed
	 *@return  Error code 
	 */	
	errno_t  GetActualTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed);

	/**
	 *@brief  Get TCP instruction speed
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] speed [x,y,z,rx,ry,rz] speed
	 *@return  Error code  
	 */	
	errno_t  GetTargetTCPSpeed(uint8_t flag, float speed[6]);

	/**
	 *@brief  Get TCP Feedback Speed
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] speed [x,y,z,rx,ry,rz] speed
	 *@return  Error code  
	 */	
	errno_t  GetActualTCPSpeed(uint8_t flag, float speed[6]);
	
	/**
    *@brief  Get the current tool pose
    *@param  [in] flag  0- blocking, 1- non-blocking
    *@param  [out] desc_pos  Tool position
    *@return  Error code
	 */
	errno_t  GetActualTCPPose(uint8_t flag, DescPose *desc_pos);
	
	/**
    *@brief  Get the current tool coordinate system number
    *@param  [in] flag  0- blocking, 1- non-blocking
    *@param  [out] id  Tool coordinate system number
    *@return  Error code
	 */
	errno_t  GetActualTCPNum(uint8_t flag, int *id);
	
	/**
    *@brief  Get the current workpiece coordinate system number
    *@param  [in] flag  0- blocking, 1- non-blocking
    *@param  [out] id  Job coordinate system number
    *@return  Error code
	 */
	errno_t  GetActualWObjNum(uint8_t flag, int *id);	
	
	/**
    *@brief  Get the current end flange pose
    *@param  [in] flag  0- blocking, 1- non-blocking
    *@param  [out] desc_pos  Flange pose
    *@return  Error code
	 */
	errno_t  GetActualToolFlangePose(uint8_t flag, DescPose *desc_pos);	
	
	/**
    *@brief  Inverse kinematics solution
    *@param  [in] type 0- absolute pose (base frame), 1- incremental pose (base frame), 2- incremental pose (tool frame)
    *@param  [in] desc_pos Cartesian pose
    *@param  [in] config Joint space configuration, [-1]- based on the current joint position, [0~7]- based on the specific joint space configuration
    *@param  [out] joint_pos Joint position
    *@return  Error code
	 */
	errno_t  GetInverseKin(int type, DescPose *desc_pos, int config, JointPos *joint_pos);

	/**
    *@brief  Inverse kinematics is solved by referring to the specified joint position
    *@param  [in] desc_pos Cartesian pose
    *@param  [in] joint_pos_ref Reference joint position
    *@param  [out] joint_pos Joint position
    *@return  Error code
	 */	
	errno_t  GetInverseKinRef(DescPose *desc_pos, JointPos *joint_pos_ref, JointPos *joint_pos);

	/**
	 *@brief  逆运动学求解，参考指定关节位置判断是否有解
    *@param  [in] desc_pos Cartesian pose
    *@param  [in] joint_pos_ref Reference joint position
    *@param  [out] result 0- no solution, 1-solution
    *@return  Error code
	 */	
	errno_t  GetInverseKinHasSolution(int type, DescPose *desc_pos, JointPos *joint_pos_ref, uint8_t *result);

    /**
    *@brief  Forward kinematics solution
    *@param  [in] joint_pos Joint position
    *@param  [out] desc_pos Cartesian pose
    *@return  Error code
	 */
    errno_t  GetForwardKin(JointPos *joint_pos, DescPose *desc_pos);
	
	/**
    *@brief Obtain the current joint torque
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] torques Joint torque
    *@return  Error code
	 */
	errno_t  GetJointTorques(uint8_t flag, float torques[6]);
	
	/**
    *@brief  Gets the weight of the current load
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] weight Load weight, unit: kg
    *@return  Error code
	 */
	errno_t  GetTargetPayload(uint8_t flag, float *weight);
	
	/**
    *@brief  Get the center of mass of the current load
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] cog Load center of mass, unit: mm
    *@return  Error code
	 */	
	errno_t  GetTargetPayloadCog(uint8_t flag, DescTran *cog);
	
	/**
    *@brief  Get the current tool coordinate system
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] desc_pos Tool coordinate position
    *@return  Error code
	 */
	errno_t  GetTCPOffset(uint8_t flag, DescPose *desc_pos);
	
	/**
    *@brief  Get the current work frame
    *@param  [in] flag 0- blocking, 1- non-blocking
    *@param  [out] desc_pos Position of workpiece coordinate system
    *@return  Error code
	 */	
	errno_t  GetWObjOffset(uint8_t flag, DescPose *desc_pos);
	
	/**
    *@brief  Obtain joint soft limit Angle
    *@param  [in] flag 0- blocking, 1- non-blocking    
    *@param  [out] negative  Negative limit Angle, unit: deg
    *@param  [out] positive  Positive limit Angle, unit: deg
    *@return  Error code
	 */
	errno_t  GetJointSoftLimitDeg(uint8_t flag, float negative[6], float positive[6]);
	
	/**
    *@brief  Get system time
    *@param  [out] t_ms unit: ms
    *@return  Error code
	 */
	errno_t  GetSystemClock(float *t_ms);
	
	/**
    *@brief  Get the current joint configuration of the robot
    *@param  [out]  config  Joint space configuration, range [0~7]
    *@return  Error code
	 */
	errno_t  GetRobotCurJointsConfig(int *config);

	/**
    *@brief  Get the robot's current speed
    *@param  [out]  vel  The unit is mm/s
    *@return  Error code
	 */	
	errno_t  GetDefaultTransVel(float *vel);

	/**
    *@brief  Query whether the robot movement is complete
    *@param  [out]  state  0- Incomplete, 1- completed
    *@return  Error code
	 */	
	errno_t  GetRobotMotionDone(uint8_t *state);
	
	/**
    *@brief  Set track recording parameters
    *@param  [in] type  Record data type, 1- joint position
    *@param  [in] name  Track file name
    *@param  [in] period_ms  Data sampling period, fixed value 2ms or 4ms or 8ms
    *@param  [in] di_choose  DI Select,bit0 to bit7 corresponds to control box DI0 to DI7, bit8 to bit9 corresponds to end DI0 to DI1, 0- do not select, 1- select
    *@param  [in] do_choose  DO select,bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0- do not select, 1- select
    *@return  Error code
	 */
	errno_t  SetTPDParam(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose);
	
	/**
    *@brief  Start track recording
    *@param  [in] type  Record data type, 1- joint position
    *@param  [in] name  Track file name
    *@param  [in] period_ms  Data sampling period, fixed value 2ms or 4ms or 8ms
    *@param  [in] di_choose  DI Select,bit0 to bit7 corresponds to control box DI0 to DI7, bit8 to bit9 corresponds to end DI0 to DI1, 0- do not select, 1- select
    *@param  [in] do_choose  DO select,bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0- do not select, 1- select
    *@return  Error code
	 */
	errno_t  SetTPDStart(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose);	
	
	/**
    *@brief  Stop track recording
    *@return  Error code
	 */
	errno_t  SetWebTPDStop();
	
	/**
    *@brief  Delete track record
    *@param  [in] name  Track file name
    *@return  Error code
	 */	
	errno_t  SetTPDDelete(char name[30]);
	
	/**
    *@brief  Trajectory preloading
    *@param  [in] name  Track file name
    *@return  Error code
	 */		
	errno_t  LoadTPD(char name[30]);
	
	/**
    *@brief  Trajectory recurrence
    *@param  [in] name  Track file name
    *@param  [in] blend 0- not smooth, 1- smooth
    *@param  [in] ovl  Speed scaling percentage, range [0~100]
    *@return  Error code
	 */
	errno_t  MoveTPD(char name[30], uint8_t blend, float ovl);
	
	/**
    *@brief  Set the default job program to be automatically loaded upon startup
    *@param  [in] flag  0- boot does not automatically load the default program, 1- boot automatically load the default program
    *@param  [in] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    *@return  Error code
	 */
	errno_t  LoadDefaultProgConfig(uint8_t flag, char program_name[64]);
	
	/**
    *@brief  Load the specified job program
    *@param  [in] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    *@return  Error code
	 */
	errno_t  ProgramLoad(char program_name[64]);
	
	/**
    *@brief  Get the loaded job program name
    *@param  [out] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    *@return  Error code
	 */
	errno_t  GetLoadedProgram(char program_name[64]);	
	
	/**
    *@brief  Get the line number of the current robot job program
    *@param  [out] line  line number
    *@return  Error code
	 */	
	errno_t  GetCurrentLine(int *line);
	
	/**
    *@brief  Run the currently loaded job program
    *@return  Error code
	 */
	errno_t  ProgramRun();
	
	/**
    *@brief  Pause the current running job program
    *@return  Error code
	 */ 
	errno_t  ProgramPause();
	
	/**
    *@brief  Resume the currently suspended job program
    *@return  Error code
	 */ 
	errno_t  ProgramResume();	
	
	/**
    *@brief  Terminates the currently running job program
    *@return  Error code
	 */ 
	errno_t  ProgramStop();		
	
	/**
    *@brief  Get the robot job program execution state
    *@param  [out]  state 1- program stop or no program running, 2- program running, 3- program pause
    *@return  Error code
	 */
	errno_t  GetProgramState(uint8_t *state);
	
	/**
    *@brief  Configure the gripper
    *@param  [in] company  Claw manufacturer, to be determined
    *@param  [in] device  Device number, not used yet. The default value is 0
    *@param  [in] softvesion  Software version. The value is not used. The default value is 0
    *@param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    *@return  Error code
	 */
	errno_t  SetGripperConfig(int company, int device, int softvesion, int bus);
	
	/**
    *@brief  Configure the gripper
    *@param  [in] company  Claw manufacturer, to be determined
    *@param  [in] device  Device number, not used yet. The default value is 0
    *@param  [in] softvesion  Software version. The value is not used. The default value is 0
    *@param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    *@return  Error code
	 */
	errno_t  GetGripperConfig(int *company, int *device, int *softvesion, int *bus);

	/**
    *@brief  Activate Activate gripper
    *@param  [in] index  gripper gripper
    *@param  [in] act  0- reset, 1- activate
    *@return  Error code
	 */
	errno_t  ActGripper(int index, uint8_t act);

	/**
    *@brief  Control gripper
    *@param  [in] index  gripper number
    *@param  [in] pos  Percentage of position, range[0~100]
    *@param  [in] vel  Percentage of velocity, range[0~100]
    *@param  [in] force  Percentage of torque, range[0~100]
    *@param  [in] max_time  Maximum wait time, range[0~30000], unit: ms
    *@param  [in] block  0- blocking, 1- non-blocking
    *@return  Error code
	 */
	errno_t  MoveGripper(int index, int pos, int vel, int force, int max_time, uint8_t block);

	/**
    *@brief  Obtain the gripper motion state
    *@param  [out] fault  0- no error, 1- error
    *@param  [out] staus  0- motion incomplete, 1- motion complete
    *@return  Error code
	 */
	errno_t  GetGripperMotionDone(uint8_t *fault, uint8_t *status);

	/**
	 *@brief  Computing Prefetch Points - Vision
	 *@param  [in] desc_pos  Grab point Cartesian pose
	 *@param  [in] zlength   z-axis offset
	 *@param  [in] zangle    Rotate the offset around the z-axis
	 *@return  Error code 
	 */
	errno_t  ComputePrePick(DescPose *desc_pos, double zlength, double zangle, DescPose *pre_pos);

	/**
	 *@brief  Calculating Retreat Points - Visual
	 *@param  [in] desc_pos  Grab point Cartesian pose
	 *@param  [in] zlength   z axis offset
	 *@param  [in] zangle    Rotate the offset around the z-axis
	 *@return  Error code 
	 */
	errno_t  ComputePostPick(DescPose *desc_pos, double zlength, double zangle, DescPose *post_pos);

	/**
    *@brief  Configured force sensor
    *@param  [in] company  Manufacturer of force sensors, 17-Kunwei Technology
    *@param  [in] device  Device number, not used yet. The default value is 0
    *@param  [in] softvesion  Software version. The value is not used. The default value is 0
    *@param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    *@return  Error code
	 */
	errno_t  FT_SetConfig(int company, int device, int softvesion, int bus);

	/**
    *@brief  Get the force sensor configuration
    *@param  [in] company  Force sensor manufacturer, to be determined
    *@param  [in] device  Device number, not used yet. The default value is 0
    *@param  [in] softvesion  Software version. The value is not used. The default value is 0
    *@param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    *@return  Error code
	 */
	errno_t  FT_GetConfig(int *company, int *device, int *softvesion, int *bus);

	/**
    *@brief  Force sensor activation
    *@param  [in] act  0- reset, 1- activate
    *@return  Error code
	 */
	errno_t  FT_Activate(uint8_t act);
	
	/**
    *@brief  Force sensor calibration
    *@param  [in] act  0- zero removal, 1- zero correction
    *@return  Error code
	 */
	errno_t  FT_SetZero(uint8_t act);	

	/**
    *@brief  Set the reference coordinate system of the force sensor
    *@param  [in] ref  0- tool frame, 1- base frame
    *@return  Error code
	 */
	errno_t  FT_SetRCS(uint8_t ref);	
	
	/**
    *@brief  Load weight identification record
    *@param  [in] id  Sensor coordinate system number, range [1~14]
    *@return  Error code
	 */
	errno_t  FT_PdIdenRecord(int id);	
	
	/**
    *@brief  Load weight identification calculation
    *@param  [out] weight  Load weight, unit: kg
    *@return  Error code
	 */	
	errno_t  FT_PdIdenCompute(float *weight);
	
	/**
    *@brief  Load centroid identification record
    *@param  [in] id  Sensor coordinate system number, range [1~14]
    *@param  [in] index Point number, range [1~3]
    *@return  Error code
	 */
	errno_t  FT_PdCogIdenRecord(int id, int index);		
	
	/**
    *@brief  Load centroid identification calculation
    *@param  [out] cog  Load center of mass, unit: mm
    *@return  Error code
	 */	
	errno_t  FT_PdCogIdenCompute(DescTran *cog);	

	/**
	 *@brief  Obtain force/torque data in the reference coordinate system
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
	 *@param  [out] ft  Force/torque，fx,fy,fz,tx,ty,tz
	 *@return  Error code
	 */	
	errno_t  FT_GetForceTorqueRCS(uint8_t flag, ForceTorque *ft);	

	/**
	 *@brief  Obtain the raw force/torque data of the force sensor
	 *@param  [in] flag 0 - blocking, 1 - non-blocking
    *@param  [out] ft  Force/torque，fx,fy,fz,tx,ty,tz
    *@return  Error code
	 */	
	errno_t  FT_GetForceTorqueOrigin(uint8_t flag, ForceTorque *ft);	

	/**
    *@brief  Collision guard
    *@param  [in] flag 0- Disable collision guard. 1- Enable collision guard
    *@param  [in] sensor_id Force sensor number
    *@param  [in] select  Select the six degrees of freedom whether to detect collision, 0- no detection, 1- detection
    *@param  [in] ft  Impact force/torque，fx,fy,fz,tx,ty,tz
    *@param  [in] max_threshold Maximum threshold
    *@param  [in] min_threshold Minimum threshold
    *@note   Force/torque detection range：(ft-min_threshold, ft+max_threshold)
    *@return  Error code
	 */	
	errno_t  FT_Guard(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float max_threshold[6], float min_threshold[6]);	
	
	/**
    *@brief  Constant force control
    *@param  [in] flag 0- turn off constant force control, 1- turn on constant force control
    *@param  [in] sensor_id Force sensor number
    *@param  [in] select  Select the six degrees of freedom whether to detect collision, 0- no detection, 1- detection
    *@param  [in] ft  Impact force/torque，fx,fy,fz,tx,ty,tz
    *@param  [in] ft_pid Force pid parameter, torque pid parameter
    *@param  [in] adj_sign Adaptive start-stop control, 0- off, 1- on
    *@param  [in] ILC_sign ILC start stop control, 0- stop, 1- training, 2- operation
    *@param  [in] Maximum Adjustment distance, unit: mm
    *@param  [in] Maximum Adjustment Angle, unit: deg
    *@return  Error code
	 */	
	errno_t  FT_Control(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float ft_pid[6], uint8_t adj_sign, uint8_t ILC_sign, float max_dis, float max_ang);	

	/**
    *@brief  Spiral exploration
    *@param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    *@param  [in] dr Feed per circle radius
    *@param  [in] ft Force/torque threshold，fx,fy,fz,tx,ty,tz，range[0~100]
    *@param  [in] max_t_ms Maximum exploration time, unit: ms
    *@param  [in] max_vel Maximum linear velocity, unit: mm/s
    *@return  Error code
	 */	
    errno_t  FT_SpiralSearch(int rcs, float dr, float ft, float max_t_ms, float max_vel);	
	
	/**
    *@brief  Rotary insertion
    *@param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    *@param  [in] angVelRot Angular velocity of rotation, unit: deg/s
    *@param  [in] ft  Force/torque threshold，fx,fy,fz,tx,ty,tz，range[0~100]
    *@param  [in] max_angle Maximum rotation Angle, unit: deg
    *@param  [in] orn Force/torque direction, 1- along the z axis, 2- around the z axis
    *@param  [in] max_angAcc Maximum rotational acceleration, in deg/s^2, not used yet, default is 0
    *@param  [in] rotorn  Rotation direction, 1- clockwise, 2- counterclockwise
    *@return  Error code
	 */	
    errno_t  FT_RotInsertion(int rcs, float angVelRot, float ft, float max_angle, uint8_t orn, float max_angAcc, uint8_t rotorn);		
	
	/**
    *@brief  Linear insertion
    *@param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    *@param  [in] ft  Force/torque threshold，fx,fy,fz,tx,ty,tz，range[0~100]
    *@param  [in] lin_v Linear velocity, unit: mm/s
    *@param  [in] lin_a Linear acceleration, unit: mm/s^2, not used yet
    *@param  [in] max_dis Maximum insertion distance, unit: mm
    *@param  [in] linorn  Insert direction, 0- negative, 1- positive
    *@return  Error code
	 */	
    errno_t  FT_LinInsertion(int rcs, float ft, float lin_v, float lin_a, float max_dis, uint8_t linorn);		

	/**
    *@brief  Surface positioning
    *@param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    *@param  [in] dir  The direction of travel, 1- positive, 2- negative
    *@param  [in] axis Axis of movement, 1-x axis, 2-y axis, 3-z axis
    *@param  [in] lin_v Explore the linear velocity in mm/s
    *@param  [in] lin_a Explore linear acceleration, in mm/s^2, not used yet, default to 0
    *@param  [in] max_dis Maximum exploration distance, in mm
    *@param  [in] ft  Action termination force/torque threshold，fx,fy,fz,tx,ty,tz  
    *@return  Error code
	 */	
    errno_t  FT_FindSurface(int rcs, uint8_t dir, uint8_t axis, float lin_v, float lin_a, float max_dis, float ft);	
	
	/**
    *@brief  Calculation of midplane position starts
    *@return  Error code
	 */	
	errno_t  FT_CalCenterStart();
	
	/**
    *@brief  Calculation of midplane position ends
    *@param  [out] pos Intermediate plane position
    *@return  Error code
	 */		
	errno_t  FT_CalCenterEnd(DescPose *pos);
	
	/**
    *@brief  Compliant control on
    *@param  [in] p Coefficient of position adjustment or compliance
    *@param  [in] force Compliant opening force threshold, unit: N
    *@return  Error code
	 */	
	errno_t  FT_ComplianceStart(float p, float force);	
	
	/**
    *@brief  Compliant control off
    *@return  Error code
	 */	
	errno_t  FT_ComplianceStop();	
	
    /**
	 *@brief  Robot interface class constructor
	 */    
    ~FRRobot();

private:
	char serverUrl[64];
};

#endif