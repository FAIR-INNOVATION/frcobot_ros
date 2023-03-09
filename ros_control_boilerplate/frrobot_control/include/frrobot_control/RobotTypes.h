#ifndef    ROBOTTYPES_H_
#define    ROBOTTYPES_H_

#include <stdio.h>

typedef  int errno_t;
typedef  unsigned char uint8_t;
typedef  unsigned short int  uint16_t;

/**
 * @brief 关节位置数据类型
 */
typedef  struct
{
	double jPos[6];   /* 六个关节位置，单位deg */
}JointPos;

/**
* @brief 笛卡尔空间位置数据类型
*/
typedef struct
{
	double x;    /* x轴坐标，单位mm  */
	double y;    /* y轴坐标，单位mm  */
	double z;    /* z轴坐标，单位mm  */
} DescTran;

/**
* @brief 欧拉角姿态数据类型
*/
typedef struct
{
	double rx;   /* 绕固定轴X旋转角度，单位：deg  */
	double ry;   /* 绕固定轴Y旋转角度，单位：deg  */
	double rz;   /* 绕固定轴Z旋转角度，单位：deg  */
} Rpy;

/**
 *@brief 笛卡尔空间位姿类型
 */
typedef struct
{
	DescTran tran;      /* 笛卡尔空间位置  */
	Rpy rpy;			/* 笛卡尔空间姿态  */
} DescPose;

/**
 * @brief 扩展轴位置数据类型
 */
typedef  struct
{
	double ePos[4];   /* 四个扩展轴位置，单位mm */
}ExaxisPos;

/**
 * @brief 力传感器的受力分量和力矩分量
 */
typedef struct
{
	double fx;  /* 沿x轴受力分量，单位N  */
	double fy;  /* 沿y轴受力分量，单位N  */
	double fz;  /* 沿z轴受力分量，单位N  */
	double tx;  /* 绕x轴力矩分量，单位Nm */
	double ty;  /* 绕y轴力矩分量，单位Nm */
	double tz;  /* 绕z轴力矩分量，单位Nm */
} ForceTorque;


/**
 * @brief  螺旋参数数据类型
 */
typedef  struct
{
	int    circle_num;           /* 螺旋圈数  */
	float  circle_angle;         /* 螺旋倾角  */
	float  rad_init;             /* 螺旋初始半径，单位mm  */
	float  rad_add;              /* 半径增量  */
	float  rotaxis_add;          /* 转轴方向增量  */
	unsigned int rot_direction;  /* 旋转方向，0-顺时针，1-逆时针  */
}SpiralParam;

#endif
