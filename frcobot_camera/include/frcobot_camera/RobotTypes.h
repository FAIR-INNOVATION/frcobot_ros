#ifndef    ROBOTTYPES_H_
#define    ROBOTTYPES_H_

#include <stdio.h>

typedef  int errno_t;
typedef  unsigned char uint8_t;
typedef  unsigned short int  uint16_t;

/**
* @brief Joint position data type
*/
typedef  struct
{
	double jPos[6];   /* Six joint positions, unit: deg */
}JointPos;

/**
* @brief Cartesian spatial location data type
*/
typedef struct
{
	double x;    /* X-axis coordinate, unit: mm  */
	double y;    /* Y-axis coordinate, unit: mm  */
	double z;    /* Z-axis coordinate, unit: mm  */
} DescTran;

/**
* @brief Euler Angle attitude data type
*/
typedef struct
{
	double rx;   /* Rotation Angle about fixed axis X, unit: deg  */
	double ry;   /* Rotation Angle about fixed axis y, unit: deg  */
	double rz;   /* Rotation Angle about fixed axis Z, unit: deg  */
} Rpy;

/**
*@brief Cartesian space pose type
*/
typedef struct
{
	DescTran tran;      /* Cartesian position  */
	Rpy rpy;            /* Cartesian space attitude  */
} DescPose;

/**
* @brief Extension axis position data type
*/
typedef  struct
{
	double ePos[4];   /* Position of four expansion shafts, unit: mm */
}ExaxisPos;

/**
* @brief The force component and torque component of the force sensor
*/
typedef struct
{
	double fx;  /* Component of force along the x axis, unit: N  */
	double fy;  /* Component of force along the y axis, unit: N  */
	double fz;  /* Component of force along the z axis, unit: N  */
	double tx;  /* Component of torque about the X-axis, unit: Nm */
	double ty;  /* Component of torque about the Y-axis, unit: Nm */
	double tz;  /* Component of torque about the Z-axis, unit: Nm */
} ForceTorque;

/**
* @brief  Spiral parameter data type
*/
typedef  struct
{
	int    circle_num;           /* Coil number  */
	float  circle_angle;         /* Spiral Angle  */
	float  rad_init;             /* Initial radius of spiral, unit: mm  */
	float  rad_add;              /* Radius increment  */
	float  rotaxis_add;          /* Increment in the direction of the axis of rotation  */
	unsigned int rot_direction;  /* Rotation direction, 0- clockwise, 1- counterclockwise  */
}SpiralParam;

#endif
