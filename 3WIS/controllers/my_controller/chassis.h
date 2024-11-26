#ifndef _CHASSIS_H
#define _CHASSIS_H

#include <webots/motor.h>
#include <webots/robot.h>
#include "sensor.h"

#define M_PI		3.14159265358979323846

#define ANGLE2RAD(x) (x / 180.0f * M_PI) //角度转弧度
#define RAD2ANGLE(x) (x / M_PI * 180.0f) //弧度转角度

#define DIS_WHEEL2CENTER (160.0f / 1000.0f)
#define WHEEL_RADIUS 0.032f
#define SQRT3 1.732

typedef struct
{
    float ct_vx;
    float ct_vy;
    float ct_wz;
}chassis_ct_t;

typedef struct 
{
    float ct_fdb_vx;
    float ct_fdb_vy;
    float ct_fdb_wz;
}chassis_fdb_ct_t;

typedef struct
{
    float angle_actual;
    float rad_actual;
    float angle_target;
    float VN2X; //轮系线速度与X轴的夹角 -180° ~ 180°
    float v_target; //期望
}helm_wheel_t;

typedef enum
{
	GLOBAL_COORDINATE,	//基于全局坐标系控制
	CHASSIS_COORDINATE,	//基于机器人底盘坐标系控制
	LOCK_CHASSIS
}chassis_cor_t;

typedef struct
{
	//速度大小
	float vel;
	//速度方向
	float direction;
	//角速度大小
	float omega;
}robotVel_t;



typedef struct
{
    chassis_ct_t chassis_ct;
    chassis_fdb_ct_t chassis_fdb_ct;
    helm_wheel_t wheel1;
    helm_wheel_t wheel2;
    helm_wheel_t wheel3;
    
    float dir_chassis;

    chassis_cor_t chassis_cor;
}helm_chassis_t;

/** Extern **/
extern helm_chassis_t helm_chassis;

/* Function */
void Chassis_Wb_Init(void);
void Chassis_Self_Spin(void);
int Helm_Chassis_Init(void);
void Helm_Chassis_Ctrl(float vx_input, float vy_input, float wz_input, helm_chassis_t *chassis,  float yaw);
void Helm_Wheel_Ctrl(float vx, float vy, float wz, helm_wheel_t *wheel_ptr);
void Angle_Limit(float *);
void V_Dir2Wheel_Angle(helm_wheel_t *, float);
void Helm_Evaluation(helm_chassis_t *);

#endif