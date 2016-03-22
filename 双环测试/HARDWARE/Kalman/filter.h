#ifndef __FILTER_H
#define __FILTER_H

#define Q_angle 0.01      // 角度数据置信度
#define Q_omega 0.0003    // 角速度数据置信度
#define R_angle 0.01      // 方差噪声
#define  Q_gyro 0.003;

 

float First_order_filter(float Com_angle,float angle_pt,float angle,float dt);// 一阶互补算法
float Second_order_filter(float Com2_angle,float angle_pt,float angle,float dt);// 二阶互补算法
float Kalman_filter(float Klm_angle,float angle_pt,float angle,float dt);// 卡尔曼滤波
void Kalman_Filter_2_(float Gyro,float Accel,float dt);
#endif
