/*******************************************************************
 *@title PID 控制函数
 *@brief 包涵的函数有单环，串级，PID初始化，PID相关数据清零
 *@brief 历史修改内容：
 *    
 ******************************************************************/
//========================================================================
//  爱好者电子工作室-淘宝 https://devotee.taobao.com/
//  STM32四轴爱好者QQ群: 810149456
//  作者：小刘
//  电话:13728698082
//  邮箱:1042763631@qq.com
//  日期：2018.05.17
//  版本：V1.0
//========================================================================
//套件购买地址：https://devotee.taobao.com/
//                 爱好者电子工作室
//特此声明：
//
//         此程序只能用作学习，如用商业用途。必追究责任！
//          
//
//
#ifndef __PID_H
#define __PID_H
#include "ALL_DATA.h"
#include <stdbool.h>

extern void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt);  //串级PID
extern void pidRest(PidObject **pid,const uint8_t len); //pid数据复位
extern void pidUpdate(PidObject* pid,const float dt);  //PID
#endif



