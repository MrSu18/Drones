/*******************************************************************
 *@title PID ���ƺ���
 *@brief �����ĺ����е�����������PID��ʼ����PID�����������
 *@brief ��ʷ�޸����ݣ�
 *    
 ******************************************************************/
//========================================================================
//  �����ߵ��ӹ�����-�Ա� https://devotee.taobao.com/
//  STM32���ᰮ����QQȺ: 810149456
//  ���ߣ�С��
//  �绰:13728698082
//  ����:1042763631@qq.com
//  ���ڣ�2018.05.17
//  �汾��V1.0
//========================================================================
//�׼������ַ��https://devotee.taobao.com/
//                 �����ߵ��ӹ�����
//�ش�������
//
//         �˳���ֻ������ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#ifndef __PID_H
#define __PID_H
#include "ALL_DATA.h"
#include <stdbool.h>

extern void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt);  //����PID
extern void pidRest(PidObject **pid,const uint8_t len); //pid���ݸ�λ
extern void pidUpdate(PidObject* pid,const float dt);  //PID
#endif



