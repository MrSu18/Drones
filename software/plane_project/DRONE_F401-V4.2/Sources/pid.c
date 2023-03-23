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
#include "pid.h"
#include "myMath.h"  

/**************************************************************
 *批量复位PID函数
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/  
void pidRest(PidObject **pid,const uint8_t len)
{
  uint8_t i;
  for(i=0;i<len;i++)
  {
      pid[i]->integ = 0;
      pid[i]->prevError = 0;
      pid[i]->out = 0;
    pid[i]->offset = 0;
  }
}

/**************************************************************
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 ***************************************************************/  
void pidUpdate(PidObject* pid,const float dt)
{
   float error;
   float deriv;
  
    error = pid->desired - pid->measured; //当前角度与实际角度的误差

    pid->integ += error * dt;   //误差积分累加值
  
  //  pid->integ = LIMIT(pid->integ,pid->IntegLimitLow,pid->IntegLimitHigh); //进行积分限幅

    deriv = (error - pid->prevError)/dt;  //前后两次误差做微分
  
    pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;//PID输出
  
    //pid->out = LIMIT(pid->out,pid->OutLimitLow,pid->OutLimitHigh); //输出限幅
    
    pid->prevError = error;  //更新上次的误差
    
}

/**************************************************************
 *  CascadePID
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/  
void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt)  //串级PID
{   
  pidUpdate(pidAngE,dt);    //先计算外环
  pidRate->desired = pidAngE->out;//把角度环的输出值给角速度环的期望值
  pidUpdate(pidRate,dt);    //再计算内环  
}

/*******************************END*********************************/



