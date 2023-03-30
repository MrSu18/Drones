#ifndef _REMOTE_H
#define _REMOTE_H

#define MIN_THR       800    //遥控器最小油门值
#define MAX_THR       1800   //遥控器最大油门值
#define TAKE_OFF_THR  1400   //能够一键起飞的油门值
#define TAKE_OFF_K    5     //一键起飞和一键降落每6ms的比例系数

void RC_Analy(void);
void remote_unlock(void);

#endif

