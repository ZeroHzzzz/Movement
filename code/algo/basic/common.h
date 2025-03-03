/*
 * common.h
 *
 *  Created on: 2023年9月6日
 *      Author: zhu
 */

#ifndef CODE_COMMON_H_
#define CODE_COMMON_H_
typedef struct //定义(整形)点坐标结构体
{
    int16 x;
    int16 y;
}Point;

typedef struct       //定义(浮点数)点坐标结构体
{
    float x;
    float y;
} PointF;

void restrictValueI(int32 *value, int32 max, int32 min);
void restrictValueF(float *value, float max, float min);

#endif /* CODE_COMMON_H_ */
