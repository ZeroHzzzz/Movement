// /*
//  * Stack.c
//  *
//  *  Created on: 2020Äê11ÔÂ12ÈÕ
//  *      Author: wangjc
//  */


// #include "Stack.h"


// void InitStack(Stack *s,uint32 max,Point *data)
// {
//     s->MAX = max;
//     s->top = -1;
//     s->data = data;
// }


// uint8 FullStack(Stack * s)
// {
//     return ((s->top)+1)==(s->MAX);

// }

// uint8 EmptyStack(Stack *s)
// {
//     return (s->top) == -1;
// }

// uint8 PushStack(Stack * s, uint8 x, uint8 y)
// {
//     if(FullStack(s))return 0;
//     Point a;
//     a.x = x;
//     a.y = y;
//     s->data[s->top+1] = a;
//     s->top = s->top+1;
//     return 1;
// }

// Point PopStack(Stack *s)
// {
//     if(EmptyStack(s))
//     {
//         Point a;
//         a.x = 0xff;a.y = 0xff;
//         return a;
//     }
//     Point ret = s->data[s->top];
//     s->top = s->top-1;
//     return ret;
// }
