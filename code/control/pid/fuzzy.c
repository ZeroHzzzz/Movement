// /*
//  * control.c
//  *
//  *  Created on: 2023年11月8日
//  *      Author: symc
//  */
// #include "zf_common_headfile.h"
// #include "fuzzy.h"
// //TODO:reset the fuzzy rule
// //TODO:fix fuzzy control
// FuzzySetType fuzzyRuleKp[7][7]={
//     {PB,PB,PM,PM,PS,PS,ZO},
//     {PB,PB,PM,PM,PS,ZO,ZO},
//     {PM,PM,PM,PS,ZO,NS,NM},
//     {PM,PS,PS,ZO,NS,NM,NM},
//     {PS,PS,ZO,NS,NS,NM,NM},
//     {ZO,ZO,NS,NM,NM,NM,NB},
//     {ZO,NS,NS,NM,NM,NB,NB},
// };
// FuzzySetType fuzzyRuleKi[7][7]={
//     {NB,NB,NB,NM,NM,ZO,ZO},
//     {NB,NB,NM,NM,NS,ZO,ZO},
//     {NM,NM,NS,NS,ZO,PS,PS},
//     {NM,NS,NS,ZO,PS,PS,PM},
//     {NS,NS,ZO,PS,PS,PM,PM},
//     {ZO,ZO,PS,PM,PM,PB,PB},
//     {ZO,ZO,PS,PM,PB,PB,PB},
// };
// FuzzySetType fuzzyRuleKd[7][7]={
//     {PS,PS,ZO,ZO,ZO,PB,PB},
//     {NS,NS,NS,NS,ZO,NS,PM},
//     {NB,NB,NM,NS,ZO,PS,PM},
//     {NB,NM,NM,NS,ZO,PS,PM},
//     {NB,NM,NS,NS,ZO,PS,PS},
//     {NM,NS,NS,NS,ZO,PS,PS},
//     {PS,ZO,ZO,ZO,ZO,PB,PB},
// };
// void fuzzySetInit(FuzzyPID *pid, float inputSet[7], float outputKpSet[7], float outputKiSet[7], float outputKdSet[7]){
//         for(uint8 i=0;i<6;i++){
//         pid->inputInterval[i].begin=inputSet[i];
//         pid->inputInterval[i].end=inputSet[i+1];
//         pid->outputKpSet[i]=outputKpSet[i];
//         pid->outputKiSet[i]=outputKiSet[i];
//         pid->outputKdSet[i]=outputKdSet[i];
//     }
// }
// // void fuzzyProcess(FuzzyPID *pid, float ref, float set, pid_type_def *pidOut){
// //     pid->input.e = set - ref;
// //     pid->input.ec = pid->input.e - pid->input.eLast;
// //     pid->input.eLast = pid->input.e;
    
// // }
// void fuzzyProcess(FuzzyPID *pid, float ref, float set, pid_type_def *pidOut){
//     pid->input.e = ref - set;
//     pid->input.ec = pid->input.e - pid->input.eLast;
//     pid->input.eLast = pid->input.e;
//      //locate e and ec in which interval
//     for(uint8 i=0;i<7;i++){
//         if(pid->input.e>=pid->inputInterval[i].begin&&pid->input.e<=pid->inputInterval[i].end){
//             pid->input.eFuzzySet=i;
//             break;
//         }else if(pid->input.e<pid->inputInterval[0].begin){
//             pid->input.eFuzzySet=0;
//             break;
//         }else if(pid->input.e>pid->inputInterval[6].end){
//             pid->input.eFuzzySet=5;
//             break;
//         }
//     }
//     for(uint8 i=0;i<7;i++){
//         if(pid->input.ec>=pid->inputInterval[i].begin&&pid->input.ec<=pid->inputInterval[i].end){
//             pid->input.ecFuzzySet=i;
//             break;
//         }else if(pid->input.ec<pid->inputInterval[0].begin){
//             pid->input.ecFuzzySet=0;
//             break;
//         }else if(pid->input.ec>pid->inputInterval[5].end){
//             pid->input.ecFuzzySet=5;
//             break;
//         }
//     }
//     float a = 0;
//     float b = 0;
//     if(pid->input.e<pid->inputInterval[0].begin){
//         a = 1;
//     }else if(pid->input.e>pid->inputInterval[6].end){
//         a = 0;
//     }else{
//         a = (float)(pid->input.e - pid->inputInterval[pid->input.eFuzzySet].begin)/(pid->inputInterval[pid->input.eFuzzySet].end - pid->inputInterval[pid->input.eFuzzySet].begin);
//     }
//     if(pid->input.ec<pid->inputInterval[0].begin){
//         b = 1;
//     }else if(pid->input.ec>pid->inputInterval[6].end){
//         b = 0;
//     }else{
//         b = (float)(pid->input.ec - pid->inputInterval[pid->input.ecFuzzySet].begin)/(pid->inputInterval[pid->input.ecFuzzySet].end - pid->inputInterval[pid->input.ecFuzzySet].begin);
//     }
//     // float a = (float)(pid->input.e - pid->inputInterval[pid->input.eFuzzySet].begin)/(pid->inputInterval[pid->input.eFuzzySet].end - pid->inputInterval[pid->input.eFuzzySet].begin);
//     // float b = (float)(pid->input.ec - pid->inputInterval[pid->input.ecFuzzySet].begin)/(pid->inputInterval[pid->input.ecFuzzySet].end - pid->inputInterval[pid->input.ecFuzzySet].begin);
//     //calculate the membership of each fuzzy set
//     pid->memberShip[0] = a * b;
//     pid->memberShip[1] = a * (1 - b);
//     pid->memberShip[2] = (1 - a) * b;
//     pid->memberShip[3] = (1 - a) * (1 - b);
//     //calculate the output of each fuzzy set
//     pid->output.kp = 0;
//     pid->output.ki = 0;
//     pid->output.kd = 0;
//     int8 indexKp[4]={0,0,0,0};
//     int8 indexKi[4]={0,0,0,0};
//     int8 indexKd[4]={0,0,0,0};
//     indexKp[0] = fuzzyRuleKp[pid->input.eFuzzySet][pid->input.ecFuzzySet];
//     indexKp[1] = fuzzyRuleKp[pid->input.eFuzzySet][pid->input.ecFuzzySet + 1];
//     indexKp[2] = fuzzyRuleKp[pid->input.eFuzzySet + 1][pid->input.ecFuzzySet];
//     indexKp[3] = fuzzyRuleKp[pid->input.eFuzzySet + 1][pid->input.ecFuzzySet + 1];
//     indexKi[0] = fuzzyRuleKi[pid->input.eFuzzySet][pid->input.ecFuzzySet];
//     indexKi[1] = fuzzyRuleKi[pid->input.eFuzzySet][pid->input.ecFuzzySet + 1];
//     indexKi[2] = fuzzyRuleKi[pid->input.eFuzzySet + 1][pid->input.ecFuzzySet];
//     indexKi[3] = fuzzyRuleKi[pid->input.eFuzzySet + 1][pid->input.ecFuzzySet + 1];
//     indexKd[0] = fuzzyRuleKd[pid->input.eFuzzySet][pid->input.ecFuzzySet];
//     indexKd[1] = fuzzyRuleKd[pid->input.eFuzzySet][pid->input.ecFuzzySet + 1];
//     indexKd[2] = fuzzyRuleKd[pid->input.eFuzzySet + 1][pid->input.ecFuzzySet];
//     indexKd[3] = fuzzyRuleKd[pid->input.eFuzzySet + 1][pid->input.ecFuzzySet + 1];
//     for(uint8 i=0;i<4;i++){
//         pid->output.kp += pid->memberShip[i] * pid->outputKpSet[indexKp[i]];
//         pid->output.ki += pid->memberShip[i] * pid->outputKiSet[indexKi[i]];
//         pid->output.kd += pid->memberShip[i] * pid->outputKdSet[indexKd[i]];
//     }
//     pidOut->Kp = (float)pid->output.kp;
//     pidOut->Ki = (float)pid->output.ki;
//     pidOut->Kd = (float)pid->output.kd;
// }
// /// @brief init the simple fuzzy pid
// /// @param pid 
// /// @param inputSet 
// /// @param outputKpSet 
// /// @param outputKiSet 
// /// @param outputKdSet 
// void simpleFuzzySetInit(SimpleFuzzyPID *pid, float inputSet[7], float outputKpSet[7], float outputKiSet[7], float outputKdSet[7]){
//     for(uint8 i=0;i<6;i++){
//         pid->inputInterval[i].begin=inputSet[i];
//         pid->inputInterval[i].end=inputSet[i+1];
//     }
//     for(uint8 i=0;i<7;i++){
//         pid->outputKpSet[i]=outputKpSet[i];
//         pid->outputKiSet[i]=outputKiSet[i];
//         pid->outputKdSet[i]=outputKdSet[i];
//     }
// }
// /// @brief process the simple fuzzy pid
// /// @param pid the simple fuzzy pid
// /// @param ref reference
// /// @param set real value
// /// @param pidOut the output of the pid
// void simpleFuzzyProcess(SimpleFuzzyPID *pid, float ref, float set, pid_type_def *pidOut){
//     pid->input.e = ref - set;
//     //locate e and ec in which interval
//     for(uint8 i=0;i<7;i++){
//         if(pid->input.e>=pid->inputInterval[i].begin&&pid->input.e<=pid->inputInterval[i].end){
//             pid->input.eFuzzySet=i;
//             break;
//         }else if(pid->input.e<pid->inputInterval[0].begin){
//             pid->input.eFuzzySet=0;
//             break;
//         }else if(pid->input.e>pid->inputInterval[5].end){
//             pid->input.eFuzzySet=5;
//             break;
//         }
//     }
//     float a = 0;
//     if(pid->input.e<pid->inputInterval[0].begin){
//         a = 0;
//     }else if(pid->input.e>pid->inputInterval[5].end){
//         a = 1;
//     }else{
//         a = (float)(pid->input.e - pid->inputInterval[pid->input.eFuzzySet].begin)/(pid->inputInterval[pid->input.eFuzzySet].end - pid->inputInterval[pid->input.eFuzzySet].begin);
//     }
//     pid->memberShip = 1 - a;
//     //calculate the output of each fuzzy set
//     pid->output.kp = pid->memberShip * pid->outputKpSet[pid->input.eFuzzySet] + (1 - pid->memberShip) * pid->outputKpSet[pid->input.eFuzzySet + 1];
//     pid->output.ki = pid->memberShip * pid->outputKiSet[pid->input.eFuzzySet] + (1 - pid->memberShip) * pid->outputKiSet[pid->input.eFuzzySet + 1];
//     pid->output.kd = pid->memberShip * pid->outputKdSet[pid->input.eFuzzySet] + (1 - pid->memberShip) * pid->outputKdSet[pid->input.eFuzzySet + 1];

//     pidOut->Kp = (float)pid->output.kp;
//     pidOut->Ki = (float)pid->output.ki;
//     pidOut->Kd = (float)pid->output.kd;
//     // {
//     //     tft180_show_int(10,60,pid->input.eFuzzySet,2);
//     //     tft180_show_float(50,60,pid->memberShip,2,2);
//     //     tft180_show_float(50,80,pidOut->Kp,4,4);
//     // }
// }