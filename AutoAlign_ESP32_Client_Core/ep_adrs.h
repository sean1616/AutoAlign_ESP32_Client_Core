#if !defined(EP_ADRS_H)
#define EP_ADRS_H

#include "Arduino.h"

const uint16_t EP_PD_Ref = 0;
const uint16_t EP_Board_ID = 8;
const uint16_t EP_Station_ID = 16;
const uint16_t EP_X_backlash = 24;
const uint16_t EP_Y_backlash = 32;
const uint16_t EP_Z_backlash = 40;
const uint16_t EP_delayBetweenStep_X = 48;
const uint16_t EP_delayBetweenStep_Y = 56;
const uint16_t EP_delayBetweenStep_Z = 64;
const uint16_t EP_Target_IL = 72;
const uint16_t EP_FW_Version = 80;
// const uint16_t EP_AA_ScanFinal_Scan_Delay_X_A = 80;
// const uint16_t EP_Server_ID = 88;        // 88~119
// const uint16_t EP_Server_Password = 120; // 120~151

// for 3D
const uint16_t EP_Motor_DIR_X = 88;
const uint16_t EP_Motor_DIR_Y = 96;
const uint16_t EP_Motor_DIR_Z = 104;
const uint16_t EP_Encoder_Motor_Step_X = 112;
const uint16_t EP_Encoder_Motor_Step_Y = 120;
const uint16_t EP_Encoder_Motor_Step_Z = 128;

const uint16_t EP_Get_PD_Points = 152;
const uint16_t EP_AQ_Scan_Compensation_Steps_Z_A = 160;
const uint16_t EP_AQ_Total_TimeSpan = 168;
const uint16_t EP_AQ_Scan_Steps_Z_A = 176;
const uint16_t EP_AQ_Scan_Steps_Z_B = 184;
const uint16_t EP_AQ_Scan_Steps_Z_C = 192;
const uint16_t EP_AQ_Scan_Steps_Z_D = 200;
const uint16_t EP_MotorStepRatio = 208;
const uint16_t EP_MotorStepDelayRatio = 216;
const uint16_t EP_FS_Count_X = 240;
const uint16_t EP_FS_Steps_X = 248;
const uint16_t EP_FS_Stable_X = 256;
const uint16_t EP_FS_DelaySteps_X = 264;
const uint16_t EP_FS_Avg_X = 272;
const uint16_t EP_FS_Count_Y = 280;
const uint16_t EP_FS_Steps_Y = 288;
const uint16_t EP_FS_Stable_Y = 296;
const uint16_t EP_FS_DelaySteps_Y = 304;
const uint16_t EP_FS_Avg_Y = 312;
const uint16_t EP_FS_Count_Z = 320;
const uint16_t EP_FS_Steps_Z = 328;
const uint16_t EP_FS_Stable_Z = 336;
const uint16_t EP_FS_DelaySteps_Z = 344;
const uint16_t EP_FS_Avg_Z = 352;
const uint16_t EP_FS_Trips_X = 360;
const uint16_t EP_FS_Trips_Y = 368;
const uint16_t EP_FS_Trips_Z = 376;
const uint16_t EP_FS_GradientTarget_X = 384;
const uint16_t EP_FS_GradientTarget_Y = 392;
const uint16_t EP_FS_GradientTarget_Z = 400;

const uint16_t EP_Station_Type = 408;

const uint16_t EP_AA_ScanRough_Feed_Steps_Z_A = 416;
const uint16_t EP_AA_ScanRough_Feed_Steps_Z_B = 424;
const uint16_t EP_AA_ScanRough_Feed_Ratio_Z_A = 432;
const uint16_t EP_AA_ScanRough_Feed_Ratio_Z_B = 440;
const uint16_t EP_AA_ScanRough_Feed_Ratio_Z_C = 448;
const uint16_t EP_AA_ScanRough_Feed_Ratio_Z_D = 456;

/// @brief EEPROM address of PD Ref array (700~928)
uint16_t EP_PD_Ref_Array[15][2] =
    {
        {700, 708},
        {716, 724},
        {732, 740},
        {748, 756}, // max -6.2 dBm
        {764, 772},
        {780, 788},
        {796, 804},
        {812, 820},
        {828, 836},
        {844, 852},
        {860, 868},
        {876, 884},
        {892, 900},
        {908, 916},
        {924, 932},
};

/// @brief Default Ref array value
double PD_Ref_Array[15][2] =
    {
        {24260, -3},
        {23644, -4},
        {23282, -6},
        {22054, -7}, // max -6.2 dBm
        {21385, -9},
        {20680, -11},
        {19956, -13},
        {19305, -15},
        {18655, -17},
        {17937, -19},
        {17285, -21},
        {15864, -25},
        {14177, -30},
        {10780, -40},
        {8240, -50},
};

#endif