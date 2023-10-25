#if !defined(CONFIG_H)
#define CONFIG_H

#include "Arduino.h"
#include "ep_adrs.h"

/// @brief 0 is CTF, 1 is VOA-Heater, 2 is VOA-No Heater, 3 is 3D Switch/WOA
byte Station_Type = 0;

const byte X_STP_Pin = 15; // x軸 步進控制Pulse
const byte X_DIR_Pin = 2;  // X軸 步進馬達方向控制
const byte Y_STP_Pin = 5;  // y軸 步進控制Pulse (Pin 0 is for boot) 0 => 5
const byte Y_DIR_Pin = 18; // y軸 步進馬達方向控制 4 => 18
const byte Z_STP_Pin = 16; // z軸 步進控制Pulse
const byte Z_DIR_Pin = 17; // z軸 步進馬達方向控制

const byte AWO_Pin = 4; // 步進馬達電流開關控制 On為中斷，Off開啟

String ID = "003";
String Station_ID = "A00";

int ButtonSelected = 0;

const byte Heater_Start_Pin = 12;
const byte Heater_Stop_Pin = 14;

// pin = 13
int Tablet_PD_mode_Trigger_Pin = 13;

Adafruit_ADS1115 ads;
TwoWire I2CADS = TwoWire(1);

#define I2C_SDA 21 // 21
#define I2C_SCL 22 // 22

typedef struct struct_Motor_Pos
{
  long X = 0;
  long Y = 0;
  long Z = 0;
} struct_Motor_Pos;

struct_Motor_Pos Pos_Now;

typedef struct struct_send_server_message
{
  String client_name = Station_ID; // ID to Station_ID
  char msg[50];
} struct_send_server_message;

struct_send_server_message sendmsg_server;

//------------------------------------------------------------------------------------------------------------------------------------------
// 90:38:0C:ED:82:28 == A06 Controller
// 0xC4, 0xDE, 0xE2, 0xC0, 0xA2, 0xD8   A05 Controller

// 8C:4B:14:16:4C:F8 == A06 Core
// 0x90, 0x38, 0x0C, 0xED, 0x82, 0x28    A06 controller

// 0x8C, 0x4B, 0x14, 0x16, 0xD8, 0xD0    A07 controller

// 90:38:0C:ED:6D:EC A09 controller

// 0xC0, 0x49, 0xEF, 0x46, 0x2C, 0x74  A08 Controller

// 0x90, 0x38, 0x0C, 0xED, 0x6D, 0xEC  A09 Controller

// 0xC4, 0xDE, 0xE2, 0xC0, 0xA9, 0x60  A10 controller

uint8_t ControllerAddress[] = {0x90, 0x38, 0x0C, 0xED, 0x6D, 0xEC}; // Controller addr
// uint8_t ServerAddress[] = {0x8C, 0x4B, 0x14, 0x16, 0x35, 0x88};     // Server Mac address  #1:0x8C, 0x4B, 0x14, 0x16, 0x37, 0xF8   #2:0x8C, 0x4B, 0x14, 0x16, 0x35, 0x88
uint8_t ServerAddress[] = {0x8C, 0x4B, 0x14, 0x16, 0x37, 0xF8}; // Server Mac address  #1:0x8C, 0x4B, 0x14, 0x16, 0x37, 0xF8   #2:0x8C, 0x4B, 0x14, 0x16, 0x35, 0x88
uint8_t ThisAddress[6];                                         // Server Mac address  #1:0x8C, 0x4B, 0x14, 0x16, 0x37, 0xF8   #2:0x8C, 0x4B, 0x14, 0x16, 0x35, 0x88
String ThisAddr = "";

String Msg, Msg_Value;

/// @brief 1: button,  2: cmd, 3:controller btn
byte Motor_Continous_Mode = 2;

bool isWiFiConnected = false;
bool isCheckingServer = false;

int MotorDir_Pin = 0;
int MotorSTP_Pin = 0;

extern bool MotorCC_A;
extern bool MotorCC_X;
extern bool MotorCC_Y;
extern bool MotorCC_Z;

double MotorStepRatio = 1;
double MotorStepDelayRatio = 1;
int delayBetweenStep = 600;
int delayBetweenStep_X = 8;
int delayBetweenStep_Y = 8;
int delayBetweenStep_Z = 8;
int MinMotorDelayTime = 320;
long MinMotroStep = 20;
int M_Level = 10;

int xyz = 0;
long X_Pos_Record = 0;
long Y_Pos_Record = 0;
long Z_Pos_Record = 0;
long X_Pos_Now = 0;
long Y_Pos_Now = 0;
long Z_Pos_Now = 0;
long Z_Pos_reLoad = 0;

int X_backlash = 0;
int Y_backlash = 0;
int Z_backlash = 0;

int X_ScanSTP = 12;
int Y_ScanSTP = 10;
int Z_ScanSTP = 200;

int X_ScanStable = 25;
int Y_ScanStable = 50;
int Z_ScanStable = 80;

int AA_SpiralRough_Feed_Steps_Z_A = 25000;
int AA_SpiralRough_Spiral_Steps_XY_A = 2000;
int AA_SpiralFine_Spiral_Steps_XY_A = 1500;
int AA_SpiralFine_Scan_Steps_X_A = 25;
int AA_SpiralFine_Scan_Steps_X_B = 30;
int AA_SpiralFine_Scan_Steps_X_C = 40;
int AA_SpiralFine_Scan_Steps_X_D = 50;
int AA_SpiralFine_Scan_Steps_Y_A = 30;
int AA_SpiralFine_Scan_Steps_Y_B = 40;
int AA_SpiralFine_Scan_Steps_Y_C = 60;
int AA_SpiralFine_Scan_Steps_Y_D = 80;
int AA_SpiralFine_Scan_Steps_Y_E = 140;
int AA_ScanRough_Feed_Steps_Z_A = 10000;
int AA_ScanRough_Feed_Steps_Z_B = 500;
double AA_ScanRough_Feed_Ratio_Z_A = 5;   // 2.8
double AA_ScanRough_Feed_Ratio_Z_B = 4;   // 2.5
double AA_ScanRough_Feed_Ratio_Z_C = 3;   // 2.0
double AA_ScanRough_Feed_Ratio_Z_D = 1.5; // 1.5
int AA_ScanRough_Scan_Steps_Y_A = 25;
int AA_ScanRough_Scan_Steps_Y_B = 30;
int AA_ScanRough_Scan_Steps_Y_C = 40;
int AA_ScanRough_Scan_Steps_Y_D = 70;
int AA_ScanRough_Scan_Steps_X_A = 25;
int AA_ScanRough_Scan_Steps_X_B = 30;
int AA_ScanRough_Scan_Steps_X_C = 80;
int AA_ScanRough_Scan_Steps_X_D = 100;
int AA_ScanRough_Scan_Steps_X_E = 120;
int AA_ScanFine_Scan_Steps_Z_A = 200;
int AA_ScanFine_Scan_Steps_Y_A = 20;
int AA_ScanFine_Scan_Steps_X_A = 20;
int AA_ScanFinal_Scan_Steps_Z_A = 125;
int AA_ScanFinal_Scan_Steps_Y_A = 20;
int AA_ScanFinal_Scan_Steps_X_A = 20;

int AQ_Scan_Compensation_Steps_Z_A = 12;
int AQ_Scan_Steps_Z_A = 130;
int AQ_Scan_Steps_Z_B = 130;
int AQ_Scan_Steps_Z_C = 130;
int AQ_Scan_Steps_Z_D = 100;

int AA_ScanFinal_Scan_Delay_X_A = 100;
int AA_ScanFinal_Scan_Delay_Y_A = 60;

int AQ_Total_TimeSpan = 2000;
int AQ_StopAlign_TimeSpan = 860;

double FS_GradientTarget_X = 0.003;
double FS_GradientTarget_Y = 0.002;
double FS_GradientTarget_Z = 0.003;

double averagePDInput = 0;

double ref_Dac = 0; // PD reference
double ref_IL = 0;  // PD reference

double PDValue_Best = 0;
double AutoCuring_Best_IL = 0, PD_Now = 0, PD_Before = 0;

unsigned long time_curing_0, time_curing_1, time_curing_2, time_curing_3;
unsigned long timer_Get_IL_1 = 0, timer_Get_IL_2;

bool btn_isTrigger = false;
int Threshold;
int stableDelay = 0;
bool key_ctrl = false;

/// @brief (1/51.2) um/pulse
double Motor_Unit_Idx = 0.01953125; // 5 phase motor min step 4nm per step

int Get_PD_Points = 1;
double Target_IL = 0;
double Target_IL_Rough = -3; // Stop IL value for Rought scan: default:-3 (CTF)
double StopValue = 0;
int cmd_No = 0;
bool isStop = false, isGetPower = false, isILStable = false;
bool isCheckStop = false;
bool sprial_JumpToBest = true;
int Q_State = 0;
unsigned long Q_Time = 0;
unsigned long LCD_Auto_Update_TimeCount = 0;

// 1:IL, 2:Dac(- ref), 3:Row IL(dBm), 4:Row Dac
byte GetPower_Mode = 1;
bool is_Scan_V2_ReWork = false;
bool is_AutoCuring = false;
bool isTrip3Jump = true; // If station is high resolution , this value could be true;
bool isWatchDog_Flag = false;
bool isLCD_Auto_Update = false;

bool X_DIR_True = true;
bool X_DIR_False = false;
bool Y_DIR_True = true;
bool Y_DIR_False = false;
bool Z_DIR_True = true;
bool Z_DIR_False = false;

uint16_t FS_Count_X = 7;
uint16_t FS_Steps_X = 25;
uint16_t FS_Stable_X = 0;
uint16_t FS_DelaySteps_X = 50;
uint16_t FS_Avg_X = 600;
uint16_t FS_Count_Y = 8;
uint16_t FS_Steps_Y = 20;
uint16_t FS_Stable_Y = 0;
uint16_t FS_DelaySteps_Y = 120;
uint16_t FS_Avg_Y = 600;
uint16_t FS_Count_Z = 7;
uint16_t FS_Steps_Z = 80;
uint16_t FS_Stable_Z = 0;
uint16_t FS_DelaySteps_Z = 80;
uint16_t FS_Avg_Z = 800;
uint16_t FS_Trips_X = 0;
uint16_t FS_Trips_Y = 0;
uint16_t FS_Trips_Z = 2;

void DataSent_Server(String MSG);
void CMDOutput(String cmd);
void CMDOutput(String cmd, bool isSentServer);
void MSGOutput(String msg);
void DataOutput();
void DataOutput(double IL);
void DataOutput(bool isIL);
void DataOutput(int xyz, double pdValue);

double ILConverter(double pdDac);
double Cal_PD_Input_IL(int averageCount);
double Cal_PD_Input_Dac(int averageCount);
double Cal_PD_Input_Row_IL(int averageCount);
double Cal_PD_Input_Row_Dac(int averageCount);
double Get_IL_DAC(int averageCount);
void EmergencyStop();

void step(byte stepperPin, long steps, int delayTime);
void step(byte stepperPin, long steps, int delayTime, byte dirPin, bool dir);
void Move_Motor(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay, bool isOutputPosition = false, int pinDelay = 8);
void Move_Motor_abs(int xyz, long Target);
void Move_Motor_abs_async(struct_Motor_Pos TargetPos, int DelayT = 40);
void Move_Motor_abs_sync(struct_Motor_Pos TargetPos, int DelayT = 10);
void Move_Motor_abs_all(long x, long y, long z, int DelayT = 6);
void Move_Motor_abs_all(int x, int y, int z, bool IsMsg, int DelayT = 10);
void Move_Motor_Cont(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep);

#endif