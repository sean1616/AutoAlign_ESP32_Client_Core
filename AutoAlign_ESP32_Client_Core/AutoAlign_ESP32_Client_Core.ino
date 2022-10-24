#include <EEPROM.h>
#include <curveFitting.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

TaskHandle_t Task_1;
#define WDT_TIMEOUT 3

// String server_ID, server_Password;

bool isWiFiConnected = false;
bool isCheckingServer = false;

String ID = "003";
String Station_ID = "A00";

const byte X_STP_Pin = 15; //x軸 步進控制Pulse
const byte X_DIR_Pin = 2;  //X軸 步進馬達方向控制
const byte Y_STP_Pin = 5;  //y軸 步進控制Pulse 0(Pin 0 is for boot)
const byte Y_DIR_Pin = 18;  //y軸 步進馬達方向控制 4
const byte Z_STP_Pin = 16; //z軸 步進控制Pulse
const byte Z_DIR_Pin = 17; //z軸 步進馬達方向控制

int ButtonSelected = 0;

// int LCD_Encoder_A_pin = 35; //22
// int LCD_Encoder_B_pin = 23; //23
// uint8_t LCD_Select_pin = 34;  //21

// bool LCD_Encoder_State = false;
// bool LCD_Encoder_LastState = false;
// int LCD_en_count = 0, idx = 0;
// int LCD_sub_count = 0, idx_sub = 0;
// int current_selection = 0;

// int LCD_Update_Mode = 0;
// uint8_t LCD_PageNow = 1;

const byte R_0 = 12;

/* Keyboard Pin Setting */
const byte R_1 = 14;
const byte R_2 = 27;
const byte R_3 = 26;
const byte C_1 = 25;
const byte C_2 = 33;
const byte C_3 = 32;

// const byte PD_Pin = 34;
int Tablet_PD_mode_Trigger_Pin = 13;

Adafruit_ADS1115 ads;
TwoWire I2CADS = TwoWire(1);

#define I2C_SDA 21  //21
#define I2C_SCL 22  //22

const byte LED_Align = 5;

int MotorDir_Pin = 0;
int MotorSTP_Pin = 0;
bool MotorCC = false;
bool MotorCC_X = false;
bool MotorCC_Y = false;
bool MotorCC_Z = false;

double MotorStepRatio = 1;
double MotorStepDelayRatio = 1;
int delayBetweenStep = 600;
int delayBetweenStep_X = 8;
int delayBetweenStep_Y = 8;
int delayBetweenStep_Z = 8;
int MinMotorDelayTime = 320;
long MinMotroStep = 20;
int M_Level = 10;

byte Motor_Continous_Mode = 1;  //1: button,  2: cmd, 3:controller btn

int xyz = 0;

long X_Pos_Record = 0;
long Y_Pos_Record = 0;
long Z_Pos_Record = 0;
long X_Pos_Now = 0;
long Y_Pos_Now = 0;
long Z_Pos_Now = 0;
long Z_Pos_reLoad = 0;

// int X_rotator_steps = 2;
// int Y_rotator_steps = 2;
// int Z_rotator_steps = 20;

int X_backlash = 0;
int Y_backlash = 0;
int Z_backlash = 0;

int X_ScanSTP = 12;
int Y_ScanSTP = 10;
int Z_ScanSTP = 200;

int X_ScanStable = 25;
int Y_ScanStable = 50;
int Z_ScanStable = 80;

//Intention _ Region _ MotionType _ ParaType _ Axis _ Rank = Value
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
int AA_ScanRough_Feed_Steps_Z_B = 1000;
double AA_ScanRough_Feed_Ratio_Z_A = 5;  //2.8
double AA_ScanRough_Feed_Ratio_Z_B = 4;  //2.5
double AA_ScanRough_Feed_Ratio_Z_C = 3.5;  //2.0
double AA_ScanRough_Feed_Ratio_Z_D = 2.5;  //1.5
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
int AQ_Scan_Steps_Z_A = 40;  //125, 30
int AQ_Scan_Steps_Z_B = 40;  //120, 30
int AQ_Scan_Steps_Z_C = 45;   //70, 35
int AQ_Scan_Steps_Z_D = 50;   //50, 50

int AA_ScanFinal_Scan_Delay_X_A = 100;
int AA_ScanFinal_Scan_Delay_Y_A = 60;

int AQ_Total_TimeSpan = 840;

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

uint16_t EP_PD_Ref = 0;
uint16_t EP_Board_ID = 8;
uint16_t EP_Station_ID = 16;
uint16_t EP_X_backlash = 24;
uint16_t EP_Y_backlash = 32;
uint16_t EP_Z_backlash = 40;
uint16_t EP_delayBetweenStep_X = 48;
uint16_t EP_delayBetweenStep_Y = 56;
uint16_t EP_delayBetweenStep_Z = 64;
uint16_t EP_Target_IL = 72;
uint16_t EP_AA_ScanFinal_Scan_Delay_X_A = 80;
uint16_t EP_Server_ID = 88;  //88~119
uint16_t EP_Server_Password = 120;  //120~151
uint16_t EP_AQ_Scan_Compensation_Steps_Z_A = 160;
uint16_t EP_AQ_Total_TimeSpan = 168;
uint16_t EP_AQ_Scan_Steps_Z_A = 176;
uint16_t EP_AQ_Scan_Steps_Z_B = 184;
uint16_t EP_AQ_Scan_Steps_Z_C = 192;
uint16_t EP_AQ_Scan_Steps_Z_D = 200;
uint16_t EP_FS_Count_X = 240;
uint16_t EP_FS_Steps_X = 248;
uint16_t EP_FS_Stable_X = 256;
uint16_t EP_FS_DelaySteps_X = 264;
uint16_t EP_FS_Avg_X = 272;
uint16_t EP_FS_Count_Y = 280;
uint16_t EP_FS_Steps_Y = 288;
uint16_t EP_FS_Stable_Y = 296;
uint16_t EP_FS_DelaySteps_Y = 304;
uint16_t EP_FS_Avg_Y = 312;
uint16_t EP_FS_Count_Z = 320;
uint16_t EP_FS_Steps_Z = 328;
uint16_t EP_FS_Stable_Z = 336;
uint16_t EP_FS_DelaySteps_Z = 344;
uint16_t EP_FS_Avg_Z = 352;
uint16_t EP_FS_Trips_X = 360;
uint16_t EP_FS_Trips_Y = 368;
uint16_t EP_FS_Trips_Z = 376;

uint16_t EP_AA_ScanRough_Feed_Steps_Z_A = 416;
uint16_t EP_AA_ScanRough_Feed_Steps_Z_B = 424;
uint16_t EP_AA_ScanRough_Feed_Ratio_Z_A = 432;
uint16_t EP_AA_ScanRough_Feed_Ratio_Z_B = 440;
uint16_t EP_AA_ScanRough_Feed_Ratio_Z_C = 448;
uint16_t EP_AA_ScanRough_Feed_Ratio_Z_D = 456;

//700~
uint16_t EP_PD_Ref_Array[15][2];

//Default Ref array value 
  int PD_Ref_Array[15][2] = 
  {
    {24260, -3},
    {23644, -4},
    {23282, -6},  
    {22054, -7}, //max -6.2 dBm
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

double averagePDInput = 0;

double ref_Dac = 0; //PD reference
double ref_IL = 0;  //PD reference

double PDValue_Best = 0;
double AutoCuring_Best_IL = 0, PD_Now = 0, PD_Before = 0;

unsigned long time_curing_0, time_curing_1, time_curing_2, time_curing_3;
unsigned long timer_Get_IL_1 = 0, timer_Get_IL_2;

bool btn_isTrigger = false;
int Threshold;
int stableDelay = 0;
bool key_ctrl = false;

double Motor_Unit_Idx = 0.01953125; /* (1/51.2) um/pulse */

int Get_PD_Points = 1;
double Target_IL = 0; //0 dB
double StopValue = 0; //0 dB
int cmd_No = 0;

bool isStop = false, isGetPower = true, isILStable = false;
bool sprial_JumpToBest = true;
int Q_State = 0;
unsigned long Q_Time = 0;
unsigned long LCD_Auto_Update_TimeCount = 0;
byte GetPower_Mode = 1;
bool is_Scan_V2_ReWork = false;
bool is_AutoCuring = false;
bool isTrip3Jump = true;  //If station is high resolution , this value could be true;

bool isWatchDog_Flag = false;
bool isLCD = true;
bool isLCD_Auto_Update = false;

void step(byte stepperPin, long steps, int delayTime)
{
  steps = abs(steps);

  for (long i = 0; i < steps; i++)
  {
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(delayTime);
  }

  //Position Record
  if (MotorCC)
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now += steps;
      MotorCC_X = true;
      break;
    case Y_STP_Pin:
      Y_Pos_Now += steps;
      MotorCC_Y = true;
      break;
    case Z_STP_Pin:
      Z_Pos_Now += steps;
      MotorCC_Z = true;
      break;
    }
  }
  else
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now -= steps;
      MotorCC_X = false;
      break;
    case Y_STP_Pin:
      Y_Pos_Now -= steps;
      MotorCC_Y = false;
      break;
    case Z_STP_Pin:
      Z_Pos_Now -= steps;
      MotorCC_Z = false;
      break;
    }
  }
}

void step(byte stepperPin, long steps, int delayTime, byte dirPin, bool dir)
{
  // steps = abs(steps);
  digitalWrite(dirPin, dir);

  for (long i = 0; i < steps; i++)
  {
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(delayTime);
  }

  //Position Record
  if (MotorCC == true)
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now += steps;
      MotorCC_X = true;
      break;
    case Y_STP_Pin:
      Y_Pos_Now += steps;
      MotorCC_Y = true;
      break;
    case Z_STP_Pin:
      Z_Pos_Now += steps;
      MotorCC_Z = true;
      break;
    }
  }
  else
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now -= steps;
      MotorCC_X = false;
      break;
    case Y_STP_Pin:
      Y_Pos_Now -= steps;
      MotorCC_Y = false;
      break;
    case Z_STP_Pin:
      Z_Pos_Now -= steps;
      MotorCC_Z = false;
      break;
    }
  }
}

int KeyValueConverter()
{
  int keyNo = -1;
  bool isKeyPressed = false;
  int keyValueSum = 0;

  if (!digitalRead(R_1))
  {
    isKeyPressed = true;
    keyValueSum += 10;
  }
  else if (!digitalRead(R_2))
  {
    isKeyPressed = true;
    keyValueSum += 5;
  }
  else if (!digitalRead(R_3))
  {
    isKeyPressed = true;
    keyValueSum += 0;
  }

  if (isKeyPressed)
  {
    pinMode(C_1, INPUT_PULLUP);
    pinMode(C_2, INPUT_PULLUP);
    pinMode(C_3, INPUT_PULLUP);

    pinMode(R_1, OUTPUT);
    pinMode(R_2, OUTPUT);
    pinMode(R_3, OUTPUT);

    delay(2);

    if (!digitalRead(C_1))
    {
      keyValueSum += 1;
    }
    else if (!digitalRead(C_2))
      keyValueSum += 2;
    else if (!digitalRead(C_3))
      keyValueSum += 3;
    else
      keyValueSum = 0;

    pinMode(R_1, INPUT_PULLUP);
    pinMode(R_2, INPUT_PULLUP);
    pinMode(R_3, INPUT_PULLUP);

    pinMode(C_1, OUTPUT);
    pinMode(C_2, OUTPUT);
    pinMode(C_3, OUTPUT);

    delay(2);
  }

  if (keyValueSum != 0)
  {

    switch (keyValueSum)
    {
    case 1:
      keyNo = 101; /* Z- */
      break;
    case 2:
      keyNo = 102; /* X+ */
      break;
    case 3:
      keyNo = 103; /* Z+ */
      break;
    case 6:
      keyNo = 104; /* Y+ */
      break;
    case 7:
      keyNo = 105; /* X- */
      break;
    case 8:
      keyNo = 106; /* Y- */
      break;
    case 11:
      keyNo = 7;
      break;
    case 12:
      keyNo = 8;
      break;
    case 13:
      keyNo = 9;
      break;
    default:
      keyNo = -1;
      break;
    }

    isKeyPressed = false;
  }

  return keyNo;
}

double a1 = 0.0374, a2 = -65.561;
double b1 = 0.0394, b2 = -67.778;

//Dac to dBm
double ILConverter(double pdDac)
{
  double IL = 0;

  if(pdDac >= PD_Ref_Array[0][0])
    return -3;
  else if (pdDac < PD_Ref_Array[14][0])
    return -50;

  for (size_t i = 1; i < 15; i++)
  {        
    if(pdDac >= PD_Ref_Array[i][0])
    {
      IL = ((pdDac - PD_Ref_Array[i][0])/(PD_Ref_Array[i-1][0] - PD_Ref_Array[i][0]) * (PD_Ref_Array[i - 1][1] - PD_Ref_Array[i][1])) + PD_Ref_Array[i][1];
      break;
    }
  }
  
  return IL;
}

//Calculate PD input value, Return Dac
double Cal_PD_Input_Dac(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;
  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += ads.readADC_SingleEnded(0);
  }

  //Function: (PD Value) - (reference) + 300
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  return (averagePDInput - ref_IL);
}

//Calculate PD input value, Return IL
double Cal_PD_Input_IL(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;

  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += ads.readADC_SingleEnded(0);
  }

  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  double IL = ILConverter(averagePDInput) - ref_IL;

  return IL;
}

//Calculate PD input value, Return Row Dac
double Cal_PD_Input_Row_IL(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;
  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += ads.readADC_SingleEnded(0);
  }
  //Function: (PD Value)
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  double IL = ILConverter(averagePDInput);

  return IL;
}

//Calculate PD input value, Return Row Dac
double Cal_PD_Input_Row_Dac(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;
  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += ads.readADC_SingleEnded(0);
  }
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  return averagePDInput;
}

void CleanEEPROM(int startPosition, int datalength)
{
  for (size_t i = startPosition; i < (startPosition + datalength); i++)
  {
    EEPROM.write(i, ' ');
  }
  Serial.println("Clean EEPROM");
}

void WriteInfoEEPROM(String data, int start_position)
{
  for (int i = 0; i < data.length(); ++i)
  {
    EEPROM.write(i + start_position, data[i]);
  }
}

String ReadInfoEEPROM(int start_position, int data_length)
{
  String EEPROM_String = "";
  for (int i = 0; i < data_length; i++)
  {
    uint8_t a = EEPROM.read(i + start_position);
    if (a != 255)
      EEPROM_String += char(EEPROM.read(i + start_position));
  }
  EEPROM_String.trim();
  return EEPROM_String;
}

String WR_EEPROM(int start_position, String data)
{
  CleanEEPROM(start_position, 8); //Clean EEPROM(int startPosition, int datalength)

  WriteInfoEEPROM(String(data), start_position); //Write Data to EEPROM (data, start_position)
  EEPROM.commit();

  String s = ReadInfoEEPROM(start_position, 8);
  return s;
}

String WR_EEPROM(int start_position, int data_length, String data)
{
  CleanEEPROM(start_position, data_length); //Clean EEPROM(int startPosition, int datalength)

  WriteInfoEEPROM(String(data), start_position); //Write Data to EEPROM (data, start_position)
  EEPROM.commit();

  String s = ReadInfoEEPROM(start_position, data_length);
  return s;
}

bool Contains(String text, String search)
{
  if (text.indexOf(search) == -1)
    return false;
  else
    return true;
}

typedef enum {
    HomeMade ,       /**< X direction */
    Phase_5,       /**< Y direction */
} Motor_Type;

Motor_Type motorType;

typedef enum {
    X_Dir = 0,       /**< X direction */
    Y_Dir = 1,       /**< Y direction */
    Z_Dir = 2,       /**< Z direction */
    All_Dir = 3,     /**< All direction */  
} axis_direction;

axis_direction axisDir;

#define ITEMS_COUNT 100
char *UI_Items[ITEMS_COUNT] =
    {" "};

#define MENU_ITEMS 6
char *UI_Menu_Items[MENU_ITEMS] =
    {"1. Status",
     "2. Target IL",
     "3. StableDelay",
     "4. Q Z-offset",
     "5. Speed",
     "6. Get Ref"};

#define Speed_Page_ITEMS 4
char *UI_Speed_Page_Items[MENU_ITEMS] =
    {"1. X Speed",
     "2. Y Speed",
     "3. Z Speed",
     "<<"};

uint8_t i, h, w, title_h, H;

int PageLevel = 0;
int PageItemsCount = 1;

int Top_Item_Index = 0;
int Bottom_Item_Index = 3;
bool ui_YesNo_Selection = false;

int mainpageIndex = 0;

int subpageIndex = 0;
int subpage_itemsCount = 1;
bool item_is_selected = false;
bool plus_minus = false;

void EmergencyStop()
{
  isStop = true;

  Serial.println("EmergencyStop");
  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

  isLCD = true;
  PageLevel = 0;
}
//------------------------------------------------------------------------------------------------------------------------------------------

// uint8_t broadcastAddress[] = {0x7C, 0xDF, 0xA1, 0xF3, 0x73, 0x58}; //ESPS3 Controller addr    7C:DF:A1:F3:73:58
uint8_t ControllerAddress[] = {0x90, 0x38, 0x0C, 0xED, 0x82, 0x28}; //Controller addr   
uint8_t ServerAddress[] = {0x8C, 0x4B, 0x14, 0x16, 0x35, 0x88};  //Server Mac address  #1:0x8C, 0x4B, 0x14, 0x16, 0x37, 0xF8   #2:0x8C, 0x4B, 0x14, 0x16, 0x35, 0x88
uint8_t ThisAddress[6];  //Server Mac address  #1:0x8C, 0x4B, 0x14, 0x16, 0x37, 0xF8   #2:0x8C, 0x4B, 0x14, 0x16, 0x35, 0x88
String ThisAddr = "";

String Msg, Msg_Value;

typedef struct struct_send_server_message {
    String client_name = Station_ID; //ID to Station_ID
    char msg[50];
} struct_send_server_message;

struct_send_server_message sendmsg_server;

typedef struct struct_send_message {
    String msg;
} struct_send_message;

//接收資料格式
typedef struct struct_receive_message {
    String contr_name;
    char cmd[30];
    char value[20];
    // String value;
} struct_receive_message;

//送出資料格式
typedef struct struct_sendmsg_msg_UI_Data {
    String msg;
    double _Target_IL;
    int _Q_Z_offset;
    double _ref_Dac;
    int _speed_x;
    int _speed_y;
    int _speed_z;
    int _QT;
    char para[30];
} struct_sendmsg_msg_UI_Data;

// Create a struct_message to hold incoming sensor readings
struct_send_message sendmsg;
struct_receive_message incomingReadings;

struct_sendmsg_msg_UI_Data sendmsg_UI_Data;

void DataSent_Server(String MSG)
{
  MSG.toCharArray(sendmsg_server.msg, 30);
  esp_err_t result = esp_now_send(ServerAddress, (uint8_t *) &sendmsg_server, sizeof(sendmsg_server));
}

void DataSent_Controller(String MSG)
{
  sendmsg_UI_Data.msg = "Core:" + MSG;
  sendmsg_UI_Data._Target_IL = Target_IL;
  sendmsg_UI_Data._ref_Dac = ref_Dac;
  sendmsg_UI_Data._Q_Z_offset = AQ_Scan_Compensation_Steps_Z_A;
  sendmsg_UI_Data._speed_x = delayBetweenStep_X;
  sendmsg_UI_Data._speed_y = delayBetweenStep_Y;
  sendmsg_UI_Data._speed_z = delayBetweenStep_Z;
  sendmsg_UI_Data._QT = Q_Time;
  esp_err_t result = esp_now_send(ControllerAddress, (uint8_t *) &sendmsg_UI_Data, sizeof(sendmsg_UI_Data));
}

String name_from_contr = "", cmd_from_contr = "", cmd_value_from_contr = "";

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  String ss = "";
  ss.toCharArray(incomingReadings.cmd, 30);
  ss.toCharArray(incomingReadings.value, 20);
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  Msg = incomingReadings.cmd;
  Msg_Value = incomingReadings.value;

  if( Msg != "")
  {
    name_from_contr = incomingReadings.contr_name;
    cmd_from_contr = Msg;

    if( Msg_Value != "")
    {
      cmd_value_from_contr = Msg_Value;

      if(Msg == "BS" && Msg_Value == "0")
        EmergencyStop();
    }
    else
      Serial.println(incomingReadings.contr_name + Msg);
  } 
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  status == ESP_NOW_SEND_SUCCESS ;

  if(isCheckingServer)
  {
    Serial.println("CheckServerConnected:" + String(status));
    if(status == 0)
      isWiFiConnected = true;
    else
      isWiFiConnected = false;
  }
}

void Task_1_sendData(void *pvParameters)
{
  while (true)
  {
    //Call ESP-Now receive data function
    esp_now_register_recv_cb(OnDataRecv);

    //Task1休息，delay(1)不可省略
    delay(10);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(20); //設定序列埠接收資料時的最大等待時間

  EEPROM.begin(4096); //宣告使用EEPROM 512 個位置

  //I2C Setting for 16 bits adc (Get PD value)
  I2CADS.begin(I2C_SDA, I2C_SCL, 100000);
  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
  if (!ads.begin(0x48, &I2CADS)) {
    Serial.println("Failed to initialize ADS.");
  }  

#pragma region : WiFi Server Setting

  // 初始化 ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
    Serial.println("Initializing ESP-NOW");

  ThisAddr = WiFi.macAddress();

  // 取得本機的MACAddress
  Serial.print("ESP32 Board MAC Address:  ");
  Serial.println(ThisAddr);  

  char Colon = ':';
  int startP = 0;
  uint8_t value;
  int indexCount = 0;

  //This Address : String to Hex
  while (indexCount < sizeof(ThisAddress))
  {
    int index = ThisAddr.indexOf(Colon, startP);
    if(index != -1)
    {
      String subS = ThisAddr.substring(startP, index);
      value = strtoul(subS.c_str(), NULL, 16);  //string to Hex
      startP = index + 1;

      ThisAddress[indexCount] = value;
      // Serial.println("value:" + String(value, HEX));      
    }
    else  //last one
    {
        String subS = ThisAddr.substring(startP);
      value = strtoul(subS.c_str(), NULL, 16);
      ThisAddress[indexCount] = value;
      // Serial.println("value:" + String(value, HEX));      
    }
    indexCount++;
  }

  // 設置發送數據回傳函數
  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // 绑定數據接收端
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));  //initialize peer if esp32 library version is 2.0.1 (no need in version 1.0.6)
  memcpy(peerInfo.peer_addr, ControllerAddress, 6); // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // 绑定Server數據接收端
  esp_now_peer_info_t peerInfo_server;
  memset(&peerInfo_server, 0, sizeof(peerInfo_server));  //initialize peer if esp32 library version is 2.0.1 (no need in version 1.0.6)
  memcpy(peerInfo_server.peer_addr, ServerAddress, 6); // Register peer
  peerInfo_server.channel = 0;
  peerInfo_server.encrypt = false;

  // // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer_controller");
    return;
  }

  if (esp_now_add_peer(&peerInfo_server) != ESP_OK) {
    Serial.println("Failed to add peer_server");
    return;
  }

    

#pragma endregion

#pragma region pinMode Setting

  pinMode(LED_Align, OUTPUT);
  pinMode(X_STP_Pin, OUTPUT);
  pinMode(X_DIR_Pin, OUTPUT);
  pinMode(Y_STP_Pin, OUTPUT);
  pinMode(Y_DIR_Pin, OUTPUT);
  pinMode(Z_STP_Pin, OUTPUT);
  pinMode(Z_DIR_Pin, OUTPUT);

  pinMode(R_0, INPUT_PULLUP);
  attachInterrupt(R_0, EmergencyStop, FALLING);
    
  pinMode(R_2, INPUT_PULLUP); // /keyValue:5
  pinMode(R_3, INPUT_PULLUP); // /keyValue:0

  pinMode(C_1, OUTPUT); ///keyValue:1
  pinMode(C_2, OUTPUT); ///keyValue:2
  pinMode(C_3, OUTPUT); ///keyValue:3

  digitalWrite(C_1, false);
  digitalWrite(C_2, false);
  digitalWrite(C_3, false);

  // pinMode(PD_Pin, INPUT);                      //PD signal input
  pinMode(Tablet_PD_mode_Trigger_Pin, OUTPUT);    //Control Tablet Mode
  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

  digitalWrite(LED_Align, true);

#pragma endregion

  Serial.println("~~ Auto-Align System ~~");

 #pragma region EEPROM Setting

  String eepromString;

  //Initialize PD_Ref_Array
  int iniEP = 700;
  for (size_t i = 0; i < sizeof(EP_PD_Ref_Array) / sizeof(EP_PD_Ref_Array[0]); i++)
  {
    EP_PD_Ref_Array[i][0] = iniEP;
    EP_PD_Ref_Array[i][1] = (iniEP + 8);
    iniEP += 16;
    
    eepromString = ReadInfoEEPROM(EP_PD_Ref_Array[i][0], 8);
    PD_Ref_Array[i][0] = isNumberic(eepromString) ? eepromString.toInt() : PD_Ref_Array[i][0];

    eepromString = ReadInfoEEPROM(EP_PD_Ref_Array[i][1], 8);
    PD_Ref_Array[i][1] = isNumberic(eepromString) ? eepromString.toInt() : PD_Ref_Array[i][1];

    Serial.printf("[%d, %d]:[%d, %d]\r",  EP_PD_Ref_Array[i][0],  EP_PD_Ref_Array[i][1], PD_Ref_Array[i][0], PD_Ref_Array[i][1]);
  }


  for (int i = 0; i < 511; i = i + 8)
  {
    eepromString = ReadInfoEEPROM(i, 8); //Reading EEPROM(int start_position, int data_length)
    MSGOutput("EEPROM(" + String(i) + ") - " + eepromString);
  }


  eepromString = ReadInfoEEPROM(0, 8); //Reading EEPROM(int start_position, int data_length)
  ref_Dac = isNumberic(eepromString) ? eepromString.toDouble() : ref_Dac;
  // ref_Dac = eepromString.toDouble();
  ref_IL = ILConverter(ref_Dac);
  MSGOutput("Ref IL: " + String(ref_IL));

  ID = ReadInfoEEPROM(8, 8);
  MSGOutput("Board ID: " + ReadInfoEEPROM(8, 8)); 

  Station_ID = ReadInfoEEPROM(16, 8);
  MSGOutput("Station ID: " + ReadInfoEEPROM(16, 8)); 
  sendmsg_server.client_name = Station_ID;

  eepromString = ReadInfoEEPROM(24, 8);
  X_backlash = isNumberic(eepromString) ? eepromString.toInt() : X_backlash;
  // X_backlash = eepromString.toInt();
  MSGOutput("X_backlash: " + String(X_backlash));

  eepromString = ReadInfoEEPROM(32, 8);
  Y_backlash = isNumberic(eepromString) ? eepromString.toInt() : Y_backlash;
  // Y_backlash = eepromString.toInt();
  MSGOutput("Y_backlash: " + String(Y_backlash));

  eepromString = ReadInfoEEPROM(40, 8);
  Z_backlash = isNumberic(eepromString) ? eepromString.toInt() : Z_backlash;
  // Z_backlash = eepromString.toInt();
  MSGOutput("Z_backlash: " + String(Z_backlash));

  eepromString = ReadInfoEEPROM(48, 8);
  delayBetweenStep_X = isNumberic(eepromString) ? eepromString.toInt() : delayBetweenStep_X;
  // delayBetweenStep_X = eepromString.toInt();
  MSGOutput("delayBetweenStep_X: " + String(delayBetweenStep_X));

  eepromString = ReadInfoEEPROM(56, 8);
  delayBetweenStep_Y = isNumberic(eepromString) ? eepromString.toInt() : delayBetweenStep_Y;
  // delayBetweenStep_Y = eepromString.toInt();
  MSGOutput("delayBetweenStep_Y: " + String(delayBetweenStep_Y));

  eepromString = ReadInfoEEPROM(64, 8);
  delayBetweenStep_Z = isNumberic(eepromString) ? eepromString.toInt() : delayBetweenStep_Z;
  // delayBetweenStep_Z = eepromString.toInt();
  MSGOutput("delayBetweenStep_Z: " + String(delayBetweenStep_Z));

  eepromString = ReadInfoEEPROM(72, 8);
  Target_IL = isNumberic(eepromString) ? eepromString.toDouble() : Target_IL;
  // Target_IL = ReadInfoEEPROM(72, 8).toDouble();
  MSGOutput("Target IL: " + String(eepromString));
  MSGOutput("Target IL: " + String(Target_IL));

  eepromString = ReadInfoEEPROM(152, 8);
  Get_PD_Points = isNumberic(eepromString) ? eepromString.toInt() : Get_PD_Points;
  Get_PD_Points = Get_PD_Points == 0 ? 1 : Get_PD_Points;
  MSGOutput("Get_PD_Points: " + String(Get_PD_Points));

  eepromString = ReadInfoEEPROM(160, 8);
  AQ_Scan_Compensation_Steps_Z_A = isNumberic(eepromString) ? eepromString.toInt() : AQ_Scan_Compensation_Steps_Z_A;
  // AQ_Scan_Compensation_Steps_Z_A = ReadInfoEEPROM(160, 8).toInt();
  MSGOutput("AQ_Scan_Compensation_Steps_Z_A: " + String(AQ_Scan_Compensation_Steps_Z_A));

  eepromString = ReadInfoEEPROM(168, 8);
  AQ_Total_TimeSpan = isNumberic(eepromString) ? eepromString.toInt() : AQ_Total_TimeSpan;
  // AQ_Total_TimeSpan = ReadInfoEEPROM(168, 8).toInt();
  MSGOutput("AQ_Total_TimeSpan: " + String(AQ_Total_TimeSpan));

  eepromString = ReadInfoEEPROM(80, 8);
  AA_ScanFinal_Scan_Delay_X_A = isNumberic(eepromString) ? eepromString.toInt() : AA_ScanFinal_Scan_Delay_X_A;
  // AA_ScanFinal_Scan_Delay_X_A = ReadInfoEEPROM(80, 8).toInt();
  MSGOutput("AA_ScanFinal_Scan_Delay_X_A: " + String(AA_ScanFinal_Scan_Delay_X_A));

  eepromString = ReadInfoEEPROM(176, 8);
  AQ_Scan_Steps_Z_A = isNumberic(eepromString) ? eepromString.toInt() : AQ_Scan_Steps_Z_A;
  // AQ_Scan_Steps_Z_A = ReadInfoEEPROM(176, 8).toInt();
  MSGOutput("AQ_Scan_Steps_Z_A: " + String(AQ_Scan_Steps_Z_A));

  eepromString = ReadInfoEEPROM(184, 8);
  AQ_Scan_Steps_Z_B = isNumberic(eepromString) ? eepromString.toInt() : AQ_Scan_Steps_Z_B;
  // AQ_Scan_Steps_Z_B = ReadInfoEEPROM(184, 8).toInt();
  MSGOutput("AQ_Scan_Steps_Z_B: " + String(AQ_Scan_Steps_Z_B));

  eepromString = ReadInfoEEPROM(192, 8);
  AQ_Scan_Steps_Z_C = isNumberic(eepromString) ? eepromString.toInt() : AQ_Scan_Steps_Z_C;
  // AQ_Scan_Steps_Z_C = ReadInfoEEPROM(192, 8).toInt();
  MSGOutput("AQ_Scan_Steps_Z_C: " + String(AQ_Scan_Steps_Z_C));

  eepromString = ReadInfoEEPROM(200, 8);
  AQ_Scan_Steps_Z_D = isNumberic(eepromString) ? eepromString.toInt() : AQ_Scan_Steps_Z_D;
  // AQ_Scan_Steps_Z_D = ReadInfoEEPROM(200, 8).toInt();
  MSGOutput("AQ_Scan_Steps_Z_D: " + String(AQ_Scan_Steps_Z_D));

  eepromString = ReadInfoEEPROM(208, 8);
  MotorStepRatio = isNumberic(eepromString) ? eepromString.toDouble() : MotorStepRatio;
  MotorStepRatio = MotorStepRatio == 0 ? 1 : MotorStepRatio;
  MSGOutput("MotorStepRatio: " + String(MotorStepRatio));

  eepromString = ReadInfoEEPROM(216, 8);
  MotorStepDelayRatio = isNumberic(eepromString) ? eepromString.toDouble() : MotorStepDelayRatio;
  MotorStepDelayRatio = MotorStepDelayRatio == 0 ? 1 : MotorStepDelayRatio;
  MSGOutput("MotorStepDelayRatio: " + String(MotorStepDelayRatio));

  eepromString = ReadInfoEEPROM(EP_FS_Count_X, 8);
  FS_Count_X = isNumberic(eepromString) ? eepromString.toInt() : FS_Count_X;
  // FS_Count_X = ReadInfoEEPROM(EP_FS_Count_X, 8).toInt();
  MSGOutput("FS_Count_X: " + String(FS_Count_X));

  eepromString = ReadInfoEEPROM(EP_FS_Steps_X, 8);
  FS_Steps_X = isNumberic(eepromString) ? eepromString.toInt() : FS_Steps_X;
  // FS_Steps_X = ReadInfoEEPROM(EP_FS_Steps_X, 8).toInt();
  MSGOutput("FS_Steps_X: " + String(FS_Steps_X));

  eepromString = ReadInfoEEPROM(EP_FS_Stable_X, 8);
  FS_Stable_X = isNumberic(eepromString) ? eepromString.toInt() : FS_Stable_X;
  // FS_Stable_X = ReadInfoEEPROM(EP_FS_Stable_X, 8).toInt();
  MSGOutput("FS_Stable_X: " + String(FS_Stable_X));

  eepromString = ReadInfoEEPROM(EP_FS_DelaySteps_X, 8);
  FS_DelaySteps_X = isNumberic(eepromString) ? eepromString.toInt() : FS_DelaySteps_X;
  // FS_DelaySteps_X = ReadInfoEEPROM(EP_FS_DelaySteps_X, 8).toInt();
  MSGOutput("FS_DelaySteps_X: " + String(FS_DelaySteps_X));

  eepromString = ReadInfoEEPROM(EP_FS_Avg_X, 8);
  FS_Avg_X = isNumberic(eepromString) ? eepromString.toInt() : FS_Avg_X;
  // FS_Avg_X = ReadInfoEEPROM(EP_FS_Avg_X, 8).toInt();
  MSGOutput("FS_Avg_X: " + String(FS_Avg_X));

  eepromString = ReadInfoEEPROM(EP_FS_Count_Y, 8);
  FS_Count_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_Count_Y;
  // FS_Count_Y = ReadInfoEEPROM(EP_FS_Count_Y, 8).toInt();
  MSGOutput("FS_Count_Y: " + String(FS_Count_Y));

  eepromString = ReadInfoEEPROM(EP_FS_Steps_Y, 8);
  FS_Steps_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_Steps_Y;
  // FS_Steps_Y = ReadInfoEEPROM(EP_FS_Steps_Y, 8).toInt();
  MSGOutput("FS_Steps_Y: " + String(FS_Steps_Y));

  eepromString = ReadInfoEEPROM(EP_FS_Stable_Y, 8);
  FS_Stable_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_Stable_Y;
  // FS_Stable_Y = ReadInfoEEPROM(EP_FS_Stable_Y, 8).toInt();
  MSGOutput("FS_Stable_Y: " + String(FS_Stable_Y));

  eepromString = ReadInfoEEPROM(EP_FS_DelaySteps_Y, 8);
  FS_DelaySteps_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_DelaySteps_Y;
  // FS_DelaySteps_Y = ReadInfoEEPROM(EP_FS_DelaySteps_Y, 8).toInt();
  MSGOutput("FS_DelaySteps_Y: " + String(FS_DelaySteps_Y));

  eepromString = ReadInfoEEPROM(EP_FS_Avg_Y, 8);
  FS_Avg_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_Avg_Y;
  // FS_Avg_Y = ReadInfoEEPROM(EP_FS_Avg_Y, 8).toInt();
  MSGOutput("FS_Avg_Y: " + String(FS_Avg_Y));

  eepromString = ReadInfoEEPROM(EP_FS_Count_Z, 8);
  FS_Count_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_Count_Z;
  // FS_Count_Z = ReadInfoEEPROM(EP_FS_Count_Z, 8).toInt();
  MSGOutput("FS_Count_Z: " + String(FS_Count_Z));

  eepromString = ReadInfoEEPROM(EP_FS_Steps_Z, 8);
  FS_Steps_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_Steps_Z;
  // FS_Steps_Z = ReadInfoEEPROM(EP_FS_Steps_Z, 8).toInt();
  MSGOutput("FS_Steps_Z: " + String(FS_Steps_Z));

  eepromString = ReadInfoEEPROM(EP_FS_Stable_Z, 8);
  FS_Stable_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_Stable_Z;
  // FS_Stable_Z = ReadInfoEEPROM(EP_FS_Stable_Z, 8).toInt();
  MSGOutput("FS_Stable_Z: " + String(FS_Stable_Z));

  eepromString = ReadInfoEEPROM(EP_FS_DelaySteps_Z, 8);
  FS_DelaySteps_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_DelaySteps_Z;
  // FS_DelaySteps_Z = ReadInfoEEPROM(EP_FS_DelaySteps_Z, 8).toInt();
  MSGOutput("FS_DelaySteps_Z: " + String(FS_DelaySteps_Z));

  eepromString = ReadInfoEEPROM(EP_FS_Avg_Z, 8);
  FS_Avg_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_Avg_Z;
  // FS_Avg_Z = ReadInfoEEPROM(EP_FS_Avg_Z, 8).toInt();
  MSGOutput("FS_Avg_Z: " + String(FS_Avg_Z));

  eepromString = ReadInfoEEPROM(EP_FS_Trips_X, 8);
  FS_Trips_X = isNumberic(eepromString) ? eepromString.toInt() : FS_Trips_X;
  // FS_Trips_X = ReadInfoEEPROM(EP_FS_Trips_X, 8).toInt();
  MSGOutput("FS_Trips_X: " + String(FS_Trips_X));

  eepromString = ReadInfoEEPROM(EP_FS_Trips_Y, 8);
  FS_Trips_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_Trips_Y;
  // FS_Trips_Y = ReadInfoEEPROM(EP_FS_Trips_Y, 8).toInt();
  MSGOutput("FS_Trips_Y: " + String(FS_Trips_Y));

  eepromString = ReadInfoEEPROM(EP_FS_Trips_Z, 8);
  FS_Trips_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_Trips_Z;
  // FS_Trips_Z = ReadInfoEEPROM(EP_FS_Trips_Z, 8).toInt();
  MSGOutput("FS_Trips_Z: " + String(FS_Trips_Z));

  eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Steps_Z_A, 8);
  AA_ScanRough_Feed_Steps_Z_A = isNumberic(eepromString) ? eepromString.toInt() : AA_ScanRough_Feed_Steps_Z_A;
  MSGOutput("AA_ScanRough_Feed_Steps_Z_A: " + String(AA_ScanRough_Feed_Steps_Z_A));

  eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Steps_Z_B, 8);
  AA_ScanRough_Feed_Steps_Z_B = isNumberic(eepromString) ? eepromString.toInt() : AA_ScanRough_Feed_Steps_Z_B;
  MSGOutput("AA_ScanRough_Feed_Steps_Z_B: " + String(AA_ScanRough_Feed_Steps_Z_B));

  eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Ratio_Z_A, 8);
  AA_ScanRough_Feed_Ratio_Z_A = isNumberic(eepromString) ? eepromString.toInt() : AA_ScanRough_Feed_Ratio_Z_A;
  MSGOutput("AA_ScanRough_Feed_Ratio_Z_A: " + String(AA_ScanRough_Feed_Ratio_Z_A));

  eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Ratio_Z_B, 8);
  AA_ScanRough_Feed_Ratio_Z_B = isNumberic(eepromString) ? eepromString.toInt() : AA_ScanRough_Feed_Ratio_Z_B;
  MSGOutput("AA_ScanRough_Feed_Ratio_Z_B: " + String(AA_ScanRough_Feed_Ratio_Z_B));

  eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Ratio_Z_C, 8);
  AA_ScanRough_Feed_Ratio_Z_C = isNumberic(eepromString) ? eepromString.toInt() : AA_ScanRough_Feed_Ratio_Z_C;
  MSGOutput("AA_ScanRough_Feed_Ratio_Z_C: " + String(AA_ScanRough_Feed_Ratio_Z_C));

  eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Ratio_Z_D, 8);
  AA_ScanRough_Feed_Ratio_Z_D = isNumberic(eepromString) ? eepromString.toInt() : AA_ScanRough_Feed_Ratio_Z_D;
  MSGOutput("AA_ScanRough_Feed_Ratio_Z_D: " + String(AA_ScanRough_Feed_Ratio_Z_D));

   //EP_PD_Ref_Array
  // MSGOutput("PD_Ref_Array:");
  // for (size_t i = 0; i < 15; i++)
  // {
  //   MSGOutput(String(PD_Ref_Array[i][0]) + ", "+ String(PD_Ref_Array[i][1]));
  // }

  #pragma endregion

  Cal_PD_Input_IL(Get_PD_Points);

  Serial.println("Get_PD_Points:" + String(Get_PD_Points));

  // Check Server is connected
  isWiFiConnected = true;
  isCheckingServer = true;
  DataSent_Server("ID?");
  Serial.println("isServerConnected:" + String(isWiFiConnected));
  isCheckingServer = false;

   //在core 0啟動 mision 1
  xTaskCreatePinnedToCore(
      Task_1_sendData, /* 任務實際對應的Function */
      "Task_1",        /* 任務名稱 */
      10000,           /* 堆疊空間 */
      NULL,            /* 無輸入值 */
      12,               /* 優先序0(0為最低) */
      &Task_1,         /* 對應的任務變數位址 */
      0);              /*指定在核心0執行 */
      
  motorType = Phase_5;
  if(motorType == HomeMade) isTrip3Jump = false;
  
  axisDir = X_Dir;
  Serial.println("axDir:" + String(axisDir));
  axisDir = Z_Dir;
  Serial.println("axDir:" + String(axisDir));
}

bool isMsgShow = false;
unsigned long previousMillis = 0;
const long interval = 150; //default:2000
String Data;

String ServerIP = "http://192.168.4.1/";
const char *serverTestData = "http://192.168.4.1/?param1=10&param2=hi";

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {    
    //Re-Initialize
    isStop = false;
    ButtonSelected = -1;

    //Keyboard Detect
    ButtonSelected = KeyValueConverter();
    if(ButtonSelected!=-1)
      Motor_Continous_Mode = 1;

    String rsMsg = "";
    if (Serial.available())
      rsMsg = Serial.readString();

    String cmd = rsMsg;
    if(cmd != "") 
      Motor_Continous_Mode = 2;

    if(cmd_from_contr != "")
    {
      cmd = cmd_from_contr;

      if(cmd_value_from_contr != "")
      {
        int v = cmd_value_from_contr.toDouble();

        if(!isStop)
        {
          Serial.print("msg:" + cmd_from_contr);
          Serial.println(", value:" + cmd_value_from_contr);
        }

        if(cmd == "BS")
        {
          ButtonSelected = v;
          Motor_Continous_Mode = 3;
        }

        // cmd_from_contr = "";
        // cmd_value_from_contr = "";
      }
    }

    cmd_No = Function_Classification(cmd, ButtonSelected);

    cmd_No = Function_Excecutation(cmd, cmd_No);

    // isGetPower
    if (isGetPower)
    {
      if (millis() - timer_Get_IL_1 > 500)
      {
        timer_Get_IL_1 = millis();

        double value;

        switch (GetPower_Mode)
        {
        case 1:
          value = Cal_PD_Input_IL(Get_PD_Points);
          break;

        case 2:
          value = Cal_PD_Input_Dac(Get_PD_Points);
          break;

        case 3:
          value = Cal_PD_Input_Row_IL(Get_PD_Points);
          break;

        case 4:
          value = Cal_PD_Input_Row_Dac(Get_PD_Points);
          break;

        default:
          break;
        }

        MSGOutput("PD_Power:" + String(value)); //dB
        DataSent_Server("PD_Power:" + String(value));  //Send to Server and check is connected or not !!
      }
    }

    cmd_from_contr = "";
    cmd_value_from_contr = "";
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

void Move_Motor_abs(int xyz, long Target)
{
  String axis = "";
  long Pos_Now = 0;
  switch (xyz)
  {
  case 0:
    MotorDir_Pin = X_DIR_Pin;
    MotorSTP_Pin = X_STP_Pin;
    Pos_Now = X_Pos_Now;
    axis = "X";
    break;
  case 1:
    MotorDir_Pin = Y_DIR_Pin;
    MotorSTP_Pin = Y_STP_Pin;
    Pos_Now = Y_Pos_Now;
    axis = "Y";
    break;
  case 2:
    MotorDir_Pin = Z_DIR_Pin;
    MotorSTP_Pin = Z_STP_Pin;
    Pos_Now = Z_Pos_Now;
    axis = "Z";
    break;
  }

  if (Target - Pos_Now < 0)
    MotorCC = false;
  else if (Target - Pos_Now > 0)
    MotorCC = true;
  else
    return;

  delayBetweenStep = 20;
  MinMotroStep = abs(Target - Pos_Now);

  MSGOutput(axis + " Go to position: " + String(Target) + ", origin position: " + String(Pos_Now));

  //byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay
  Move_Motor(MotorDir_Pin, MotorSTP_Pin, MotorCC, MinMotroStep, delayBetweenStep, 0);
}

void Move_Motor_abs_all(int x, int y, int z)
{
  Move_Motor_abs(0, x);
  Move_Motor_abs(1, y);
  Move_Motor_abs(2, z);
}

void Move_Motor(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay)
{
  MotorCC = dirt;
  digitalWrite(dir_pin, dirt);
  delay(2);
  if (moveSteps > 0)
  {
    step(stp_pin, moveSteps, delayStep);
    delay(stableDelay);
    DataOutput(false);
  }
}

void Move_Motor(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay, bool isOutputPosition)
{
  MotorCC = dirt;
  digitalWrite(dir_pin, dirt);
  delay(5);
  if (moveSteps > 0)
  {
    step(stp_pin, moveSteps, delayStep);
    delay(stableDelay);

    if (isOutputPosition)
      DataOutput(false);
  }
}

void Move_Motor_Cont(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep)
{
  MotorSTP_Pin = dir_pin;

  if (MotorDir_Pin != dir_pin || MotorCC != dirt)
  {
    MotorCC = dirt;
    MotorDir_Pin = dir_pin;
    digitalWrite(MotorDir_Pin, MotorCC); //步進馬達方向控制, false為負方向
    delay(3);
  }

  step(stp_pin, moveSteps, delayStep);
}

//------------------------------------------------------------------------------------------------------------------------------------------

String Region, msg;
bool Fine_Scan(int axis, bool Trip2Stop)
{
  isLCD = true;

  MSGOutput("");
  MSGOutput("Fine Scan ");

  MSGOutput("Stop Value: " + String(StopValue));
  
  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
  delay(5);

  Threshold = -37.3;
  // delayBetweenStep = 100;

  double pdBest = Cal_PD_Input_IL(Get_PD_Points);

  // Region = Region + "_Fine_Scan";
  String msg;

  bool K_OK = true;

  bool initial_wifi_status = isWiFiConnected;

  if (axis < 3)
  {
    switch (axis)
    {
    case 0:

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      MotorCC_X = digitalRead(X_DIR_Pin);

      CMDOutput("AS");
      K_OK = Scan_AllRange_TwoWay(0, FS_Count_X, FS_Steps_X * MotorStepRatio, FS_Stable_X, MotorCC_X, FS_DelaySteps_X, 0, FS_Avg_X, FS_Trips_X, "X Fine-Scan,Trip_");
      CMDOutput("%:");

      if (!K_OK)
      {
        MotorCC_X = digitalRead(X_DIR_Pin);

        CMDOutput("AS");
        Scan_AllRange_TwoWay(0, FS_Count_X, FS_Steps_X * MotorStepRatio, FS_Stable_X, MotorCC_X, FS_DelaySteps_X, StopValue, FS_Avg_X, FS_Trips_X, "X Re-Scan,Trip_");
        CMDOutput("%:");
      }

      break;

    case 1:

      PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);

      MotorCC_Y = digitalRead(Y_DIR_Pin);

      CMDOutput("AS");
      K_OK = Scan_AllRange_TwoWay(1, FS_Count_Y, FS_Steps_Y * MotorStepRatio, FS_Stable_Y, MotorCC_Y, FS_DelaySteps_Y, StopValue, FS_Avg_Y, FS_Trips_Y, "Y Fine-Scan,Trip_"); 
      CMDOutput("%:");

      if (!K_OK)
      {
        MotorCC_Y = digitalRead(Y_DIR_Pin);

        CMDOutput("AS");
        Scan_AllRange_TwoWay(1, FS_Count_Y, FS_Steps_Y * MotorStepRatio, FS_Stable_Y, MotorCC_Y, FS_DelaySteps_Y, StopValue, FS_Avg_Y, FS_Trips_Y, "Y Re-Scan,Trip_"); 
        CMDOutput("%:");
      }

      break;

    case 2:

      PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);

      CMDOutput("AS");
      K_OK = Scan_AllRange_TwoWay(2, FS_Count_Z, FS_Steps_Z * MotorStepRatio, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Fine-Scan,Trip_");
      CMDOutput("%:");

      if (!K_OK)
      {
        CMDOutput("AS");
        Scan_AllRange_TwoWay(2, FS_Count_Z, FS_Steps_Z * MotorStepRatio, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Re-Scan,Trip_");
        CMDOutput("%:");
      }

      break;
    }

    if(Q_Time !=0)
      MSGOutput("Scan at QTime:" + String(Q_Time));
  }
  //Case 4: all actions should be excuted
  else if (axis == 3)
  {
    // Region = Region + "_Fine_Scan (All Range)";
    CMDOutput("AS");
    msg = Region + "_Fine_Scan (All Range)" + ", Z Scan, Trip_";
    Scan_AllRange_TwoWay(2, 8, 125, 0, 0, 100, StopValue, 500, 2, "Z Scan, Trip_");
    CMDOutput("%:");

    if (isStop)
    {
      return true;
    }

    CMDOutput("AS");
    msg = Region + ", Y Scan, Trip_";
    Scan_AllRange_TwoWay(1, 7, 20, 0, 0, 120, StopValue, 500, 2, "Y Scan, Trip_");
    CMDOutput("%:");

    if (isStop)
    {
      return true;
    }

    CMDOutput("AS");
    msg = Region + ", X Scan, Trip_";
    Scan_AllRange_TwoWay(0, 8, 22, 0, 0, 120, StopValue, 500, 2, "X Scan, Trip_");
    CMDOutput("%:");
  }

  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
  delay(5);
  MSGOutput("Fine Scan End");

  // isLCD = true;
  // LCD_Update_Mode = 0;
  // LCD_PageNow = 100;

  isWiFiConnected = initial_wifi_status;
  MSGOutput("initial_wifi_status:" + String(initial_wifi_status));
}

//------------------------------------------------------------------------------------------------------------------------------------------

void AutoAlign()
{
  StopValue = Target_IL;

  Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, AA_SpiralRough_Feed_Steps_Z_A * MotorStepRatio, 12 * MotorStepDelayRatio, 150); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)

  unsigned long time1 = 0, time2 = 0, time3 = 0, time4 = 0, time5 = 0, time6 = 0, time7 = 0;
  double PD_LV1, PD_LV2, PD_LV3, PD_LV4, PD_LV5, PD_LV6;
  double PD_Now = 0;
  time1 = millis();
  MSGOutput(" ");
  CMDOutput("AA"); //Auto Align

  bool Init_WifiStatus = isWiFiConnected;
  isWiFiConnected = false;

  MSGOutput(" ");
  CMDOutput("AS"); //Align Start

  #pragma region - Spiral 1

  MSGOutput("... Spiral ...");

  //Spiral - Rough - 1
  int matrix_level = 10;
  CMDOutput("^X");
  CMDOutput("R:" + String(M_Level * 2 + 1));
  CMDOutput("C:" + String(M_Level * 2 + 1));

  delayBetweenStep = 10 * MotorStepDelayRatio;                           //default:15
  MinMotroStep = AA_SpiralRough_Spiral_Steps_XY_A  * MotorStepRatio; //2000
  Region = "Sprial(Rough)";
  AutoAlign_Spiral(matrix_level, -35, 0); //Input : (Sprial Level, Threshold, stable)  Threshold:42
  CMDOutput("X^");

  DataOutput();

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  matrix_level = 2;
  CMDOutput("^X");
  CMDOutput("R:" + String(M_Level * 2 + 1));
  CMDOutput("C:" + String(M_Level * 2 + 1));

  delayBetweenStep = 10 * MotorStepDelayRatio;                           //default:15
  MinMotroStep = AA_SpiralRough_Spiral_Steps_XY_A / 3  * MotorStepRatio; //2000
  Region = "Sprial(Rough 2)";
  AutoAlign_Spiral(matrix_level, -25, 0); //Input : (Sprial Level, Threshold, stable)  Threshold:42
  CMDOutput("X^");

  DataOutput();

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  int Threshold = -21.4; //default: 144
  stableDelay = 0;       //default : 25

  msg = Region + ",Y Scan, Trip_";

  if (PD_Now > -9)
    MinMotroStep = 25;
  else if (PD_Now > -16 && PD_Now <= -9)
    MinMotroStep = 30;
  else if (PD_Now > -22 && PD_Now <= -16)
    MinMotroStep = 40;
  else
    MinMotroStep = 50;
  AutoAlign_Scan_DirectionJudge_V2(Y_Dir, 20, Threshold, MinMotroStep, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, -4.7, msg);

  CMDOutput("%:");

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  msg = Region + ",X Scan, Trip_";

  if (PD_Now > -9)
    MinMotroStep = 30;
  else if (PD_Now > -16 && PD_Now <= -9)
    MinMotroStep = 40;
  else if (PD_Now > -22 && PD_Now <= -16)
    MinMotroStep = 60;
  else if (PD_Now > -29.6)
    MinMotroStep = 80;
  else
    MinMotroStep = 140;
  AutoAlign_Scan_DirectionJudge_V2(X_Dir, 17, Threshold, MinMotroStep * MotorStepRatio, stableDelay, MotorCC_X, delayBetweenStep, Get_PD_Points, -4.7, msg);

  CMDOutput("%:");

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  PD_LV1 = PD_Now;
  time2 = millis();
  MSGOutput("Sprial(Rough) TimeSpan : " + String((time2 - time1) / 1000) + " s");
  MSGOutput(" ");

  if (isStop)
    return;

  MSGOutput(String(PD_LV1));
  MSGOutput(String(isStop));

  //Spiral Fine Scan
  if (PD_LV1 < -40) //default: 300
  {
    Region = "Sprial(Fine)";
    int matrix_level = 10;
    CMDOutput("^X");
    CMDOutput("R:" + String(M_Level * 2 + 1));
    CMDOutput("C:" + String(M_Level * 2 + 1));

    delayBetweenStep = 25 * MotorStepDelayRatio;
    MinMotroStep = AA_SpiralFine_Spiral_Steps_XY_A  * MotorStepRatio; //1500
    AutoAlign_Spiral(matrix_level, -37, 0);         //Input : (Sprial Level, Threshold, stable)  Threshold:166

    CMDOutput("X^");
  }

  #pragma endregion

  PD_LV2 = Cal_PD_Input_IL(Get_PD_Points);
  time3 = millis();

  if (isStop)
    return;

  PD_Now = PD_LV2;
  if (PD_Now < -50)
  {
    MSGOutput("High loss after spiral scan");
    return;
  }

  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd_No = Function_Excecutation(cmd, Function_Classification(cmd, -1));
    cmd = ""; //Reset command from serial port
  }

  #pragma region - Rough Scan

  MSGOutput(" ");
  MSGOutput("... X, Y, Z Scan(Rough) ...");
  Region = "Scan(Rough)";
  double PD_Before = 0;
  double PD_After = 0;
  delayBetweenStep = 50; //Default 50
  Threshold = 124;
  stableDelay = 10; //Default 25
  int scanPoints = 8;
  int stopValue = -3; //Default : -2.9
  int delta_X, delta_Y, X_pos_before, Y_pos_before;
  double PDValue_After_Scan = -60;

  //Rough-Scan
  if (PD_Now < -4 && true)
  {
    MSGOutput("Scan(Rough)");
    for (int i = 0; i < 15; i++)
    {
      //Scan(Rough) : Feed Z Loop
      if (true)
      {
        if (i > 0 && abs(PD_After - PD_Before) < 0.1) //1.2
        {
          MSGOutput("Break_Loop... :" + String(PD_After) + "," + String(PD_Before));
          return;
        }
        else
          MSGOutput("XYZ_Scan(Rough) Round :" + String(i + 1) + ",After:" + String(PD_After) + ",Before:" + String(PD_Before));

        PD_Before = Cal_PD_Input_IL(Get_PD_Points);

        if (PD_Before > stopValue) break;

        stableDelay = 100;
        double PD_Z_before = 0;
        for (int r = 0; r < 5; r++)
        {
          if (isStop) return;

          PD_Z_before = Cal_PD_Input_IL(Get_PD_Points);
          MSGOutput("PD_Z_before:" + String(PD_Z_before));

          int motorStep = AA_ScanRough_Feed_Steps_Z_A; //default: 10000
          if (PD_Z_before < -38)
          {
            motorStep = AA_ScanRough_Feed_Steps_Z_A;
            Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, motorStep * MotorStepRatio, 20 * MotorStepDelayRatio, stableDelay); 
          }
          else
          {
            double ratio_idx = AA_ScanRough_Feed_Ratio_Z_A;
            if (PD_Z_before > -37.5 && PD_Z_before <= -22.7)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_A; //default: 3.2
            else if (PD_Z_before > -22.7 && PD_Z_before <= -18.2)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_B; //default: 2.9
            else if (PD_Z_before > -18.2 && PD_Z_before <= -9)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_C; //default: 2.5
            else if (PD_Z_before > -9)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_D; //default: 1.8

            //default: 0.5
            motorStep = abs(ratio_idx * (588 + (55 * abs(PD_Z_before)))); //Default: 588-55*pd
            if (motorStep < AA_ScanRough_Feed_Steps_Z_B)
              motorStep = AA_ScanRough_Feed_Steps_Z_B; //default: 1000

            MSGOutput("ratio_idx:" + String(ratio_idx));

            Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, motorStep * MotorStepRatio, 50 * MotorStepDelayRatio, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
          }

          MSGOutput("Z_feed:" + String(motorStep * MotorStepRatio));
          DataOutput();

          PD_Now = Cal_PD_Input_IL(Get_PD_Points);

          if (PD_Now > stopValue)
            MSGOutput("Over_Stop_Value");

          if (PD_Now <= PD_Z_before || (PD_Z_before - PD_Now) > 30 || abs(PD_Z_before - PD_Now) <= 1.5)
          {
            MSGOutput("Z_feed_break,Now:" + String(PD_Now) + ",Before:" + String(PD_Z_before) + ",Z_Pos_Now:" + String(Z_Pos_Now));
            MSGOutput(" ");
            break;
          }
          else
            MSGOutput("Z_feed,Now:" + String(PD_Now) + ",Before:" + String(PD_Z_before) + ",Z_Pos_Now:" + String(Z_Pos_Now));
        }
      }

      Threshold = 120;
      scanPoints = 11;
      stableDelay = 25; //default: 25

      //Scan(Rough) : Spiral, if IL<-54
      if (PD_Now < -40)
      {
        MSGOutput("Spiral:z_feed_region");

        CMDOutput("^X");
        CMDOutput("R:" + String(M_Level * 2 + 1));
        CMDOutput("C:" + String(M_Level * 2 + 1));

        MinMotroStep = 300; //350
        if (!AutoAlign_Spiral(11, -36.4, 0))
        {
          CMDOutput("X^");
          MSGOutput("Spiral:Target_IL_not_found");
          MSGOutput(" ");

          return;
        }
        CMDOutput("X^");

        MSGOutput(" ");

        if (isStop)
          return;

        stableDelay = 0;

        Region = "Scan(Rough)(1)";

        msg = Region + ",Y_Scan" + ",Trip_";

        AutoAlign_Scan_DirectionJudge_V2(Y_Dir, 20, Threshold, 150  * MotorStepRatio, stableDelay * MotorStepDelayRatio, MotorCC_Y, delayBetweenStep, Get_PD_Points, 279, msg);

        CMDOutput("%:");

        msg = Region + ",X_Scan" + ",Trip_";

        AutoAlign_Scan_DirectionJudge_V2(X_Dir, 25, Threshold, 150  * MotorStepRatio, stableDelay * MotorStepDelayRatio, MotorCC_X, delayBetweenStep, Get_PD_Points, 279, msg);

        CMDOutput("%:");

        PD_Now = Cal_PD_Input_IL(Get_PD_Points);
        if (PD_Now < -54)
          return;

        double b, a;
        int rd = 0;
        for (int s = 0; s < rd; s++)
        {
          b = Cal_PD_Input_IL(Get_PD_Points);

          if (isStop)
            return;

          Region = "Scan(Rough)(2)";

          //(int XYZ, int count, int Threshold, int motorStep, int stableDelay,
          //bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg)

          msg = Region + ",Y_Scan" + ",Trip_";
          AutoAlign_Scan_DirectionJudge_V2(Y_Dir, 15, Threshold, 80 * MotorStepRatio, stableDelay * MotorStepDelayRatio, MotorCC_Y, delayBetweenStep, Get_PD_Points, stopValue, msg);
          CMDOutput("%:");

          msg = Region + ",X_Scan" + ",Trip_";
          AutoAlign_Scan_DirectionJudge_V2(X_Dir, 13, Threshold, 100 * MotorStepRatio, stableDelay * MotorStepDelayRatio, MotorCC_X, delayBetweenStep, Get_PD_Points, stopValue, msg);
          CMDOutput("%:");

          a = Cal_PD_Input_IL(Get_PD_Points);
          if (abs(a - b) < 5)
            break;
        }
      }

      PD_After = Cal_PD_Input_IL(Get_PD_Points);
      if (PD_After > stopValue) break;

      //Scan(Rough) : XY Scan
      if (true)
      {
        delayBetweenStep = 80;
        MinMotroStep = 350;                        //800
        stableDelay = AA_ScanFinal_Scan_Delay_X_A; //default:0

        if (isStop)
          return;

        Region = "Scan(Rough)(3)";

        msg = Region + ",Y_Scan" + ",Trip_";
        if (PD_After > -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_A; //default:25
        else if (PD_After > -16 && PD_After <= -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_B; //default:30
        else if (PD_After > -22.7 && PD_After <= -16)
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_C; //default:40
        else
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_D; //default:70

        CMDOutput("AS");
        MSGOutput("Gap:" + String(MinMotroStep * MotorStepRatio));
        PD_After = AutoAlign_Scan_DirectionJudge_V2(Y_Dir, 20, Threshold, MinMotroStep * MotorStepRatio, stableDelay * MotorStepDelayRatio, MotorCC_Y, delayBetweenStep, Get_PD_Points, Target_IL, msg);
        CMDOutput("%:");

        msg = Region + ",X_Scan" + ",Trip_";
        if (PD_After > -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_A; //default:25
        else if (PD_After > -16 && PD_After <= -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_B; //default:30
        else if (PD_After > -22.7 && PD_After <= -16)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_C; //default:80
        else if (PD_After > -30 && PD_After <= -22.7)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_D; //default:100
        else
          MinMotroStep = AA_ScanRough_Scan_Steps_X_E; //default:120

        CMDOutput("AS");
        MSGOutput("Gap:" + String(MinMotroStep * MotorStepRatio));
        PD_After = AutoAlign_Scan_DirectionJudge_V2(X_Dir, 20, Threshold, MinMotroStep * MotorStepRatio, stableDelay * MotorStepDelayRatio, MotorCC_X, delayBetweenStep, Get_PD_Points, Target_IL, msg);
        CMDOutput("%:");
      }

      PD_After = Cal_PD_Input_IL(Get_PD_Points);

      DataOutput();

      //Scan(Rough) : stop condition
      if (true)
      {
        if (abs(PD_After - PDValue_After_Scan) < 0.1 && PD_After > -10) //default:16
        {
          MSGOutput("Scan(Rough)(A)-Pass_best_Z_position");
          MSGOutput(String(PD_After) + "," + String(PDValue_After_Scan));
          break;
        }

        if (PD_After < PDValue_After_Scan && PD_After > -8) //default:10
        {
          MSGOutput("Scan(Rough)(B)-Pass_best_Z_position:" + String(PD_After) + "," + String(PDValue_After_Scan));
          break;
        }

        if (PD_After < -42) //default:16
        {
          MSGOutput("Scan(Rough)(C)-Pass_best_Z_position");
          Serial.println(String(PD_After) + "," + String(PDValue_After_Scan));
          break;
        }

        if (Serial.available())
        {
          String cmd = Serial.readStringUntil('\n');
          cmd_No = Function_Excecutation(cmd, Function_Classification(cmd, -1));
          cmd = ""; //Reset command from serial port
        }

        PDValue_After_Scan = PD_After;

        if (i == 1)
        {
          X_pos_before = X_Pos_Now;
          Y_pos_before = Y_Pos_Now;
        }
        else if (i > 1)
        {
          delta_X = X_Pos_Now - X_pos_before;
          delta_Y = Y_Pos_Now - Y_pos_before;
          X_pos_before = X_Pos_Now;
          Y_pos_before = Y_Pos_Now;
          Serial.println("------- Delta_X:" + String(delta_X) + ",Delta_Y:" + String(delta_Y));
        }

        if (PD_After > stopValue)
        {
          Serial.println("Better_than_stopValue:" + String(PD_After) + ",stopvalue:" + String(stopValue));
          break;
        }

        if (PD_After < -54)
        {
          Serial.println("Rough_Scan:High_loss");
          return;
        }
      }
    }
  }

  #pragma endregion

  PD_LV3 = Cal_PD_Input_IL(Get_PD_Points);
  time4 = millis();

  if (isStop)
    return;

  #pragma region Fine Scan

  //Scan(Fine)
  Serial.println(" ");
  Serial.println("........ XYZ_Scan(Fine) ........");
  Region = "Scan(Fine)";

  if (true && PD_Now > -25)
  {
    for (size_t i = 0; i < 1; i++)
    {
      PD_Before = Cal_PD_Input_IL(2 * Get_PD_Points);

      Fine_Scan(Z_Dir, false); 

      DataOutput(false);

      if (Serial.available())
      {
        String cmd = Serial.readStringUntil('\n');
        cmd_No = Function_Excecutation(cmd, Function_Classification(cmd, -1));
        cmd = ""; //Reset command from serial port
      }

      Fine_Scan(Y_Dir, false); 

      DataOutput(false);

       if (Serial.available())
      {
        String cmd = Serial.readStringUntil('\n');
        cmd_No = Function_Excecutation(cmd, Function_Classification(cmd, -1));
        cmd = ""; //Reset command from serial port
      }

      Fine_Scan(X_Dir, false); 

      DataOutput(false);

      PD_After = Cal_PD_Input_IL(2 * Get_PD_Points);

      if(PD_After <= PD_Before || abs(PD_Before - PD_After) <= 0.12)
      {
        MSGOutput("Final Fine-Scan loop break");
        break;
      }
    }
  }

  #pragma endregion

  PD_LV4 = Cal_PD_Input_IL(Get_PD_Points);
  time5 = millis();

  PDValue_Best = -2.45;

  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

  if (isStop)
    return;

  isLCD = true;

  Serial.println(" ");
  Serial.println("Sprial(Rough)_TimeSpan:" + String((time2 - time1) / 1000) + "s,PD:" + String(PD_LV1));
  Serial.println("Sprial(Fine)_TimeSpan:" + String((time3 - time2) / 1000) + "s,PD:" + String(PD_LV2));
  Serial.println("Scan(Rough)_TimeSpan:" + String((time4 - time3) / 1000) + "s,PD:" + String(PD_LV3));
  Serial.println("Scan(Fine)_TimeSpan:" + String((time5 - time4) / 1000) + "s,PD:" + String(PD_LV4));
  Serial.println("Auto_Align_TimeSpan:" + String((time5 - time1) / 1000) + "s");
  DataOutput(true);
    isWiFiConnected = Init_WifiStatus;
}

//------------------------------------------------------------------------------------------------------------------------------------------

int matrix_edge;
int x = 0, y = 0;
double AutoAlign_Result[3] = {0, 0, 0};

bool AutoAlign_Spiral(int M, double StopValue, int stableDelay)
{
  DataOutput(false);
  CMDOutput("ST" + String(MinMotroStep));
  Serial.println("StopValue:" + String(StopValue));
  Serial.println("stableDelay:" + String(stableDelay));

  double ts = 0;
  unsigned long timer_1 = 0, timer_2 = 0;
  timer_1 = millis();

  double PD_BestIL = -100, PD_Now = -100;
  int PD_BestIL_Position[2];
  int PD_Best_Pos_Abs[2];

  double SpiralStop_Threshold = StopValue; //Default : 198
  bool isFindThreshold = false;

  AutoAlign_Result[0] = 0;
  AutoAlign_Result[1] = 0;
  AutoAlign_Result[2] = 0;

  matrix_edge = 2 * M + 1;
  x = 0;
  y = 0;

  PD_BestIL = -100;
  PD_BestIL_Position[0] = x;
  PD_BestIL_Position[1] = y;

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  int m = 1;

  CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now); //[0,0]
  PD_BestIL = PD_Now;
  PD_BestIL_Position[0] = x;
  PD_BestIL_Position[1] = y;
  PD_Best_Pos_Abs[0] = X_Pos_Now;
  PD_Best_Pos_Abs[1] = Y_Pos_Now;
  Serial.println("Inital:(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));

  if (PD_Now >= SpiralStop_Threshold)
  {
    Serial.println("Over Threshold: " + String(PD_Now) + ", Threshold: " + String(SpiralStop_Threshold));
    return true;
  }

  for (int n = 1; abs(n) < (M + 1); n++)
  {
    if (isStop)
      return true;

    CMDOutput("ML");
    Serial.println("Matrix Layers: " + String(n));

    if (n > m)
      m++;

    y--;
    MotorCC = false;
    Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay, true);

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    // Serial.println("$[" + String(x) + "," + String(y) + "," + String(Z_Pos_Now) + "]=" + PD_Now);  //[0,-1]
    CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now); //[0,-1]

    if (PD_Now > PD_BestIL)
    {
      PD_BestIL = PD_Now;
      PD_BestIL_Position[0] = x;
      PD_BestIL_Position[1] = y;
      PD_Best_Pos_Abs[0] = X_Pos_Now;
      PD_Best_Pos_Abs[1] = Y_Pos_Now;

      if (PD_Now >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_Now) + ", Threshold: " + String(SpiralStop_Threshold));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }
    }

    x--;

    if (isStop)
      return true;

    //To Left

    MotorCC = false;

    while (x >= (-n))
    {
      Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          Serial.println(String(PD_BestIL_Position[0]) + ", " + String(PD_BestIL_Position[1]));
          return true;
        }
      }
      x--;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL) + ", Threshold: " + String(SpiralStop_Threshold));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }

      if (isStop)
        return true;
    }

    x++;
    y++;

    if (isStop)
      return true;

    //Up

    MotorCC = true;

    int nM = n;
    while (y <= (nM))
    {
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          return true;
        }
      }
      y++;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }

      if (isStop)
        return true;
    }

    y--;
    x++;

    if (isStop)
      return true;

    //To Right

    MotorCC = true;

    while (x <= (n))
    {
      Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          return true;
        }
      }
      x++;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }

      if (isStop)
        return true;
    }

    x--;
    y--;

    if (isStop)
      return true;

    //Down

    MotorCC = false;

    while (y >= (-n))
    {
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          return true;
        }
      }
      y--;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }

      if (isStop)
        return true;
    }

    y++;
  }

  CMDOutput("ML");
  Serial.println("Matrix Layers: Max");

  if (isStop)
    return true;

  int delta_X = 0, delta_Y = 0;

  if (!sprial_JumpToBest)
  {
    PD_BestIL_Position[0] = 0;
    PD_BestIL_Position[1] = 0; //Jump to (0,0)
  }

  if (PD_BestIL_Position[0] <= 2 * M && PD_BestIL_Position[1] <= 2 * M)
  {
    Move_Motor_abs(0, PD_Best_Pos_Abs[0]);
    Move_Motor_abs(1, PD_Best_Pos_Abs[1]);

    delay(200);

    double finalIL = Cal_PD_Input_IL(Get_PD_Points);
    Serial.println("Final IL : " + String(finalIL));

    Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));

    AutoAlign_Result[0] = PD_BestIL_Position[0];
    AutoAlign_Result[1] = PD_BestIL_Position[1];
    AutoAlign_Result[2] = finalIL;

    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));
  }
  else
  {
    Serial.println("Delta step out of range.");
  }
  return false;
}

//------------------------------------------------------------------------------------------------------------------------------------------

bool Scan_Fast(int XYZ, int count, int motorStep, int stableDelay,
                          bool Direction, int delayBetweenStep, double StopPDValue, int Get_PD_Points, int Trips, String msg)
{
  int DIR_Pin = 0;
  int STP_Pin = 0;
  int backlash = 40;
  MotorCC = Direction; // initial direction
  int trip = 1;
  int dataCount = 3;
  int dataCount_ori;
  int indexofBestIL = 0;
  double PD_Value[4 * count + 1];
  long Step_Value[4 * count + 1];
  unsigned long timer_1 = 0, timer_2 = 0;
  double PD_Best = -50;
  timer_1 = millis();

  dataCount = 2 * count + 1;
  dataCount_ori = dataCount;

  switch (XYZ)
  {
  case 0:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = X_backlash;
    delay(5);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = Y_backlash;
    delay(5);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = Z_backlash;
    delay(5);
    break;
  }

  MSGOutput("Scan Step: " + String(motorStep));
  MSGOutput("Backlash: " + String(backlash));
  CMDOutput(">>" + msg + String(trip));

  MSGOutput("StopValue:" + String(StopPDValue));

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);
  MSGOutput("Initial PD: " + String(PD_initial));

  
  if (PD_initial >= StopPDValue)
    return true;
  else
    PD_Best = PD_initial;
  //-------------------------------------------Jump to Trip_1 initial position-------------------------------------
   
  digitalWrite(DIR_Pin, MotorCC);
  delay(5); 

  //-------------------------------------------------------Trip_1 -----------------------------------------------
 
  for (int i = 0; i < dataCount; i++)
  {
    PD_Value[i] = 0;
    Step_Value[i] = 0;
  }

  double IL_Best_Trip1 = PD_initial;
  long Pos_Best_Trip1 = Get_Position(XYZ);
  long Pos_Ini_Trip1 = Get_Position(XYZ);

  int data_plus_time = 0;

  for (int i = 0; i < dataCount; i++)
  {
    if (isStop)
      return true;

    if (i == 0)
    {
      PD_Value[i] = PD_Now;
      Step_Value[i] = Get_Position(XYZ);
      continue;
    }

    step(STP_Pin, motorStep, delayBetweenStep);    
    delay(stableDelay);

    PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);

    Step_Value[i] = Get_Position(XYZ);

    if (PD_Value[i] > IL_Best_Trip1)
    {
      indexofBestIL = i;
      IL_Best_Trip1 = PD_Value[i];
      Pos_Best_Trip1 = Get_Position(XYZ);
    }    

    DataOutput();
    DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue
   
    if (PD_Value[i] >= StopPDValue)
    {
      MSGOutput("Better than StopValue");
      return true;
    }

    if(i > 3 && PD_Value[i]<PD_Value[i-1] && PD_Value[i-1]<PD_Value[i-2] && PD_Value[i-2]<PD_Value[i-3])
    {
      MSGOutput("Over Best IL 3 points, Break trip 1.");
      break;
    }    
  }

  PD_Best = IL_Best_Trip1;

  trip++;

  double Trip2_Initial_IL = 0;
  double IL_Best_Trip2 = 0;
  long Pos_Best_Trip2 = 0;
  long Pos_Ini_Trip2 = 0;

  //------------------------------------Trip_2 ------------------------------------------------------------
  MSGOutput(" --- Trip 2 --- " );
  MSGOutput("trip: " + String(trip) );
  
  if (true)
  {
    CMDOutput("~:" + msg + String(trip));

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    IL_Best_Trip2 = PD_Now;
    
    Pos_Best_Trip2 = Get_Position(XYZ);
    Pos_Ini_Trip2 = Get_Position(XYZ);

    for (int i = 0; i < dataCount; i++)
    {
      PD_Value[i] = 0;
      Step_Value[i] = 0;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    Trip2_Initial_IL = PD_Now;
    Serial.println("Trip2_Initial_IL:" + String(Trip2_Initial_IL));

    for (int i = 0; i < dataCount; i++)
    {
      if (isStop)
      {
        Serial.println("Emergency Stop");
        return true;
      }

      if (i == 0)
      {
        PD_Value[i] = PD_Now;
        Step_Value[i] = Get_Position(XYZ);
        continue;
      }

      step(STP_Pin, motorStep, delayBetweenStep);
      delay(stableDelay);

      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);
      Step_Value[i] = Get_Position(XYZ);

      if (PD_Value[i] > IL_Best_Trip2)
      {
        indexofBestIL = i;
        IL_Best_Trip2 = PD_Value[i];
        Pos_Best_Trip2 = Get_Position(XYZ);
      }

      DataOutput();
      DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

      if(i>3 && PD_Value[i] >= Trip2_Initial_IL && (PD_initial - PD_Value[i]) < 0.05 && PD_Value[i]<= PD_Value[i-1] && PD_Value[i-1]<= PD_Value[i-2])
      {
        Serial.println("Best Position, IL is: " + String(PD_Value[i]));
        break;
      }

       if(i>3 
        && (PD_Value[i] >= PD_initial || abs(PD_Value[i] - PD_initial) <= 0.02)
        && abs(PD_Value[i] - PD_Value[i-1])<=0.03 
        && abs(PD_Value[i-1] - PD_Value[i-2]) <= 0.03)
      {
        Serial.println("Best Position (2), IL is: " + String(PD_Value[i]));
        break;
      }

      // if(i>10 && PD_Value[i] < PD_initial && PD_Value[i]<= PD_Value[i-1] && PD_Value[i-1]<= PD_Value[i-2] && PD_Value[i-2]<= PD_Value[i-3])
      // {
      //   Serial.println("Scan Fail");
      //   return false;
      // }

      if (PD_Value[i] >= StopPDValue)
      {
        MSGOutput("Better than StopValue");
        break;
      }
     
    }
  }
  MSGOutput("IL_Best_Trip1: " + String(IL_Best_Trip1));
  MSGOutput("IL_Best_Trip2: " + String(IL_Best_Trip2));

  if(PD_Best < IL_Best_Trip2)
    PD_Best = IL_Best_Trip2;
  
  //------------------------------------Trip_3 -------------------------------------------------------

  PD_Now = Cal_PD_Input_IL(2*Get_PD_Points);
  MSGOutput("Best IL: " + String(PD_Best));
  MSGOutput("Final IL: " + String(PD_Now));

  timer_2 = millis();
  double ts = (timer_2 - timer_1) * 0.001;
  CMDOutput("t:" + String(ts, 2));

  if (PD_Now < PD_Best - 0.2)
    return false;
  else
    return true;
}

//------------------------------------------------------------------------------------------------------------------------------------------
double maxIL_in_FineScan = 0;
double minIL_in_FineScan = -100;

bool Scan_AllRange_TwoWay(int XYZ, int count, int motorStep, int stableDelay,
                          bool Direction, int delayBetweenStep, double StopPDValue, int Get_PD_Points, int Trips, String msg)
{
  int DIR_Pin = 0;
  int STP_Pin = 0;
  int backlash = 40;
  MotorCC = Direction; // initial direction
  int trip = 1;
  int dataCount = 3;
  int dataCount_ori;
  int indexofBestIL = 0;
  double PD_Value[4 * count + 1];
  long Step_Value[4 * count + 1];
  double Gradient_IL_Step[4 * count + 1];
  int GradientCount = 0;
  double GradientTarget = 0.005;  //default: 0.007
  unsigned long timer_1 = 0, timer_2 = 0;
  // delayBetweenStep = stableDelay;
  timer_1 = millis();

  dataCount = 2 * count + 1;
  dataCount_ori = dataCount;

  switch (XYZ)
  {
  case X_Dir:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = X_backlash;
    delay(5);
    break;
  case Y_Dir:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = Y_backlash;
    delay(5);
    break;
  case Z_Dir:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = Z_backlash;
    delay(5);
    break;
  }

  bool iniWifiStatus = isWiFiConnected;
  if(!is_AutoCuring)
    isWiFiConnected = false;

  MSGOutput("Scan Step: " + String(motorStep));
  MSGOutput("Backlash: " + String(backlash));
  MSGOutput("GradientTarget : " + String(GradientTarget, 4));
  MSGOutput("isWiFiConnected : " + String(isWiFiConnected));
  CMDOutput(">>" + msg + String(trip));

  MSGOutput("StopValue:" + String(StopPDValue));

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);
  MSGOutput("Initial PD: " + String(PD_initial));

  maxIL_in_FineScan = PD_initial;
  minIL_in_FineScan = PD_initial;

  if (PD_initial >= StopPDValue) return true;
    
  //-------------------------------------------Jump to Trip_1 initial position-------------------------------------
  digitalWrite(DIR_Pin, MotorCC);
  delay(1);

  if(true)
  {
    step(STP_Pin, motorStep * count, delayBetweenStep);   //First Jump
    delay(100);
    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput();
    DataOutput(XYZ, PD_Now); //int xyz, double pdValue

    Serial.println("Jump IL: " + String(PD_Now));
  }
  else
  {
    for (size_t i = 0; i < count; i++)
    {
      step(STP_Pin, motorStep, delayBetweenStep);
      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      if(PD_Now < (PD_initial - 3.5))
      {
        Serial.println("Jump IL < (IL - 3.5): " + String(PD_Now));
        break;
      }
    }
  }
   
  MotorCC = !MotorCC; //Reverse direction
  digitalWrite(DIR_Pin, MotorCC);

  delay(stableDelay + 105); 

  //-------------------------------------------------------Trip_1 -----------------------------------------------

  if (PD_Now >= StopPDValue)
  {
    maxIL_in_FineScan = 0;
    minIL_in_FineScan = -100;
    return true;
  }

  for (int i = 0; i < dataCount; i++)
  {
    PD_Value[i] = 0;
    Step_Value[i] = 0;
  }

  double IL_Best_Trip1 = PD_Now;
  long Pos_Best_Trip1 = Get_Position(XYZ);
  long Pos_Ini_Trip1 = Get_Position(XYZ);

  double IL_Best_Trip = PD_Now;
  long Pos_Best_Trip = Get_Position(XYZ);

  int data_plus_time = 0;

  for (int i = 0; i < dataCount; i++)
  {
    if (isStop) return true;

    if (i == 0)
    {
      PD_Value[i] = PD_Now;
      Step_Value[i] = Get_Position(XYZ);
      continue;
    }

    step(STP_Pin, motorStep, delayBetweenStep);    
    delay(stableDelay);

    if(i > 1 && PD_Value[i-1] > -2)
      delay(30);

    if(i>0 && PD_Value[i-1] > -2)
      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points * 3);  // 2500
    else
      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);

    Step_Value[i] = Get_Position(XYZ);

    if (PD_Value[i] > IL_Best_Trip1)
    {
      indexofBestIL = i;
      IL_Best_Trip1 = PD_Value[i];
      Pos_Best_Trip1 = Get_Position(XYZ);
    }

    //Update Min, Max IL in Scan Process
    if(PD_Value[i]>maxIL_in_FineScan)
        maxIL_in_FineScan=PD_Value[i];
    if(PD_Value[i]<minIL_in_FineScan)
        minIL_in_FineScan=PD_Value[i];    

    DataOutput();
    DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue
    DataSent_Server("PD Power:" + String(PD_Value[i]));

    //Gradient analyze
    if(i>0)
    {
      Gradient_IL_Step[i-1] = (PD_Value[i] - PD_Value[i-1]) / (motorStep / MotorStepRatio);

      if(XYZ == Z_Dir)
      {
        Gradient_IL_Step[i-1] *= (FS_Steps_Z / FS_Steps_X);
      }

      if(Gradient_IL_Step[i-1] <= -0.01)
        GradientCount = 0;
      else
        GradientCount++;

      if(i > 3)
      {
        if(PD_Value[i] > -1.6 && Gradient_IL_Step[i-1] <= GradientTarget && GradientCount > 3) //default: -1.35
        {
          MSGOutput("Gradient <= GradientTarget : " + String(Gradient_IL_Step[i-1], 4));

          timer_2 = millis();
          double ts = (timer_2 - timer_1) * 0.001;
          CMDOutput("t:" + String(ts, 2));

          isWiFiConnected = iniWifiStatus;

          return true;
        }        
      }
    }


    if(IL_Best_Trip1 >= -2 && PD_Value[i] <= (IL_Best_Trip1 - 1.5) && Trips == 1 && i>4)
    {
      Serial.println("IL < (IL-1.5): " + String(PD_Value[i]));     

      //Curfit
      if (indexofBestIL != 0 && Pos_Best_Trip1 != Get_Position(XYZ))
      {
        double x[3];
        double y[3];
        for (int k = -1; k < 2; k++)
        {
          x[k + 1] = Step_Value[indexofBestIL + k]; //idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];   //fill this with your sensor data
          Serial.println("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
        }
        Pos_Best_Trip1 = Curfit(x, y, 3);
        MSGOutput("Best IL position in Trip_1 is: " + String(Pos_Best_Trip1));
      }

      break;
    }

    if (PD_Value[i] >= StopPDValue)
    {
      MSGOutput("Better than StopValue");
      return true;
    }

    if(Trips == 0 && i > 3)
    {
      if( (PD_Value[i]<=PD_Value[i-1] || abs(PD_Value[i] - PD_Value[i-1]) <=0.02) && PD_Value[i]>=-1.8)
      {
        MSGOutput("Over best IL in trip 1");
        PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);
        MSGOutput("Final IL: " + String(PD_Now));
        timer_2 = millis();
        double ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        return true;
      }
    }

    if (i == (dataCount - 1) && Pos_Best_Trip1 == Get_Position(XYZ))
    {
      Serial.println("Datacount+3");
      dataCount = dataCount + 3;
      data_plus_time = data_plus_time + 1;

      if (dataCount - dataCount_ori > 20 || data_plus_time > 3)
      {
        Serial.println("Data plus time: " + String(data_plus_time));
        timer_2 = millis();
        double ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }
    }

    else if (indexofBestIL != 0 && i == (dataCount - 1) && Pos_Best_Trip1 != Get_Position(XYZ))
    {
      MSGOutput("i:" + String(i) + ", Pos_Best_Trip1:" + String(Pos_Best_Trip1));
      double x[3];
      double y[3];
      for (int k = -1; k < 2; k++)
      {
        x[k + 1] = Step_Value[indexofBestIL + k]; //idex * step = real steps
        y[k + 1] = PD_Value[indexofBestIL + k];   //fill this with your sensor data
        Serial.println("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
      }
      Pos_Best_Trip1 = Curfit(x, y, 3);
      MSGOutput("Best IL position in Trip_1 is: " + String(Pos_Best_Trip1));
    }
  }

  trip++;

  double IL_Best_Trip2 = 0;
  long Pos_Best_Trip2 = 0;
  long Pos_Ini_Trip2 = 0;

  //------------------------------------Trip_2 ------------------------------------------------------------
  MSGOutput(" --- Trip 2 --- " );
  MSGOutput("trip: " + String(trip) );
  MSGOutput("Trips: " + String(Trips) );

  GradientCount = 0;
  
  if (Trips != 1)
  {
    CMDOutput("~:" + msg + String(trip));

    IL_Best_Trip2 = PD_Now;
    Pos_Best_Trip2 = Get_Position(XYZ);
    Pos_Ini_Trip2 = Get_Position(XYZ);

    for (int i = 0; i < dataCount; i++)
    {
      PD_Value[i] = 0;
      Step_Value[i] = 0;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    for (int i = 0; i < dataCount; i++)
    {
      if (isStop)
      {
        Serial.println("Emergency Stop");
        return true;
      }

      if (i == 0)
      {
        PD_Value[i] = PD_Now;
        Step_Value[i] = Get_Position(XYZ);
        continue;
      }

      step(STP_Pin, motorStep, delayBetweenStep);
      delay(stableDelay);

      if(i > 1 && PD_Value[i-1] > -2)
       delay(30);

      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);
      Step_Value[i] = Get_Position(XYZ);

      if (PD_Value[i] > IL_Best_Trip2)
      {
        indexofBestIL = i;
        IL_Best_Trip2 = PD_Value[i];
        Pos_Best_Trip2 = Get_Position(XYZ);
      }

      //Update Min, Max IL in Scan Process
      if(PD_Value[i]>maxIL_in_FineScan)
          maxIL_in_FineScan=PD_Value[i];
      if(PD_Value[i]<minIL_in_FineScan)
          minIL_in_FineScan=PD_Value[i];

      DataOutput();
      DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue
      DataSent_Server("PD Power:" + String(PD_Value[i]));
      
      //Gradient analyze
      if(i>0)
      {
        Gradient_IL_Step[i-1] = (PD_Value[i] - PD_Value[i-1]) / motorStep;

        if(Gradient_IL_Step[i-1] <= -0.01)
          GradientCount = 0;
        else
          GradientCount++;

        if(i > 3)
        {
          if(PD_Value[i] > -1.35 && Gradient_IL_Step[i-1] <= GradientTarget && GradientCount > 3)
          {
            MSGOutput("Gradient <= 0.007 : " + String(Gradient_IL_Step[i-1], 4));

            timer_2 = millis();
            double ts = (timer_2 - timer_1) * 0.001;
            CMDOutput("t:" + String(ts, 2));

            isWiFiConnected = iniWifiStatus;

            return true;
          }
        }
      }

      if(IL_Best_Trip1 >= -2.5 && PD_Value[i] <= (IL_Best_Trip1 - 1.5) && Trips == 1)
      {
        Serial.println("IL < (IL-1.5): " + String(PD_Value[i]));
        break;
      }

      if (PD_Value[i] >= StopPDValue)
      {
        MSGOutput("Better than StopValue");
        return true;
      }

      if (indexofBestIL != 0 && i == (dataCount - 1) && Pos_Best_Trip2 != Get_Position(XYZ))
      {
        double x[3];
        double y[3];
        for (int k = -1; k < 2; k++)
        {
          x[k + 1] = Step_Value[indexofBestIL + k]; //idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];   //fill this with your sensor data
          MSGOutput("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
        }
        Pos_Best_Trip2 = Curfit(x, y, 3);
        MSGOutput("Best IL position in Trip_2 is: " + String(Pos_Best_Trip2));
      }
    }
  }
  else
    trip--;

  trip++;
  CMDOutput("~:" + msg + String(trip));

  //------------------------------------Trip_3 -------------------------------------------------------

  MSGOutput(" --- Trip 3 --- " );

  double PD_Best = IL_Best_Trip1;
  int deltaPos = 0;

  if(motorType == HomeMade)
  {
    //Best IL in Trip 2
    if (IL_Best_Trip2 > IL_Best_Trip1 && (IL_Best_Trip2 - IL_Best_Trip1)> 0.05 && Trips != 1)
    {
      if (isStop) return true;

      MotorCC = !MotorCC; //Reverse direction
      digitalWrite(DIR_Pin, MotorCC);
      delay(15);

      MSGOutput("Best pos in Trip_2 : " + String(Pos_Best_Trip2)); //------------Best in Trip_2----------------

      if (XYZ == 2)
        Pos_Best_Trip2 = Pos_Best_Trip2 - AQ_Scan_Compensation_Steps_Z_A; 

      MSGOutput("Best pos in Trip_2 (Compensation) : " + String(Pos_Best_Trip2));

      PD_Best = IL_Best_Trip2;

      Move_Motor_abs(XYZ, Pos_Ini_Trip2); //Jump to Trip_2 start position

      if(backlash > 0)
      {
        MSGOutput("Jump Backlash : " + String(backlash));
        step(STP_Pin, backlash, delayBetweenStep);
        delay(100);
      }

      deltaPos = abs(Pos_Best_Trip2 - Get_Position(XYZ));

      if (deltaPos < backlash)
      {
        MSGOutput("Jump Backlesh 2:" + String((backlash - deltaPos)));
        step(STP_Pin, (backlash - deltaPos), delayBetweenStep);
        delay(stableDelay + 200);

        deltaPos = abs(Pos_Best_Trip2 - Get_Position(XYZ));
      }

      delay(300);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      DataOutput();
      DataOutput(XYZ, PD_Now); //int xyz, double pdValue

      MotorCC = !MotorCC; //Reverse direction
      digitalWrite(DIR_Pin, MotorCC);
      delay(5);
    }

    //Best IL in Trip 1
    else if (Trips == 1)
    {
      MotorCC = !MotorCC; //Reverse direction
      digitalWrite(DIR_Pin, MotorCC);
      delay(15);

      MSGOutput("Jump to Trip Initial Pos : " + String(Pos_Ini_Trip1));
      Move_Motor_abs(XYZ, Pos_Ini_Trip1); //Jump to Trip_1 start position

      MSGOutput("Jump Backlash : " + String(backlash));
      step(STP_Pin, backlash, delayBetweenStep);

      delay(300); //100

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      DataOutput();
      DataOutput(XYZ, PD_Now); //int xyz, double pdValue

      MotorCC = !MotorCC; //Reverse direction
      digitalWrite(DIR_Pin, MotorCC);
      delay(5);

      deltaPos = abs(Pos_Best_Trip1 - Get_Position(XYZ));
      MSGOutput("deltaPos : " + String(deltaPos));
    }

    else //------------Best in Trip_1----------------
    {
      MSGOutput("Best in Trip_1 : " + String(Pos_Best_Trip1));
      MSGOutput("Position Now : " + String(Get_Position(XYZ)));

      if(Pos_Best_Trip1 == Get_Position(XYZ))
      {
        PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);
        if(abs(PD_Now - IL_Best_Trip1)<=0.12 || PD_Now > IL_Best_Trip1)
          return true;
        else
          return false;
      }
        
      if (XYZ == 2)
        Pos_Best_Trip1 = Pos_Best_Trip1 - AQ_Scan_Compensation_Steps_Z_A;

      MSGOutput("Best in Trip_1 (Compensation) : " + String(Pos_Best_Trip1));


      PD_Best = IL_Best_Trip1;
      deltaPos = abs(Pos_Best_Trip1 - Get_Position(XYZ));

      if (deltaPos < motorStep * 2)
      {
        MSGOutput("Jump Backlesh 1");
      
        step(STP_Pin, (backlash), delayBetweenStep+20);
        delay(stableDelay + 200);

        deltaPos = abs(Pos_Best_Trip2 - Get_Position(XYZ)); //Two curves are totally different, then back to best pos in trip 2
      }

      if (isStop) return true;

      MotorCC = !MotorCC; //Reverse direction
      digitalWrite(DIR_Pin, MotorCC);
      delay(2);
    }
  
    MSGOutput("Delta Pos : " + String(deltaPos));

    // isTrip3Jump = false;

    //Move to the best IL position in trip 1 & 2
    if(!isTrip3Jump)   
    {
      int failCount = 0;
      double preIL = Cal_PD_Input_IL(Get_PD_Points);
      
      while (true)
      {
        if (isStop) return true;

        //Feed a step to close to the best IL position
        if (deltaPos >= motorStep)
        {
          deltaPos = deltaPos - motorStep;

          Move_Motor(DIR_Pin, STP_Pin, MotorCC, motorStep, delayBetweenStep, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)

          PD_Now = Cal_PD_Input_IL(Get_PD_Points);
          DataOutput(XYZ, PD_Now); //int xyz, double pdValue

          if (PD_Now >= StopPDValue)
          {
            MSGOutput("StopPDValue");
            break;
          }
          if (PD_Now >= PD_Best)
          {
            MSGOutput("Reach IL_Best before");
            break;
          }
          if (preIL >= (PD_Best-0.09) && preIL >= PD_Now)
          {
            MSGOutput("Over IL_Best before");
            break;
          }

          if(PD_Now < preIL)
            preIL++;
          else
            preIL =0;

          if(preIL >=4) break;

          preIL = PD_Now;

        }
        
        //Feed final step to the best IL position
        else if (deltaPos > 0 && deltaPos < motorStep)
        {
          Move_Motor(DIR_Pin, STP_Pin, MotorCC, deltaPos, delayBetweenStep, 0); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
          PD_Now = Cal_PD_Input_IL(Get_PD_Points);
          DataOutput(XYZ, PD_Now); //int xyz, double pdValue
          break;
        }
        else
          break;
      }
    }
    else
    {  
      Move_Motor(DIR_Pin, STP_Pin, MotorCC, deltaPos, delayBetweenStep, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      DataOutput(XYZ, PD_Now); //int xyz, double pdValue
      MSGOutput("Trip 3 Jump");
    }
  }

  //5-Phase motor
  else
  {
    if(Trips ==2)
    {
      if(IL_Best_Trip2 > IL_Best_Trip1)
      {
        PD_Best = IL_Best_Trip2;
        Pos_Best_Trip = Pos_Best_Trip2;
      }
      else
      {
        PD_Best = IL_Best_Trip1;
        Pos_Best_Trip = Pos_Best_Trip1;
      }
    }
    else
    {
      Pos_Best_Trip = Pos_Best_Trip1;
      PD_Best = IL_Best_Trip1;
    }

    MSGOutput("Jump to best IL position : " + String(Pos_Best_Trip));

    Move_Motor_abs(XYZ, Pos_Best_Trip); //Jump to Trip_2 start position

    delay(100);

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput();
    DataOutput(XYZ, PD_Now); //int xyz, double pdValue
  }

  PD_Now = Cal_PD_Input_IL(2*Get_PD_Points);
  MSGOutput("Best IL: " + String(PD_Best));
  MSGOutput("Final IL: " + String(PD_Now));

  timer_2 = millis();
  double ts = (timer_2 - timer_1) * 0.001;
  CMDOutput("t:" + String(ts, 2));

  isWiFiConnected = iniWifiStatus;

  if (PD_Now < PD_Best - 0.5)
    return false;
  else
    return true;
}

//------------------------------------------------------------------------------------------------------------------------------------------

void BackLash_Reverse(int XYZ, bool dir, int stableDelay)
{
  int backlash = 40;
  int DIR_Pin = 0;
  int STP_Pin = 0;
  double Modify_Ratio = 1.3;

  switch (XYZ)
  {
  case 0:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = 60;    //default: 95
    Modify_Ratio = 1; //default:1.6
    delay(stableDelay);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = 40;    //default: 85
    Modify_Ratio = 1; //default:1.3
    delay(stableDelay);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = 500; //default: 1100
    Modify_Ratio = 1.5;
    delay(stableDelay);
    break;
  }

  MotorCC = dir;
  digitalWrite(DIR_Pin, MotorCC);
  delay(5);

  step(STP_Pin, (backlash + 20), delayBetweenStep); //Backlash about 40 pulse
  delay(stableDelay * 2);

  //Reverse
  MotorCC = !MotorCC;
  digitalWrite(DIR_Pin, MotorCC);
  delay(10);
  step(STP_Pin, (backlash * Modify_Ratio + 20), delayBetweenStep); //Backlash about 40 pulse
  delay(stableDelay * 2);

  switch (XYZ)
  {
  case 0:
    MotorCC_X = MotorCC;
    break;
  case 1:
    MotorCC_Y = MotorCC;
    break;
  case 2:
    MotorCC_Z = MotorCC;
    break;
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

long Curfit(double x1[], double y1[], int dataCount)
{
  Serial.println("Curfit Test");
  char buf[4];
  int xpower = 0;
  int order = 2;
  snprintf(buf, 4, "Fitting curve of order %i to data of power %i...\n", order, xpower);
  Serial.println(buf);

  double x[dataCount]; //idex * step = real steps
  double y[dataCount]; //fill this with your sensor data

  double center_x = x1[0];
  Serial.println(String(center_x));
  for (int i = 0; i < 3; i++)
  {
    x[i] = x1[i] - center_x;
    y[i] = y1[i];
    // Serial.println("X:" + String(x[i]));
  }

  int step_distance = abs(x[1] - x[0]);

  double coeffs[order + 1];

  int ret = fitCurve(order, sizeof(y) / sizeof(double), x, y, sizeof(coeffs) / sizeof(double), coeffs);

  if (ret == 0)
  { //Returned value is 0 if no error
    uint8_t c = 'a';
    // Serial.println("Coefficients are");

    // for (int i = 0; i < sizeof(coeffs) / sizeof(double); i++)
    // {
    //   snprintf(buf, 100, "%c=", c++);
    // Serial.print(buf);
    // Serial.print(coeffs[i]);
    // Serial.print('\t');
    // }
    // Serial.println("");

    long result_x = (-1 * coeffs[1]) / (2 * coeffs[0]);
    // Serial.println("Curfit X is : " + String(result_x));

    if (step_distance > abs(x[1] - result_x))
      return result_x + center_x;
    else
    {
      result_x = x1[1];
      // Serial.println("Final X is : " + String(result_x));
      return result_x;
    }
  }
  else
  {
    return x1[1];
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

double AutoAlign_Scan_DirectionJudge_V2(int axisDir, int count, int Threshold, int motorStep, int stableDelay,
                                        bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg)
{
  int DIR_Pin = 0;
  int STP_Pin = 0;
  MotorCC = Direction; // direction first
  int backlash = 40, trip = 1;
  bool isReverse = false;
  double ts = 0;
  unsigned long timer_1 = 0, timer_2 = 0;
  timer_1 = millis();
  isWiFiConnected = false;

  Serial.println("stableDelay: " + String(stableDelay));
  Serial.println("motorStep: " + String(motorStep));

  switch (axisDir)
  {
  case X_Dir:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = 40;
    delay(stableDelay);
    break;
  case Y_Dir:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = 80;
    delay(stableDelay);
    break;
  case Z_Dir:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = 40;
    delay(stableDelay);
    break;
  }

  CMDOutput(">>" + msg + String(trip)); //Trip_1------------------------------------------------------------

  double PD_Best = -64,
         PD_Trip2_Best = -64,
         PD_Now = -64;

  double PD_Value[count * 2];
  double PD_Rvrs_Value[count];
  int PD_Best_Pos = 0;
  int PD_Best_Pos_Trip2 = 0;

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points * 5);
  PD_initial = Cal_PD_Input_IL(Get_PD_Points * 5);
  DataOutput(axisDir, PD_initial); //int xyz, double pdValue

  if (PD_initial >= StopPDValue)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    PD_Best = PD_initial;

    return PD_Best;
  }

  Move_Motor(DIR_Pin, STP_Pin, MotorCC, motorStep * 4, delayBetweenStep, 100, true);

  // delay(150);

  PD_Now = Cal_PD_Input_IL(Get_PD_Points * 5);
  PD_Now = Cal_PD_Input_IL(Get_PD_Points * 5);
  DataOutput(axisDir, PD_Now); //int xyz, double pdValue

  Serial.println("Initial: " + String(PD_initial) + ", After:" + String(PD_Now));

  if (PD_Now >= StopPDValue)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    PD_Best = PD_Now;

    return PD_Best;
  }

  PD_Best_Pos = 0;

  for (int i = 0; i < count; i++)
  {
    PD_Value[i] = 0;
  }

  bool isFirstPointPassBest = false;
  bool Dir = true;

  if (PD_Now >= PD_initial)
  {
    Dir = true;
    PD_Best = PD_Now;
    digitalWrite(DIR_Pin, MotorCC);
    delay(10);
    Serial.println("MotorCC_Forward");

    PD_Value[0] = Cal_PD_Input_IL(Get_PD_Points);
  }
  else
  {
    Dir = false;
    PD_Best = PD_initial;
    BackLash_Reverse(axisDir, MotorCC, stableDelay);
    Serial.println("MotorCC_Reverse");
    isReverse = true;
  }

  //-------------------------------------------------Trip_2------------------------------------------------------------
  trip++;
  CMDOutput("~:" + msg + String(trip)); 

  int Plus_Times = 0;
  int trend_count = 0;

  long Pos_Ini_Trip2 = Get_Position(axisDir);

  if (!isFirstPointPassBest)
  {
    for (int i = 0; i < count; i++)
    {
      if (isStop)
        return true;

      if(i!=0)
      {
        step(STP_Pin, motorStep, delayBetweenStep);
        delay(stableDelay);
      }

      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);
      DataOutput(axisDir, PD_Value[i]); //int xyz, double pdValue

      if (i == 0)
      {
        PD_Trip2_Best = PD_Value[i];
        PD_Best_Pos_Trip2 = Get_Position(axisDir);
      }

      if (PD_Value[i] >= StopPDValue) //Condition 1
      {
        Serial.println("Stop Condition 1 : Over StopPDValue");

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        Serial.println("");

        PD_Best = PD_Value[i];
        return PD_Best;
      }

      if (PD_Value[i] > PD_Best)
      {
        PD_Best = PD_Value[i];
        PD_Best_Pos = i;
      }

      if (PD_Value[i] > PD_Trip2_Best)
      {
        PD_Trip2_Best = PD_Value[i];
        PD_Best_Pos_Trip2 = Get_Position(axisDir);
      }

      if(i>=2 && PD_Value[i] <= PD_Value[i - 1]) //Condition 2
      {
        trend_count ++;
        if(trend_count > 7)
        {
           Serial.println("Stop Condition 2-1 : Pass best IL");
           break;          
        }
        else if(Dir && trend_count > 3)
        {
          Serial.println("Stop Condition 2-2 : Pass best IL");
          break;
        }
        else if (trend_count >3 && i >= 12)
        {
          Serial.println("Stop Condition 2-3 : Pass best IL");
          break;
        }
      }
      
       if (i >= 6 
      && abs(PD_Value[i] - PD_Value[i - 1]) <=0.03 
      && abs(PD_Value[i-1] - PD_Value[i - 2]) <=0.03 
      && abs(PD_Value[i-2] - PD_Value[i - 3]) <=0.03 
      && abs(PD_Value[i-3] - PD_Value[i - 4]) <=0.03 
      ) //Condition 3 
      {
        if(PD_Value[i] < PD_Best){
          Serial.println("Stop Condition 3 : ERROR Status");
          break;
        }
      }     
     
      if (PD_Value[i] < -45) //Condition 5
      {
        Serial.println("Stop Condition 5 : Miss Target");
        break;
      }

      if (i == count - 1 && PD_Value[i] == PD_Best) //未通過最高點
        {
          if (Plus_Times < 5)
          {
            MSGOutput("Plus three points");
            count = count + 3;
            Plus_Times++;
          }
        }
    }
  }

  // ----------------------------------------------Trip 3 ------------------------------------------------------------

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  trip++;
  CMDOutput("~:" + msg + String(trip)); //Trip_3------------------------------------------------------------

  if(isTrip3Jump)
  {
    Move_Motor_abs(axisDir, PD_Best_Pos_Trip2); //Jump to Trip_2 start position
    delay(100);
    DataOutput(axisDir, Cal_PD_Input_IL(Get_PD_Points)); //int xyz, double pdValue
    MSGOutput("Jump to best position");
  }
  else
    if(PD_Now < PD_Best - 0.5)
    {
      if(PD_Best <= 3)
      {
        Move_Motor_abs(axisDir, Pos_Ini_Trip2); //Jump to Trip_2 start position
        delay(150);
        DataOutput(axisDir, Cal_PD_Input_IL(Get_PD_Points)); //int xyz, double pdValue
      }

      long pNow = PD_Best_Pos_Trip2;
      if(xyz == 0)
        pNow = X_Pos_Now;
      else if (xyz == 1)
        pNow = Y_Pos_Now;
      else if (xyz == 2)
        pNow = Z_Pos_Now;
      
      if(PD_Best_Pos_Trip2 != pNow)
      {
        Move_Motor_abs(axisDir, PD_Best_Pos_Trip2); //Jump to Trip_2 start position
        delay(100);
        DataOutput(axisDir, Cal_PD_Input_IL(Get_PD_Points)); //int xyz, double pdValue
        MSGOutput("Back to best position");
      }
    }

  delay(stableDelay);

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  if (abs(PD_Now - PD_Best) < 0.4)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    return PD_Best;
  }

  return PD_Best;

  timer_2 = millis();
  ts = (timer_2 - timer_1) * 0.001;
  Serial.print("TS:");
  Serial.println(ts, 2);
  Serial.println(" ");

  return PD_Best;
}

//------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------

int Function_Classification(String cmd, int ButtonSelected)
{
  if (cmd != "" && ButtonSelected < 0)
  {
    cmd.trim();
    MSGOutput("get_cmd:" + String(cmd));

    String cmdUpper = cmd;
    cmdUpper.toUpperCase();

//Keyboard - Motor Control
#pragma region - Keyboard - Motor Control
    if (cmd == "Xp1")
    {
      cmd_No = 102;
    }
    else if (cmd == "Xm1")
    {
      cmd_No = 105;
    }
    else if (cmd == "Yp1")
    {
      cmd_No = 104;
    }
    else if (cmd == "Ym1")
    {
      cmd_No = 106;
    }
    else if (cmd == "Zp1")
    {
      cmd_No = 103;
    }
    else if (cmd == "Zm1")
    {
      cmd_No = 101;
    }

    //Jog
    else if (Contains(cmd, "Jog_"))
    {
      cmd.remove(0, 4);

      byte dirPin, stpPin;
      bool dirt;

      if (Contains(cmd, "X"))
      {
        dirPin = X_DIR_Pin;
        stpPin = X_STP_Pin;
      }
      else if (Contains(cmd, "Y"))
      {
        dirPin = Y_DIR_Pin;
        stpPin = Y_STP_Pin;
      }
      else if (Contains(cmd, "Z"))
      {
        dirPin = Z_DIR_Pin;
        stpPin = Z_STP_Pin;
      }

      cmd.remove(0, 1);

      if (Contains(cmd, "m"))
      {
        dirt = false;
      }
      else if (Contains(cmd, "p"))
      {
        dirt = true;
      }

      cmd.remove(0, 2);

      Move_Motor(dirPin, stpPin, dirt, cmd.toDouble(), delayBetweenStep_Y, 0); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    }

    //Abs
    else if (Contains(cmd, "Abs_"))
    {
      cmd.remove(0, 4);

      byte dirPin, stpPin, xyz;
      bool dirt;

      if (Contains(cmd, "X"))
      {
        dirPin = X_DIR_Pin;
        stpPin = X_STP_Pin;
        xyz = 0;
      }
      else if (Contains(cmd, "Y"))
      {
        dirPin = Y_DIR_Pin;
        stpPin = Y_STP_Pin;
        xyz = 1;
      }
      else if (Contains(cmd, "Z"))
      {
        dirPin = Z_DIR_Pin;
        stpPin = Z_STP_Pin;
        xyz = 2;
      }

      cmd.remove(0, 2);

      Move_Motor_abs(xyz, cmd.toDouble());
    }

    //Abs All
    else if (Contains(cmd, "AbsAll_"))
    {
      cmd.remove(0, 7);

      int travel_x = 0, travel_y = 0, travel_z = 0;

      travel_x = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz

      cmd.remove(0, cmd.indexOf('_') + 1);

      travel_y = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz

      cmd.remove(0, cmd.indexOf('_') + 1);

      travel_z = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz

      Move_Motor_abs_all(travel_x, travel_y, travel_z);
    }

#pragma endregion

#pragma region - String Command
    //(CScan) Scan Twoway Command
    else if (Contains(cmd, "CScan_"))
    {
      cmd.remove(0, 6);
      Serial.println(cmd);

      int XYZ;
      int count;
      int motorStep;
      int stableDelay;
      bool Direction;
      int delayBetweenStep;
      int StopPDValue;
      int Get_PD_Points;
      int Trips;

      XYZ = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz
      cmd.remove(0, cmd.indexOf('_') + 1);

      count = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //count
      cmd.remove(0, cmd.indexOf('_') + 1);

      motorStep = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //step
      cmd.remove(0, cmd.indexOf('_') + 1);

      stableDelay = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stable delay
      cmd.remove(0, cmd.indexOf('_') + 1);

      Direction = cmd.substring(0, cmd.indexOf('_')) == "1";
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //direction
      cmd.remove(0, cmd.indexOf('_') + 1);

      delayBetweenStep = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //delaySteps
      cmd.remove(0, cmd.indexOf('_') + 1);

      StopPDValue = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stopValue
      cmd.remove(0, cmd.indexOf('_') + 1);

      Get_PD_Points = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //average points
      cmd.remove(0, cmd.indexOf('_') + 1);

      Trips = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd); //trips

      // int delayBetweenStep = 50;
      String msg = "Manual_Fine_Scan_Trip_";

      bool isOK = true;

      CMDOutput("AS");

      isOK = Scan_AllRange_TwoWay(XYZ, count, motorStep, stableDelay,
                                  Direction, delayBetweenStep, StopPDValue, Get_PD_Points, Trips, msg);

      CMDOutput("%:");

      if (!isOK)
      {
        CMDOutput("AS");
        Scan_AllRange_TwoWay(XYZ, count, motorStep, stableDelay,
                             Direction, delayBetweenStep, StopPDValue, Get_PD_Points, Trips, msg);
        CMDOutput("%:");
      }

      MSGOutput("Auto_Align_End");
    }

    //(SScan) Spiral Scan Command
    else if (Contains(cmd, "SScan_"))
    {
      cmd.remove(0, 6);
      Serial.println(cmd);

      int matrix;
      int motorStep;
      int stb;
      int delay_btw_steps;
      int StopPDValue;
      int Z_Layers;
      int Z_Steps;

      matrix = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //matrix
      cmd.remove(0, cmd.indexOf('_') + 1);

      motorStep = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //step
      cmd.remove(0, cmd.indexOf('_') + 1);

      stb = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stable delay
      cmd.remove(0, cmd.indexOf('_') + 1);

      //          delay_btw_steps = cmd.substring(0, cmd.indexOf('_')) == "1";
      delay_btw_steps = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //delay_btw_steps
      cmd.remove(0, cmd.indexOf('_') + 1);

      StopPDValue = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stopValue
      cmd.remove(0, cmd.indexOf('_') + 1);
      //          Serial.println(cmd);  //stopValue

      Z_Layers = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //Z_Layers
      cmd.remove(0, cmd.indexOf('_') + 1);

      Z_Steps = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd); //Z_Steps

      digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
      delay(5);

      M_Level = matrix;

      CMDOutput("AS");
      Serial.println("Auto-Align Start");
      CMDOutput("^X");
      CMDOutput("R:" + String(M_Level * 2 + 1));
      CMDOutput("C:" + String(M_Level * 2 + 1));
      // Serial.println("Rows=" + String(M_Level * 2 + 1));
      // Serial.println("Columns=" + String(M_Level * 2 + 1));
      // Serial.println("^X");

      MinMotroStep = motorStep  * MotorStepRatio; //350
      stableDelay = stb;
      delayBetweenStep = delay_btw_steps;

      if (Z_Layers > 1)
        sprial_JumpToBest = false;

      for (int ZL = 0; ZL < Z_Layers; ZL++)
      {
        if (ZL > 0)
        {
          Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, Z_Steps, 8, 150); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
        }

        AutoAlign_Spiral(M_Level, StopPDValue, stableDelay); //Input : (Sprial Level, Threshold, stable) Threshold:128
      }

      sprial_JumpToBest = true;

      CMDOutput("X^");
      // Serial.println("X^");

      digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
    }

    //Set auto-align / auto-curing Parameter
    else if (Contains(cmd, "Set::"))
    {
      cmd.remove(0, 5);

      String ParaName = cmd.substring(0, cmd.indexOf('='));
      cmd.remove(0, cmd.indexOf('=') + 1);
      cmd.trim();

      Serial.println("ParaName:" + ParaName + ", Value:" + String(cmd.toDouble()));

      if(!isNumberic(cmd))
      {
        Serial.println("cmd value is not a number!");
        return -1;
      }

      if (ParaName == "AA_SpiralRough_Feed_Steps_Z_A")
        AA_SpiralRough_Feed_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_SpiralRough_Spiral_Steps_XY_A")
        AA_SpiralRough_Spiral_Steps_XY_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Spiral_Steps_XY_A")
        AA_SpiralFine_Spiral_Steps_XY_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_A")
        AA_SpiralFine_Scan_Steps_X_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_B")
        AA_SpiralFine_Scan_Steps_X_B = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_C")
        AA_SpiralFine_Scan_Steps_X_C = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_D")
        AA_SpiralFine_Scan_Steps_X_D = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_A")
        AA_SpiralFine_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_B")
        AA_SpiralFine_Scan_Steps_Y_B = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_C")
        AA_SpiralFine_Scan_Steps_Y_C = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_D")
        AA_SpiralFine_Scan_Steps_Y_D = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_E")
        AA_SpiralFine_Scan_Steps_Y_E = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Feed_Steps_Z_A")
      {
        AA_ScanRough_Feed_Steps_Z_A = cmd.toInt();
        Serial.println("Write EEPROM AA_ScanRough_Feed_Steps_Z_A: " + WR_EEPROM(416, cmd));
      }
      else if (ParaName == "AA_ScanRough_Feed_Steps_Z_B")
      {
        AA_ScanRough_Feed_Steps_Z_B = cmd.toInt();
        Serial.println("Write EEPROM AA_ScanRough_Feed_Steps_Z_B: " + WR_EEPROM(424, cmd));
      }
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_A")
      {
        AA_ScanRough_Feed_Ratio_Z_A = cmd.toDouble();
        Serial.println("Write EEPROM AA_ScanRough_Feed_Ratio_Z_A: " + WR_EEPROM(432, cmd));
      }
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_B")
      {
        AA_ScanRough_Feed_Ratio_Z_B = cmd.toDouble();
        Serial.println("Write EEPROM AA_ScanRough_Feed_Ratio_Z_B: " + WR_EEPROM(440, cmd));
      }
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_C")
      {
        AA_ScanRough_Feed_Ratio_Z_C = cmd.toDouble();
        Serial.println("Write EEPROM AA_ScanRough_Feed_Ratio_Z_C: " + WR_EEPROM(448, cmd));
      }
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_D")
      {
        AA_ScanRough_Feed_Ratio_Z_D = cmd.toDouble();
        Serial.println("Write EEPROM AA_ScanRough_Feed_Ratio_Z_D: " + WR_EEPROM(456, cmd));
      }

      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_A")
        AA_ScanRough_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_B")
        AA_ScanRough_Scan_Steps_Y_B = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_C")
        AA_ScanRough_Scan_Steps_Y_C = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_D")
        AA_ScanRough_Scan_Steps_Y_D = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_A")
        AA_ScanRough_Scan_Steps_X_A = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_B")
        AA_ScanRough_Scan_Steps_X_B = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_C")
        AA_ScanRough_Scan_Steps_X_C = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_D")
        AA_ScanRough_Scan_Steps_X_D = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_E")
        AA_ScanRough_Scan_Steps_X_E = cmd.toInt();
      else if (ParaName == "AA_ScanFine_Scan_Steps_Z_A")
        AA_ScanFine_Scan_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_ScanFine_Scan_Steps_Y_A")
        AA_ScanFine_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_ScanFine_Scan_Steps_X_A")
        AA_ScanFine_Scan_Steps_X_A = cmd.toInt();
      else if (ParaName == "AA_ScanFinal_Scan_Steps_Z_A")
        AA_ScanFinal_Scan_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_ScanFinal_Scan_Steps_Y_A")
        AA_ScanFinal_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_ScanFinal_Scan_Steps_X_A")
        AA_ScanFinal_Scan_Steps_X_A = cmd.toInt();

      else if (ParaName == "AQ_Scan_Compensation_Steps_Z_A")
      {
        AQ_Scan_Compensation_Steps_Z_A = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Compensation_Steps_Z_A: " + WR_EEPROM(160, cmd));
      }
      else if (ParaName == "AQ_Total_TimeSpan")
      {
        AQ_Total_TimeSpan = cmd.toInt();
        Serial.println("Write EEPROM AQ_Total_TimeSpan: " + WR_EEPROM(168, cmd));
      }

      else if (ParaName == "AA_ScanFinal_Scan_Delay_X_A")
      {
        AA_ScanFinal_Scan_Delay_X_A = cmd.toInt();
        Serial.println("Write EEPROM AA_ScanFinal_Scan_Delay_X_A: " + WR_EEPROM(80, cmd));
      }

      else if (ParaName == "AQ_Scan_Steps_Z_A")
      {
        AQ_Scan_Steps_Z_A = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_A: " + WR_EEPROM(176, cmd));
      }
      else if (ParaName == "AQ_Scan_Steps_Z_B")
      {
        AQ_Scan_Steps_Z_B = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_B: " + WR_EEPROM(184, cmd));
      }
      else if (ParaName == "AQ_Scan_Steps_Z_C")
      {
        AQ_Scan_Steps_Z_C = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_C: " + WR_EEPROM(192, cmd));
      }
      else if (ParaName == "AQ_Scan_Steps_Z_D")
      {
        AQ_Scan_Steps_Z_D = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_D: " + WR_EEPROM(200, cmd));
      }

      else if (ParaName == "FS_Count_X")
      {
        FS_Count_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Count_X: " + WR_EEPROM(EP_FS_Count_X, cmd));
      }
      else if (ParaName == "FS_Steps_X")
      {
        FS_Steps_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Steps_X: " + WR_EEPROM(EP_FS_Steps_X, cmd));
      }
      else if (ParaName == "FS_Stable_X")
      {
        FS_Stable_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Stable_X: " + WR_EEPROM(EP_FS_Stable_X, cmd));
      }
      else if (ParaName == "FS_DelaySteps_X")
      {
        FS_DelaySteps_X = cmd.toInt();
        Serial.println("Write EEPROM FS_DelaySteps_X: " + WR_EEPROM(EP_FS_DelaySteps_X, cmd));
      }
      else if (ParaName == "FS_Avg_X")
      {
        FS_Avg_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Avg_X: " + WR_EEPROM(EP_FS_Avg_X, cmd));
      }

      else if (ParaName == "FS_Count_Y")
      {
        FS_Count_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Count_Y: " + WR_EEPROM(EP_FS_Count_Y, cmd));
      }
      else if (ParaName == "FS_Steps_Y")
      {
        FS_Steps_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Steps_Y: " + WR_EEPROM(EP_FS_Steps_Y, cmd));
      }
      else if (ParaName == "FS_Stable_Y")
      {
        FS_Stable_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Stable_Y: " + WR_EEPROM(EP_FS_Stable_Y, cmd));
      }
      else if (ParaName == "FS_DelaySteps_Y")
      {
        FS_DelaySteps_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_DelaySteps_Y: " + WR_EEPROM(EP_FS_DelaySteps_Y, cmd));
      }
      else if (ParaName == "FS_Avg_Y")
      {
        FS_Avg_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Avg_Y: " + WR_EEPROM(EP_FS_Avg_Y, cmd));
      }
      else if (ParaName == "FS_Count_Z")
      {
        FS_Count_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Count_Z: " + WR_EEPROM(EP_FS_Count_Z, cmd));
      }
      else if (ParaName == "FS_Steps_Z")
      {
        FS_Steps_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Steps_Z: " + WR_EEPROM(EP_FS_Steps_Z, cmd));
      }
      else if (ParaName == "FS_Stable_Z")
      {
        FS_Stable_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Stable_Z: " + WR_EEPROM(EP_FS_Stable_Z, cmd));
      }
      else if (ParaName == "FS_DelaySteps_Z")
      {
        FS_DelaySteps_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_DelaySteps_Z: " + WR_EEPROM(EP_FS_DelaySteps_Z, cmd));
      }
      else if (ParaName == "FS_Avg_Z")
      {
        FS_Avg_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Avg_Z: " + WR_EEPROM(EP_FS_Avg_Z, cmd));
      }
      else if (ParaName == "FS_Trips_X")
      {
        FS_Trips_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Trips_X: " + WR_EEPROM(EP_FS_Trips_X, cmd));
      }
      else if (ParaName == "FS_Trips_Y")
      {
        FS_Trips_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Trips_Y: " + WR_EEPROM(EP_FS_Trips_Y, cmd));
      }
      else if (ParaName == "FS_Trips_Z")
      {
        FS_Trips_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Trips_Z: " + WR_EEPROM(EP_FS_Trips_Z, cmd));
      }
    }

    //Set BackLash Command
    else if (Contains(cmd, "_BL:"))
    {
      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 4);

        X_backlash = cmd.toInt();

        CleanEEPROM(24, 8); //Clean EEPROM(int startPosition, int datalength)

        WriteInfoEEPROM(String(cmd), 24); //(data, start_position)  // Write Data to EEPROM

        EEPROM.commit();

        Serial.println("Set X BackLash: " + String(String(cmd)));

        // Reading Data from EEPROM
        Serial.println("X BackLash in eeprom: " + ReadInfoEEPROM(24, 8)); //(start_position, data_length)

        X_backlash = ReadInfoEEPROM(24, 8).toInt();
      }

      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 5);

        Y_backlash = cmd.toInt();

        CleanEEPROM(32, 8); //Clean EEPROM(int startPosition, int datalength)

        WriteInfoEEPROM(String(cmd), 32); //(data, start_position)  // Write Data to EEPROM

        EEPROM.commit();

        Serial.println("Set Y BackLash: " + String(String(cmd)));

        Serial.println("Y BackLash in eeprom: " + ReadInfoEEPROM(32, 8)); //(start_position, data_length)

        Y_backlash = ReadInfoEEPROM(32, 8).toInt();
      }

      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 5);

        Z_backlash = cmd.toInt();

        CleanEEPROM(40, 8); //Clean EEPROM(int startPosition, int datalength)

        WriteInfoEEPROM(String(cmd), 40); //(data, start_position)  // Write Data to EEPROM

        EEPROM.commit();

        Serial.println("Set Z BackLash: " + String(String(cmd)));

        Serial.println("Z BackLash in eeprom: " + ReadInfoEEPROM(40, 8)); //(start_position, data_length)

        Z_backlash = ReadInfoEEPROM(40, 8).toInt();
      }
    }

    //Set Scan Steps Command
    else if (Contains(cmd, "_ScanSTP:"))
    {
      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 9);
        cmd.trim();

        X_ScanSTP = cmd.toInt();

        CleanEEPROM(48, 8);                                         //Clean EEPROM(int startPosition, int datalength)
        WriteInfoEEPROM(String(cmd), 48);                           //(data, start_position)  // Write Data to EEPROM
        Serial.println("Save in eeprom: " + ReadInfoEEPROM(48, 8)); //(start_position, data_length)

        Serial.println("Set X Scan Step: " + String(X_ScanSTP));
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 10);
        Y_ScanSTP = cmd.toInt();

        CleanEEPROM(56, 8);                                         //Clean EEPROM(int startPosition, int datalength)
        WriteInfoEEPROM(String(cmd), 56);                           //(data, start_position)  // Write Data to EEPROM
        Serial.println("Save in eeprom: " + ReadInfoEEPROM(56, 8)); //(start_position, data_length)

        Serial.println("Set Y Scan Step: " + String(String(cmd)));
      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 10);
        Z_ScanSTP = cmd.toInt();

        CleanEEPROM(64, 8);                                         //Clean EEPROM(int startPosition, int datalength)
        WriteInfoEEPROM(String(cmd), 64);                           //(data, start_position)  // Write Data to EEPROM
        Serial.println("Save in eeprom: " + ReadInfoEEPROM(64, 8)); //(start_position, data_length)

        Serial.println("Set Z Scan Step: " + String(String(cmd)));
      }
    }

    //Get Ref Command
    else if (cmd == "REF?")
    {
      String eepromString = ReadInfoEEPROM(0, 8); //(int start_position, int data_length)

      Serial.println("Get_Ref:" + eepromString); //(start_position, data_length)  // Reading Data from EEPROM

      eepromString.trim();
      ref_Dac = eepromString.toDouble();
      ref_IL = ILConverter(ref_Dac);

      Serial.println("Dac:" + eepromString);  //(start_position, data_length)  // Reading Data from EEPROM
      Serial.println("IL:" + String(ref_IL)); //(start_position, data_length)  // Reading Data from EEPROM
    }

    //Set Ref Command
    else if (Contains(cmd, "Set_Ref:"))
    {
      cmd.remove(0, 8);
      cmd.trim();

      CleanEEPROM(0, 8);               //Clean EEPROM(int startPosition, int datalength)
      WriteInfoEEPROM(String(cmd), 0); //(data, start_position)  // Write Data to EEPROM
      EEPROM.commit();

      Serial.println("Update Ref Value : " + String(cmd));

      String eepromString = ReadInfoEEPROM(0, 8); //(int start_position, int data_length)

      Serial.println("PD ref: " + eepromString); //(start_position, data_length)  // Reading Data from EEPROM

      ref_Dac = eepromString.toDouble();
      ref_IL = ILConverter(ref_Dac);
    }

    //Set Target IL
    else if (Contains(cmd, "Set_Target_IL:"))
    {
      cmd.remove(0, 14);
      cmd.trim();
      Target_IL = WR_EEPROM(72, cmd).toDouble();
      MSGOutput("Set_Target_IL:" + String(Target_IL));
    }

    //Set Motor Step Ratio
    else if (Contains(cmd, "Set_MotorStepRatio:"))
    {
      cmd.remove(0, 19);
      cmd.trim();
      MotorStepRatio = WR_EEPROM(208, cmd).toDouble();
      MSGOutput("Set_MotorStepRatio:" + String(MotorStepRatio));
    }

     //Set Motor Step Delay Ratio
    else if (Contains(cmd, "Set_MotorStepDelayRatio:"))
    {
      cmd.remove(0, 24);
      cmd.trim();
      MotorStepDelayRatio = WR_EEPROM(216, cmd).toDouble();
      MSGOutput("Set_MotorStepDelayRatio:" + String(MotorStepDelayRatio));
    }

    //Get Motor position now
    else if (Contains(cmd, "POS?"))
    {
      Serial.println("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now));
    }

    // //Set Manual Control Motor Speed
    else if (Contains(cmd, "SPD"))
    {
      cmd.remove(0, 3);  

      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 3);  //Include empty char deleted
        delayBetweenStep_X = cmd.toInt();
        WR_EEPROM(48, cmd);
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 3);  //Include empty char deleted
        delayBetweenStep_Y = cmd.toInt();
        WR_EEPROM(56, cmd);

      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 3);  //Include empty char deleted
        delayBetweenStep_Z = cmd.toInt();
        WR_EEPROM(64, cmd);
      }
      else if (Contains(cmd, "?"))
      {
        Serial.println("Motor Speed (x, y, z): (" 
          + ReadInfoEEPROM(48, 8)
          + ","
          + ReadInfoEEPROM(56, 8)
          + ","
          + ReadInfoEEPROM(64, 8)
          + ")"
          );
      }
      else
      {
        int dbt = cmd.toInt();
        delayBetweenStep_Y = dbt;
        delayBetweenStep_Z = dbt;
        delayBetweenStep_Z = dbt;
        WR_EEPROM(48, cmd);
        WR_EEPROM(56, cmd);
        WR_EEPROM(64, cmd);
      }

      Serial.println("Set Motor Speed:" + cmd);
    }

    //Set Manual Control Motor Speed
    else if (Contains(cmd, "Set_Motor_Speed_"))
    {
      cmd.remove(0, 16);

      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 2);
        cmd.trim();
        delayBetweenStep_X = cmd.toInt();
        WR_EEPROM(48, cmd);
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 2);
        cmd.trim();
        delayBetweenStep_Y = cmd.toInt();
        WR_EEPROM(56, cmd);

      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 2);
        cmd.trim();
        delayBetweenStep_Z = cmd.toInt();
        WR_EEPROM(64, cmd);
      }

      Serial.println("Set Manual Control Motor Speed:" + cmd);
    }

    //Move motor to abs position sync
    // else if (Contains(cmd, "GSP "))
    // {
    //   cmd.remove(0, 4);
    //   if(cmd == "1")
    //   {
    //     Move_Motor_abs_all_sync(Pos_1_Record.X_Pos, Pos_1_Record.Y_Pos, Pos_1_Record.Z_Pos);
    //   }
    //   else if (cmd =="2")
    //   {
    //     Move_Motor_abs_all_sync(Pos_2_Record.X_Pos, Pos_2_Record.Y_Pos, Pos_2_Record.Z_Pos);
    //   }
    //   else if (cmd =="3")
    //   {
    //     Move_Motor_abs_all_sync(Pos_3_Record.X_Pos, Pos_3_Record.Y_Pos, Pos_3_Record.Z_Pos);
    //   }
    //   else if (cmd =="4")
    //   {
    //     Move_Motor_abs_all_sync(Pos_4_Record.X_Pos, Pos_4_Record.Y_Pos, Pos_4_Record.Z_Pos);
    //   }
    // }

    //Get/Set PD_Ref_Array[15][0]=
    else if (Contains(cmd, "PD_Ref_Array"))
    {
      cmd.remove(0, 12);
      cmd.trim();
      
      if (Contains(cmd, "]"))
      {
        int idx_start = cmd.indexOf('[') + 1;
        int idx_end = cmd.indexOf(']');
        int idx_1 = cmd.substring(idx_start, idx_end).toInt();
        // Serial.println("Substring:" + cmd.substring(idx_start, idx_end));
        idx_start = idx_end + 2;
        idx_end = cmd.indexOf(']', idx_start);
        int idx_2 = cmd.substring(idx_start, idx_end).toInt();
        // Serial.println("Substring:" + cmd.substring(idx_start, idx_end));

        if (Contains(cmd, "?"))
        {          
          if(idx_2 < 2)
          {
            int value = PD_Ref_Array[idx_1][idx_2];
            Serial.printf("idx1:%d, idx2:%d, value:%d\n", idx_1, idx_2, value);
          }         
        }
        else
        {
          idx_start = cmd.indexOf('=', idx_end) + 1;
          if (isNumberic(cmd.substring(idx_start)))
          {
            int value = cmd.substring(idx_start).toInt();
            Serial.printf("idx1:%d, idx2:%d, value:%d\n", idx_1, idx_2, value);

            PD_Ref_Array[idx_1][idx_2] = value;
            Serial.println("Set PD_Ref_Array: " + WR_EEPROM(EP_PD_Ref_Array[idx_1][idx_2], String(value)));
          }
        }
      }
      else if(Contains(cmd, "?") && !Contains(cmd, "]"))
      {
        String eepromString;
        int iniEP = 700;
        for (size_t i = 0; i < sizeof(PD_Ref_Array) / sizeof(PD_Ref_Array[0]); i++)
        {                   
          Serial.printf("[%d, %d]:[%d, %d]\n",  EP_PD_Ref_Array[i][0],  EP_PD_Ref_Array[i][1], PD_Ref_Array[i][0], PD_Ref_Array[i][1]);
        }
      }
    }


    //Set PD average points
    else if (Contains(cmd, "Set_PD_average_Points:"))
    {
      cmd.remove(0, 22);
      cmd.trim();
      Get_PD_Points = cmd.toInt();

      Serial.println("Set Get_PD_Points: " + WR_EEPROM(152, cmd));
    }

    //Set Board ID
    else if (Contains(cmd, "ID#"))
    {
      cmd.remove(0, 3);
      Serial.println("Set Board ID: " + WR_EEPROM(8, cmd));
    }

    //Get Board ID
    else if (cmd == "ID?")
    {
      Serial.println(ReadInfoEEPROM(8, 8));
    }

    //Set Station ID
    else if (Contains(cmd, "ID_Station#"))
    {
      cmd.remove(0, 11);
      Station_ID = cmd;
      Serial.println("Set Station ID: " + WR_EEPROM(16, cmd));
    }

    //Get Station ID
    else if (cmd == "ID_Station?")
    {
      Serial.println(ReadInfoEEPROM(16, 8));
    }

    //Set Server ID
    else if (Contains(cmd, "ID_Server#"))
    {
      cmd.remove(0, 10);
      Serial.println("Set Server ID: " + WR_EEPROM(88, 32, cmd));
    }

    //Get Server ID
    else if (cmd == "ID_Server?")
    {
      MSGOutput(ReadInfoEEPROM(88, 32));
    }

    //Set Server Password
    else if (Contains(cmd, "PW_Server#"))
    {
      cmd.remove(0, 10);
      Serial.println("Set Server Password: " + WR_EEPROM(120, 32, cmd));
    }

    //Get Server Password
    else if (cmd == "PW_Server?")
    {
      Serial.println(ReadInfoEEPROM(120, 32));
    }

    //Clena EEPROM : Start position (default length = 8)
    else if (Contains(cmd, "Clean_EEPROM:"))
    {
      cmd.remove(0, 13);
      CleanEEPROM(cmd.toInt(), 8);
      WR_EEPROM(cmd.toInt(), "");
      EEPROM.commit();
      Serial.println("Clean_EEPROM:" + cmd);
      // Serial.println(ReadInfoEEPROM(cmd.toInt(), 8));
    }

    //Get UI_Data cmd and return value to controller
    else if (Contains(cmd, "UI?"))
    {
      DataSent_Controller(cmd);
      // sendmsg_UI_Data.msg = "Core:" + cmd;
      // sendmsg_UI_Data._Target_IL = Target_IL;
      // sendmsg_UI_Data._ref_Dac = ref_Dac;
      // sendmsg_UI_Data._Q_Z_offset = AQ_Scan_Compensation_Steps_Z_A;
      // sendmsg_UI_Data._speed_x = delayBetweenStep_X;
      // sendmsg_UI_Data._speed_y = delayBetweenStep_Y;
      // sendmsg_UI_Data._speed_z = delayBetweenStep_Z;
      // // Serial.println("Target_IL:" + String(Target_IL));

      // esp_err_t result = esp_now_send(ControllerAddress, (uint8_t *) &sendmsg_UI_Data, sizeof(sendmsg_UI_Data));
      cmd = "";
      cmd_from_contr = "";
      cmd_value_from_contr = "";
    }

    else if (Contains(cmd, "MAC:Core?"))
    {
       #pragma region Show string of Server address

        String addr = "";
        // Serial.print("Core Address:  ");
        for (size_t i = 0; i < sizeof(ThisAddress); i++)
        {
          String adr = String(ThisAddress[i], HEX);
          adr.toUpperCase();
          addr += adr;
          if(i < sizeof(ThisAddress) - 1)
            addr += ":";  
        }

        #pragma endregion 

      sendmsg_UI_Data.msg = "MAC:Core";
      addr.toCharArray(sendmsg_UI_Data.para, 30);
      Serial.println("MAC Core:" + addr);

      esp_err_t result = esp_now_send(ThisAddress, (uint8_t *) &sendmsg_UI_Data, sizeof(sendmsg_UI_Data));
      cmd = "";
      cmd_from_contr = "";
      cmd_value_from_contr = "";
    }

    else if (Contains(cmd, "MAC:Server?"))
    {
       #pragma region Show string of Server address

        String addr = "";
        for (size_t i = 0; i < sizeof(ServerAddress); i++)
        {
          String adr = String(ServerAddress[i], HEX);
          adr.toUpperCase();
          addr += adr;
          if(i < sizeof(ServerAddress) - 1)
            addr += ":";  
        }

        #pragma endregion 

      sendmsg_UI_Data.msg = "MAC:Server";
      addr.toCharArray(sendmsg_UI_Data.para, 30);
      Serial.println("MAC Server:" + ThisAddr);

      esp_err_t result = esp_now_send(ControllerAddress, (uint8_t *) &sendmsg_UI_Data, sizeof(sendmsg_UI_Data));
      cmd = "";
      cmd_from_contr = "";
      cmd_value_from_contr = "";
    }
    
    //Set UI_Data from controller
    else if (Contains(cmd, "UI_Data_"))
    {
      String s = "UI_Data_";
      cmd.remove(0, s.length());
      cmd.trim();
      // Serial.println("UI_Data_:" + cmd);

      if(isNumberic(cmd_value_from_contr))
      {     
        if(Contains(cmd, "Target_IL"))
          Target_IL = WR_EEPROM(72, cmd_value_from_contr).toDouble();
        else if(Contains(cmd, "Z_offset"))
          AQ_Scan_Compensation_Steps_Z_A = WR_EEPROM(160, cmd_value_from_contr).toInt();        
        else if(Contains(cmd, "speed_x"))
          delayBetweenStep_X = WR_EEPROM(48, cmd_value_from_contr).toInt();       
        else if(Contains(cmd, "speed_y"))
          delayBetweenStep_Y = WR_EEPROM(56, cmd_value_from_contr).toInt();     
        else if(Contains(cmd, "speed_z"))
          delayBetweenStep_Z = WR_EEPROM(64, cmd_value_from_contr).toInt();    
        else if(Contains(cmd, "Ref"))
        {
          if (true)
          {
            digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
            delay(5);

            averagePDInput = 0;
            for (int i = 0; i < 20; i++)
              averagePDInput += ads.readADC_SingleEnded(0);

            averagePDInput = (averagePDInput / 20);

            ref_Dac = averagePDInput;
            ref_IL = ILConverter(averagePDInput);
                      
            MSGOutput("EEPROM(" + String(0) + ") - " + WR_EEPROM(0, String(ref_Dac))); // For update HMI ref value
            Serial.println("Ref_Dac: " + String(ref_Dac) + ", Ref_IL: " + String(ref_IL)); //Reading Data from EEPROM(start_position, data_length)

            digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
            delay(3);

            // sendmsg_UI_Data.msg = "Core:" + cmd;
            // sendmsg_UI_Data._Target_IL = Target_IL;
            // sendmsg_UI_Data._ref_Dac = ref_Dac;
            // sendmsg_UI_Data._Q_Z_offset = AQ_Scan_Compensation_Steps_Z_A;
            // sendmsg_UI_Data._speed_x = delayBetweenStep_X;
            // sendmsg_UI_Data._speed_y = delayBetweenStep_Y;
            // sendmsg_UI_Data._speed_z = delayBetweenStep_Z;
            // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendmsg_UI_Data, sizeof(sendmsg_UI_Data));
          }
        }

        Serial.println(cmd + " : " + cmd_value_from_contr);
      }
     
      cmd = "";
      cmd_from_contr = "";
      cmd_value_from_contr = "";
    }

    #pragma endregion

    //Command No.
    else if (Contains(cmd, "cmd"))
    {
      cmd.remove(0, 3);
      cmd_No = cmd.toInt();
      delay(10);

      //          cmd_No = 4;  //Auto-Align
      //          cmd_No = 5;   //Fine scan
      //          cmd_No = 6;   //Auto curing
      //          cmd_No = 7;  //To re-load position
      //          cmd_No = 8;  //To Home
      //          cmd_No = 9;  //To Home
      //          cmd_No = 10;  //To Home
      //          cmd_No = 16;  //Set reLoad
      //          cmd_No = 17;  //Set home
      //          cmd_No = 18;  //Set Z target
      //          cmd_No = 19;  //Get ref
      //          cmd_No = 20;  //Spiral
      //          cmd_No = 21;  //Keep print IL to PC
      //          cmd_No = 22;  //Scan X
      //          cmd_No = 23;  //Scan Y
      //          cmd_No = 24;  //Scan Z
    }

    //Action : Reply
    if (true)
    {
    }
  }
  else if (ButtonSelected >= 0)
  {
    //Keyboard No. to Cmd Set No.
    switch (ButtonSelected)
    {
    case 7:
      cmd_No = 1;
      break;

    case 8:
      cmd_No = 2;
      break;

    case 9:
      cmd_No = 3;
      break;

    default:
      cmd_No = ButtonSelected;
      break;
    }
  }

  return cmd_No;
}

//------------------------------------------------------------------------------------------------------------------------------------------

int Function_Excecutation(String cmd, int cmd_No)
{
  //Function Execution
  // String cmd = "";

  if (cmd_No != 0)
  {
    Serial.println("Btn:" + String(ButtonSelected) + ", CMD:" + String(cmd_No));

    //Functions: Alignment
    if (cmd_No <= 100)
    {
      switch (cmd_No)
      {
        //Functions: Auto Align
      case 1: /* Auto Align */
        if (true)
        {
          DataSent_Controller("AA");

          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          AQ_Scan_Compensation_Steps_Z_A = 0;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(3);

          AutoAlign();

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
          Serial.println("Auto Align End");
          MotorCC = true;

          StopValue = 0; //0 dB

          AQ_Scan_Compensation_Steps_Z_A = ReadInfoEEPROM(160, 8).toInt();
          Serial.println("AQ_Scan_Compensation_Steps_Z_A: " + String(AQ_Scan_Compensation_Steps_Z_A));

          DataSent_Controller("Menu");

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

        //Functions: Fine Scan
      case 2: /* Fine Scan */
        if (!btn_isTrigger)
        {
          DataSent_Controller("FS");
          StopValue = 0; //0 dB

          bool K_OK = true;
          bool initial_wifi_isConnected = isWiFiConnected;

          AQ_Scan_Compensation_Steps_Z_A = 0;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode

          //------------------------------------------------------------------------------------------------------------------------------Q Scan Z

          Fine_Scan(Z_Dir, false); 
          Serial.println("Fine_Scan 3 End");
          if (isStop) true;

          //------------------------------------------------------------------------------------------------------------------------------Q Scan Y

          Fine_Scan(Y_Dir, false); 

          if (isStop) true;

          //------------------------------------------------------------------------------------------------------------------------------Q Scan X

          Fine_Scan(X_Dir, false); 

          if (isStop) true;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

          MSGOutput("Auto Align End");

          DataSent_Controller("Menu");

          AQ_Scan_Compensation_Steps_Z_A = ReadInfoEEPROM(160, 8).toInt();

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      //Functions: Auto Curing
     case 3: /* Auto Curing */
        if (!btn_isTrigger)
        {
          btn_isTrigger = false;

          isILStable = false;
          bool isStopAlign = false;
          is_AutoCuring = true;

          ButtonSelected = -1;

          double IL_stable_count = 0;
          double Acceptable_Delta_IL = 0.3; //12,10, 2
          unsigned long lastUpdateQT = 0;
          Q_Time = 0;

          time_curing_0 = millis();
          time_curing_1 = time_curing_0;
          time_curing_2 = time_curing_1;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(155);

          AutoCuring_Best_IL = Cal_PD_Input_IL(Get_PD_Points * 3);

          StopValue = AutoCuring_Best_IL;

          double temp_targetIL = Target_IL;
          Target_IL = AutoCuring_Best_IL;  //Send AQ target IL to controller UI

          DataSent_Controller("AQ");

          if(StopValue > -0.9)
            StopValue = -0.9;          

          Z_ScanSTP = AQ_Scan_Steps_Z_A; //125 (AQ_Scan_Steps_Z_A)
          MSGOutput("Auto-Curing");
          CMDOutput("AQ");                             // Auto_Curing Start
          CMDOutput("QT" + String(AQ_Total_TimeSpan)); // Auto_Curing Start
          MSGOutput("StopValue : " + String(StopValue));

          while (true)
          {
            if(digitalRead(Tablet_PD_mode_Trigger_Pin))
            {
              digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
              delay(2);
            }

            PD_Now = Cal_PD_Input_IL(Get_PD_Points);
            Q_Time = ((millis() - time_curing_0) / 1000);
            
            if(isStopAlign)
            { 
              if(Q_Time - lastUpdateQT >= 3) //Reduce get data rate after stop auto-aligning
              {
                lastUpdateQT = Q_Time;
                MSGOutput("QT_IL:" + String(Q_Time)+ " s, " + String(PD_Now));
              } 
            }
            else
              MSGOutput("QT_IL:" + String(Q_Time)+ " s, " + String(PD_Now));

            String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
            DataSent_Controller("AQ");

            if (Serial.available())
              cmd = Serial.readString();

            cmd_No = Function_Classification(cmd, ButtonSelected);
            if(cmd_No == 30) 
              EmergencyStop();

            cmd = ""; //Reset command from serial port

            isLCD = true;

            delay(998); //get data time rate
          
            if (isStop)
              break;

            //Q State
            if (true)
            {
              if (Q_Time <= 540)  // 540
              {
                Q_State = 1;
              }
              else if (Q_Time > 540 && Q_Time <= 600)  //540, 600
              {
                Q_State = 2;
              }
              else if (Q_Time > 600 && Q_Time <= 700)  //600, 700
              {
                Q_State = 3;
              }
              else if (Q_Time > 700)  //700
              {
                Q_State = 4;
              }
            }

            //Q Stop Conditions
            if (true)
            {
              //IL Stable Time ,  70 secs,  curing time threshold , 12.5 mins
              if (time_curing_2 - time_curing_1 > 70000 && Q_Time >= 860 && !isStopAlign) // 800
              {
                MSGOutput("Update : IL Stable - Stop Auto Curing");
                isStopAlign = true;
              }
              //Total curing time , 14 mins, 840s
              else if (Q_Time >= AQ_Total_TimeSpan - 1)
              {
                MSGOutput("Over Limit Curing Time - Stop Auto Curing");
                isStop = true;
                break;
              }
            }

            if (isStop)
              break;

            //Q scan conditions
            if (true)
            {
              if (Q_State == 2)
              {
                if (Z_ScanSTP > AQ_Scan_Steps_Z_B)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_B; //125 - > 35 (AQ_Scan_Steps_Z_B)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }
              else if (Q_State == 3)
              {
                if (Z_ScanSTP > AQ_Scan_Steps_Z_C)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_C; //70 (AQ_Scan_Steps_Z_C)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }
              else if (Q_State == 4)
              {
                if (Z_ScanSTP > AQ_Scan_Steps_Z_D)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_D; //50 (AQ_Scan_Steps_Z_D)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }

                if (Q_Time > 420)
              {
                if(Acceptable_Delta_IL > 0.2 )
                {
                  if(StopValue > -1.3)
                    Acceptable_Delta_IL = 0.2; // Target IL changed 0.25
                  else
                    Acceptable_Delta_IL = 0.12; // Target IL changed 0.25

                  MSGOutput("Update Scan Condition: " + String(Acceptable_Delta_IL));     
                }
              }
            }

            PD_Now = Cal_PD_Input_IL(Get_PD_Points * 3);  //Increase IL stability

            bool forcedAlign = false;
            if(cmd_value_from_contr == "8") forcedAlign = true;

            if (PD_Now >= (AutoCuring_Best_IL - (Acceptable_Delta_IL)) && cmd_value_from_contr != "8")
            {
              time_curing_2 = millis();
              continue;
            }
            else  //Start Align
            {
              cmd_from_contr = "";
              cmd_value_from_contr = "";
              if(!isStopAlign || forcedAlign)
              {
                //Q Scan
                if (true && Q_Time <= 900 && !isStopAlign || forcedAlign)
                {
                  PD_Now = Cal_PD_Input_IL(Get_PD_Points * 3);  //Increase IL stability

                  // Into Q Scan X
                  # pragma region Q Scan X
                  if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL) || forcedAlign)
                  {
                    DataSent_Controller("Scan");

                    Fine_Scan(X_Dir, false); //--------------------------------------------------------Q Scan X

                    Q_Time = ((millis() - time_curing_0) / 1000);
                    MSGOutput("QT_IL:" + String(Q_Time)+ " s, " + String(PD_Now));
                    String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                    DataSent_Controller("AQ");

                    if (PD_Now - Cal_PD_Input_IL(Get_PD_Points) > 1)
                    {
                      DataSent_Controller("Scan");

                      Fine_Scan(X_Dir, false); //------------------------------------------------------Q Scan X

                      Q_Time = ((millis() - time_curing_0) / 1000);
                      MSGOutput("QT_IL:" + String(Q_Time)+ " s, " + String(PD_Now));
                      String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                      DataSent_Controller("AQ");
                    }

                    if (Q_State >= 4 && (maxIL_in_FineScan - minIL_in_FineScan)<=0.25 && Q_Time>=820 && !isStopAlign){
                      MSGOutput("Update : Delta IL < 0.25, break"); //break curing loop
                      isStopAlign = true;
                    }
                  }
                  #pragma endregion

                  if (isStop) break;

                  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
                  delay(5);

                  PD_Now = Cal_PD_Input_IL(Get_PD_Points * 3);  //Increase IL stability
                  MSGOutput("Q_State: " + String(Q_State));

                  // Into Q Scan Y
                  # pragma region Q Scan Y
                  if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL) || Q_State == 1 || forcedAlign)
                  {
                    DataSent_Controller("Scan");

                    Fine_Scan(Y_Dir, false); //--------------------------------------------------------Q Scan Y

                    Q_Time = ((millis() - time_curing_0) / 1000);
                    MSGOutput("QT_IL:" + String(Q_Time)+ " s, " + String(PD_Now));
                    String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                    DataSent_Controller("AQ");

                    if (PD_Now - Cal_PD_Input_IL(Get_PD_Points) > 1)
                    {
                      DataSent_Controller("Scan");

                      Fine_Scan(Y_Dir, false); //------------------------------------------------------Q Scan Y

                      Q_Time = ((millis() - time_curing_0) / 1000);
                      MSGOutput("QT_IL:" + String(Q_Time)+ " s, " + String(PD_Now));
                      String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                      DataSent_Controller("AQ");
                    }

                    if (Q_State >= 4 && (maxIL_in_FineScan - minIL_in_FineScan)<=0.25 && Q_Time>=820 && !isStopAlign){
                      MSGOutput("Update : Delta IL < 0.25, break"); //break curing loop
                      isStopAlign = true;
                    }
                  }
                  #pragma endregion

                  if (isStop) break;

                  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
                  delay(5);

                  PD_Before = Cal_PD_Input_IL(Get_PD_Points);

                  bool K_OK = true;

                  //Into Q Scan Z
                   # pragma region Q Scan Z
                  if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL) || forcedAlign)
                  {
                    //-----------------------------------------------------------Q Scan Z

                    DataSent_Controller("Scan");

                    digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
                    delay(5);

                    CMDOutput("AS");
                    K_OK = Scan_AllRange_TwoWay(2, FS_Count_Z, Z_ScanSTP, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Scan,Trip_");
                    CMDOutput("%:");

                    Q_Time = ((millis() - time_curing_0) / 1000);
                    MSGOutput("QT_IL:" + String(Q_Time)+ " s, " + String(PD_Now));
                    String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                    DataSent_Controller("AQ");
                    
                    if (!K_OK)
                    {
                      DataSent_Controller("Scan");

                      CMDOutput("AS");
                      Scan_AllRange_TwoWay(2, FS_Count_Z, Z_ScanSTP, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Re-Scan,Trip_");
                      CMDOutput("%:");

                      Q_Time = ((millis() - time_curing_0) / 1000);
                      MSGOutput("QT_IL:" + String(Q_Time)+ " s, " + String(PD_Now));
                      String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                      DataSent_Controller("AQ");
                    }
                  }
                  #pragma endregion

                  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
                  delay(5);
                  
                  if (isStop) break;
                }

                PD_Now = Cal_PD_Input_IL(Get_PD_Points);

                if (abs(PD_Before - PD_Now) < 0.3 && Q_Time > 750)
                {
                  IL_stable_count++;

                  if (IL_stable_count > 4 && !isStopAlign && Q_Time > 750)
                  {
                    MSGOutput("IL stable to break");
                    MSGOutput("Stable Time : " + String(Q_Time));
                    isStopAlign = true;
                  }
                }

                time_curing_1 = millis();
                time_curing_2 = time_curing_1;
              }
            }
          }

          time_curing_3 = millis();
          MSGOutput("Total Auto-Curing Time: " + String((time_curing_3 - time_curing_0) / 1000) + " s");

          StopValue = Target_IL;
          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

          String eepromString = ReadInfoEEPROM(40, 8);                         //Reading z backlash from EEPROM
          MSGOutput("Reset Z backlash from EEPROM: " + ReadInfoEEPROM(40, 8)); //(start_position, data_length)
          Z_backlash = eepromString.toInt();

          Target_IL = temp_targetIL;  //Re-cover Target IL value

          isLCD = true;
          PageLevel = 0;

          MSGOutput("Auto Q End");

          // updateUI(PageLevel);
          DataSent_Controller("Menu");

          MSGOutput("LCD Re-Start");

          Q_Time = 0;
        }
        cmd_No = 0;
        break;

      case 5: /* Fine Scan X */
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;

          Fine_Scan(X_Dir, false); 

          MSGOutput("Auto Align End");

          isLCD = true;

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 6: /* Fine Scan Y */
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;

          Fine_Scan(Y_Dir, false); 

          MSGOutput("Auto Align End");

          isLCD = true;

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 7: /* Fine Scan Z */
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;

          Fine_Scan(Z_Dir, false); 

          MSGOutput("Auto Align End");

          isLCD = true;

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 8:
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;

          MSGOutput("");
          MSGOutput("Fine Scan ");

          MSGOutput("Stop Value: " + String(StopValue));

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(5);

          bool K_OK = true;

          MotorCC_X = digitalRead(X_DIR_Pin);

          CMDOutput("AS");
          K_OK = Scan_Fast(0, FS_Count_X, FS_Steps_X, FS_Stable_X, MotorCC_X, FS_DelaySteps_X, StopValue, FS_Avg_X, FS_Trips_X, "X Scan,Trip_");
          CMDOutput("%:");

          if (!K_OK)
          {
            MotorCC_X = digitalRead(X_DIR_Pin);
            CMDOutput("AS");
            Scan_Fast(0, FS_Count_X, FS_Steps_X, FS_Stable_X, MotorCC_X, FS_DelaySteps_X, StopValue, FS_Avg_X, FS_Trips_X, "X Re-Scan,Trip_");
            CMDOutput("%:");
          }

          MSGOutput("Auto Align End");

          isLCD = true;

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 9:
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;

          MSGOutput("");
          MSGOutput("Fine Scan ");

          MSGOutput("Stop Value: " + String(StopValue));

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(5);

          bool K_OK = true;

          MotorCC_Y = digitalRead(Y_DIR_Pin);

          CMDOutput("AS");
          K_OK = Scan_Fast(0, FS_Count_Y, FS_Steps_Y, FS_Stable_Y, MotorCC_Y, FS_DelaySteps_Y, StopValue, FS_Avg_Y, FS_Trips_Y, "Y Scan,Trip_");
          CMDOutput("%:");

          if (!K_OK)
          {
            MotorCC_Y = digitalRead(Y_DIR_Pin);
            CMDOutput("AS");
            Scan_Fast(0, FS_Count_Y, FS_Steps_Y, FS_Stable_Y, MotorCC_Y, FS_DelaySteps_Y, StopValue, FS_Avg_Y, FS_Trips_Y, "Y Scan,Trip_");
            CMDOutput("%:");
          }

          MSGOutput("Auto Align End");

          isLCD = true;

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 10:
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;

          MSGOutput("");
          MSGOutput("Fine Scan ");

          MSGOutput("Stop Value: " + String(StopValue));

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(5);

          bool K_OK = true;

          MotorCC_Z = digitalRead(Z_DIR_Pin);

          CMDOutput("AS");
          K_OK = Scan_Fast(0, FS_Count_Z, FS_Steps_Z, FS_Stable_Z, MotorCC_Z, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Scan,Trip_");
          CMDOutput("%:");

          if (!K_OK)
          {
            MotorCC_Z = digitalRead(Z_DIR_Pin);
            CMDOutput("AS");
            Scan_Fast(0, FS_Count_Z, FS_Steps_Z, FS_Stable_Z, MotorCC_Z, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Scan,Trip_");
            CMDOutput("%:");
          }

          MSGOutput("Auto Align End");

          isLCD = true;

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 11:
        MSGOutput("Board ID: " + ReadInfoEEPROM(8, 8));
        cmd_No = 0;
        break;

      case 12:
        MSGOutput("Station ID: " + ReadInfoEEPROM(16, 8));
        cmd_No = 0;
        break;

      // case 13:
      //   MSGOutput("Server ID: " + String(server_ID)); 
      //   cmd_No = 0;
      //   break;

      // case 14:
      //   MSGOutput("Server Pw: " + String(server_Password)); 
      //   cmd_No = 0;
      //   break;

      case 18: /* Set Target IL */
        if (true)
        {
          StopValue = Target_IL;

          Serial.println("Update Target IL : " + WR_EEPROM(72, String(Target_IL)));
        }
        cmd_No = 0;
        break;

      case 19: /* Get Ref */
        if (true)
        {
          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(3);

          averagePDInput = 0;
          for (int i = 0; i < 20; i++)
            averagePDInput += ads.readADC_SingleEnded(0);

          averagePDInput = (averagePDInput / 20);

          ref_Dac = averagePDInput;
          ref_IL = ILConverter(averagePDInput);

          CleanEEPROM(0, 8); //Clean EEPROM(int startPosition, int datalength)

          WriteInfoEEPROM(String(averagePDInput), 0); //Write Data to EEPROM (data, start_position)
          EEPROM.commit();

          Serial.println("Ref_Dac: " + ReadInfoEEPROM(0, 8) + ", Ref_IL: " + String(ref_IL)); //Reading Data from EEPROM(start_position, data_length)

          MSGOutput("EEPROM(" + String(0) + ") - " + ReadInfoEEPROM(0, 8)); // For update HMI ref value

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
          delay(3);

          isLCD = true;
        }

        cmd_No = 0;
        break;

      // case 20:
      //   if(true)
      //   {
      //     // server_ID = ReadInfoEEPROM(88, 32);
      //     server_Password = ReadInfoEEPROM(120, 32);

      //     if (Contains(server_ID, "??") || server_ID == "")
      //     {
      //       server_ID = "GFI-ESP32-Access-Point";
      //     }

      //     if (Contains(server_Password, "??"))
      //     {
      //       server_Password = "22101782";
      //     }

      //     Serial.println("Server ID: " + server_ID);
      //     Serial.println("Server Password: " + server_Password);

      //     WiFi.begin(server_ID.c_str(), server_Password.c_str());
      //     Serial.println("Connecting");

      //     int wifiConnectTime = 0;
      //     while (WiFi.status() != WL_CONNECTED)
      //     {
      //       delay(300);
      //       Serial.print(".");

      //       wifiConnectTime += 300;
      //       if (wifiConnectTime > 2400)
      //         break;
      //     }

      //     if (wifiConnectTime <= 2400)
      //     {
      //       Serial.println("");
      //       Serial.print("Connected to WiFi network with IP Address:");
      //       Serial.println(WiFi.localIP());
      //       isWiFiConnected = true;
      //     }
      //     else
      //     {
      //       Serial.println("Connected to WiFi network failed");
      //     }
      //   }
      //   cmd_No = 0;
      //   break;

      case 21:
        isGetPower = true;

        Serial.println("Cmd: Get IL On");
        digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode

        cmd_No = 0;
        break;

      case 22:
        isGetPower = false;

        Serial.println("Cmd: Get Power Off");
        digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

        cmd_No = 0;
        break;

      case 23:
        GetPower_Mode = 1;
        Serial.println("Cmd: Get Power Mode: IL(dB)");
        cmd_No = 0;
        break;

      case 24:
        GetPower_Mode = 2;
        Serial.println("Cmd: Get Power Mode: Dac");
        cmd_No = 0;
        break;

      case 25:
        GetPower_Mode = 3;
        Serial.println("Cmd: Get Power Mode: Row IL(dBm)");
        cmd_No = 0;
        break;

      case 26:
        GetPower_Mode = 4;
        Serial.println("Cmd: Get Power Mode: Row Dac");
        cmd_No = 0;
        break;

      case 27:
        Serial.println(String(Cal_PD_Input_Row_Dac(Get_PD_Points)));
        cmd_No = 0;
        break;

      case 28:
        for (int i = 0; i < 511; i = i + 8)
        {
          if(i==88)
          {
            MSGOutput("EEPROM(" + String(i) + ") - " + ReadInfoEEPROM(i, 32)); //Server ID
            i=120;
          }
          else if(i==120)
          {
            MSGOutput("EEPROM(" + String(i) + ") - " + ReadInfoEEPROM(i, 32)); //Server Password
            i=152;
          }
          else
            MSGOutput("EEPROM(" + String(i) + ") - " + ReadInfoEEPROM(i, 8)); //Reading EEPROM(int start_position, int data_length)
        }

      case 29: /* Get XYZ Position */
        DataOutput(false);
        cmd_No = 0;
        break;

      case 31:
        isLCD = true;
        // LCD_Update_Mode = 100;
        Serial.println("LCD Re-Start");
        cmd_No = 0;
        break;

      // case 51: /* Get ID */
      //   Serial.println(ReadInfoEEPROM(8, 8));
      //   cmd_No = 0;
      //   break;
      }
    }

    //Functions: Motion
    if (cmd_No > 100)
      switch (cmd_No)
      {
        // Function: Cont-------------------------------------------------------------------------
        //Z feed - cont
      case 101:
        while (true)
        {
          MotorCC = MotorCC_Z;
          Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, false, 400 * MotorStepRatio, delayBetweenStep_Z);
          MotorCC_Z = false;

          if(Motor_Continous_Mode == 1)
          {
            if (digitalRead(R_3)) break;
          }
          else if(Motor_Continous_Mode == 2)
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
          else if (Motor_Continous_Mode == 3)
          {
            if(cmd_from_contr != "BS")
            {
              Serial.println("break:" + String(cmd_from_contr));
              cmd_from_contr = "";
              break;
            }
          }      

          
        }
        DataOutput();
        cmd_No = 0;
        break;
      case 103:
        while (true)
        {
          MotorCC = MotorCC_Z;
          Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, true, 400 * MotorStepRatio, delayBetweenStep_Z);
          MotorCC_Z = true;

          if(Motor_Continous_Mode == 1)
          {
            if (digitalRead(R_3)) break;
          }
          else if(Motor_Continous_Mode == 2)
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
          else if (Motor_Continous_Mode == 3)
          {
            if(cmd_from_contr != "BS")
            {
              Serial.println("break:" + String(cmd_from_contr));
              cmd_from_contr = "";
              break;
            }
          }    


        }
        DataOutput();
        cmd_No = 0;
        break;

        //X feed - cont
      case 102:

        while (true)
        {
          MotorCC = MotorCC_X;
          Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, false, 400 * MotorStepRatio, delayBetweenStep_X);
          MotorCC_X = false;

         if(Motor_Continous_Mode == 1)
          {
            if (digitalRead(R_3)) break;
          }
          else if(Motor_Continous_Mode == 2)
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
          else if (Motor_Continous_Mode == 3)
          {
            if(cmd_from_contr != "BS")
            {
              Serial.println("break:" + String(cmd_from_contr));
              cmd_from_contr = "";
              break;
            }
          }                
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;
        //X+ - cont
      case 105:

        while (true)
        {
          MotorCC = MotorCC_X;
          Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, true, 400 * MotorStepRatio, delayBetweenStep_X);
          MotorCC_X = true;

          if(Motor_Continous_Mode == 1)
          {
            if (digitalRead(R_2)) break;
          }
          else if(Motor_Continous_Mode == 2)
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
          else if (Motor_Continous_Mode == 3)
          {
            if(cmd_from_contr != "BS")
            {
              Serial.println("break:" + String(cmd_from_contr));
              cmd_from_contr = "";
              break;
            }
          }      

        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

        //Y- feed - cont
      case 106:

        while (true)
        {
          MotorCC = MotorCC_Y;
          Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, false, 400 * MotorStepRatio, delayBetweenStep_Y);
          MotorCC_Y = false;

          if(Motor_Continous_Mode == 1)
          {
            if (digitalRead(R_2)) break;
          }
          else if(Motor_Continous_Mode == 2)
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
          else if (Motor_Continous_Mode == 3)
          {
            if(cmd_from_contr != "BS")
            {
              Serial.println("break:" + String(cmd_from_contr));
              cmd_from_contr = "";
              break;
            }
          }      

        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

      //Y+ feed - cont
      case 104:

        while (true)
        {
          MotorCC = MotorCC_Y;
          Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, true, 400 * MotorStepRatio, delayBetweenStep_Y);
          MotorCC_Y = true;

          if(Motor_Continous_Mode == 1)
          {
            if (digitalRead(R_2)) break;
          }
          else if(Motor_Continous_Mode == 2)
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
          else if (Motor_Continous_Mode == 3)
          {
            if(cmd_from_contr != "BS")
            {
              Serial.println("break:" + String(cmd_from_contr));
              cmd_from_contr = "";
              break;
            }
          }               
        }

        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

        // Function: Jog-------------------------------------------------------------------------

      //X+ feed - jog
      case 107:
        MotorCC = MotorCC_X;
        Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, false, 400, delayBetweenStep_X);
        MotorCC_X = true;

        break;

        //X- feed - jog
      case 108:
        MotorCC = MotorCC_X;
        Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, true, 400, delayBetweenStep_X);
        MotorCC_X = false;

        break;

      //Y+ feed - jog
      case 109:
        MotorCC = MotorCC_Y;
        Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, false, 400, delayBetweenStep_Y);
        MotorCC_Y = true;
        break;

      //Y- feed - jog
      case 110:
        MotorCC = MotorCC_Y;
        Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, true, 400, delayBetweenStep_Y);
        MotorCC_Y = false;
        break;

        //Z+ feed - jog
      case 111:
        MotorCC = MotorCC_Z;
        Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, true, 400, delayBetweenStep_Z);
        MotorCC_Z = true;
        break;

        //Z- feed - jog
      case 112:
        MotorCC = MotorCC_Z;
        Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, false, 400, delayBetweenStep_Z);
        MotorCC_Z = false;
        break;

      //EmergencyStop
      case 129:
        isStop = true;
        Serial.println("EmergencyStop");
        digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
        isLCD = true;
        PageLevel = 0;
        break;

      //Go Home
      case 130:
        Move_Motor_abs_all(0, 0, 0);
        break;
      }
  
    cmd_from_contr = "";
    cmd_value_from_contr = "";
  }

  return cmd_No;
}

//------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------

// bool isWiFiConnected = false;
void CMDOutput(String cmd)
{
  String msg = "CMD::" + cmd;
  Serial.println(msg);

  if (isWiFiConnected)
    DataSent_Server(msg);

  // if (isWiFiConnected)
  // {
  //   httpTestRequest(ServerIP.c_str(), msg.c_str());
  // }

  // Check WiFi connection status
  // if (WiFi.status() == WL_CONNECTED)
  // {
  //   // Data = ServerIP + "?" + ID + "=" + msg;
  //   // httpTestRequest(Data.c_str()); //Send message to server
  //   httpTestRequest(ServerIP.c_str(), msg.c_str());

  //   isWiFiConnected = true;
  // }
  // else
  // {
  //   if (isWiFiConnected)
  //   {
  //     Serial.println("WiFi Disconnected");
  //     isWiFiConnected = false;
  //   }
  // }
}

void CMDOutput(String cmd, bool isSentServer)
{
  String msg = "CMD::" + cmd;
  Serial.println(msg);

  if (isSentServer)
    DataSent_Server(msg);
}

void DataOutput()
{
  double IL = Cal_PD_Input_IL(Get_PD_Points);
  Serial.println("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now) + "," + String(IL));
}

void DataOutput(bool isIL)
{
  if (isIL)
  {
    double IL = Cal_PD_Input_IL(Get_PD_Points);
    MSGOutput("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now) + "," + String(IL));
  }
  else
    MSGOutput("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now));
}

void DataOutput(int xyz, double pdValue)
{
  switch (xyz)
  {
  case 0:
    CMDOutput(">:" + String(X_Pos_Now) + "," + String(pdValue));
    break;

  case 1:
    CMDOutput(">:" + String(Y_Pos_Now) + "," + String(pdValue));
    break;

  case 2:
    CMDOutput(">:" + String(Z_Pos_Now) + "," + String(pdValue));
    break;
  }
}

long Get_Position(int xyz)
{
  switch (xyz)
  {
  case 0:
    return X_Pos_Now;
    break;

  case 1:
    return Y_Pos_Now;
    break;

  case 2:
    return Z_Pos_Now;
    break;
  }
}

void MSGOutput(String msg)
{
  Serial.println(msg);

  if (isWiFiConnected)
  {
    DataSent_Server(msg);
  }  

  // Check WiFi connection status
  // if (isWiFiConnected)
  // {
  //   httpTestRequest(ServerIP.c_str(), msg.c_str());
  // }
  // if (WiFi.status() == WL_CONNECTED)
  // {
  //   // Data = ServerIP + "?" + ID + "=" + msg;
  //   // httpTestRequest(Data.c_str()); //Send message to server

  //   httpTestRequest(ServerIP.c_str(), msg.c_str());

  //   isWiFiConnected = true;
  // }
  // else
  // {
  //   if (isWiFiConnected)
  //   {
  //     Serial.println("WiFi Disconnected");
  //     isWiFiConnected = false;
  //   }
  // }
}

bool isNumberic(String str)
{
  str.trim();
  unsigned int stringLength = str.length();
 
  if (stringLength == 0) {
      return false;
  }
 
  bool seenDecimal = false;
  bool seenMinus = false;
 
  for(unsigned int i = 0; i < stringLength; ++i) 
  {
      if (isDigit(str.charAt(i))) {
          continue;
      }

      if (str.charAt(i) == '-') 
      {
          if (seenMinus) 
          {
              return false;
          }
          seenMinus = true;
          continue;
      }

      if (str.charAt(i) == '.') 
      {
          if (seenDecimal) 
          {
              return false;
          }
          seenDecimal = true;
          continue;
      }
      return false;
  }
  return true;
}
