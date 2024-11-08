#include <EEPROM.h>
#include <curveFitting.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <BasicLinearAlgebra.h>
#include "config.h"

using namespace BLA;

TaskHandle_t Task_1, Task_2;
#define WDT_TIMEOUT 3

extern bool MotorCC_A;
extern bool MotorCC_X;
extern bool MotorCC_Y;
extern bool MotorCC_Z;

typedef struct struct_AlignResult
{
  bool IsResultGood = true;
  double Result_IL = -50;
  double Best_IL = -50;
  struct_Motor_Pos Result_Pos = {};
} struct_AlignResult;

struct_AlignResult AlignResult;

enum SpiralType
{
  rectangular,
  smooth
};

enum SpiralPlane
{
  XY,
  YZ,
  XZ
};

void CleanEEPROM(int startPosition, int datalength)
{
  for (size_t i = startPosition; i < (startPosition + datalength); i++)
  {
    EEPROM.write(i, ' ');
  }
}

void WriteInfoEEPROM(String data, int start_position)
{
  for (int i = 0; i < data.length(); ++i)
  {
    EEPROM.write(i + start_position, data[i]);
  }
}

/// @brief Comine CleanEEPROM and WriteInfoEEPROM together
/// @param data
/// @param start_position
/// @param maxDataLength
void WriteInfoEEPROM(int start_position, String data, int maxDataLength)
{
  if (data.length() > maxDataLength)
  {
    for (int i = start_position; i < (start_position + maxDataLength); i++)
    {
      EEPROM.write(i, ' ');
    }

    for (int i = 0; i < data.length(); ++i)
    {
      EEPROM.write(i + start_position, data[i]);
    }
  }
  else
  {
    for (int i = 0; i < maxDataLength; ++i)
    {
      if (i < data.length())
        EEPROM.write(i + start_position, data[i]);
      else
        EEPROM.write(i + start_position, ' ');
    }
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
  CleanEEPROM(start_position, 8); // Clean EEPROM(int startPosition, int datalength)

  WriteInfoEEPROM(String(data), start_position); // Write Data to EEPROM (data, start_position)
  EEPROM.commit();

  String s = ReadInfoEEPROM(start_position, 8);
  return s;
}

String WR_EEPROM(int start_position, int data_length, String data)
{
  CleanEEPROM(start_position, data_length); // Clean EEPROM(int startPosition, int datalength)

  WriteInfoEEPROM(String(data), start_position); // Write Data to EEPROM (data, start_position)
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

String ExtractCmd(String cmd, const char *header)
{
  int indexOffSet = cmd.indexOf(header);
  int strLength = strlen(header); // 使用 strlen 函式取得字串長度，並賦值給 int 變數

  cmd.remove(0, strLength + indexOffSet);

  cmd.trim();

  return cmd;
}

typedef enum
{
  HomeMade, /**< X direction */
  Phase_5,  /**< Y direction */
} Motor_Type;

Motor_Type motorType;

typedef enum
{
  X_Dir = 0,   /**< X direction */
  Y_Dir = 1,   /**< Y direction */
  Z_Dir = 2,   /**< Z direction */
  All_Dir = 3, /**< All direction */
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

bool plus_minus = false;

typedef struct struct_send_message
{
  String msg;
} struct_send_message;

// 接收資料格式
typedef struct struct_receive_message
{
  String contr_name;
  char cmd[30];
  char value[20];
  // String value;
} struct_receive_message;

// 送出資料格式
typedef struct struct_sendmsg_msg_UI_Data
{
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
  esp_err_t result = esp_now_send(ControllerAddress, (uint8_t *)&sendmsg_UI_Data, sizeof(sendmsg_UI_Data));
}

String name_from_contr = "", cmd_from_contr = "", cmd_value_from_contr = "";
unsigned long preMillis_DataRec = 0;
unsigned long nowMillis_DataRec = 0;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  String ss = "";
  ss.toCharArray(incomingReadings.cmd, 30);
  ss.toCharArray(incomingReadings.value, 20);
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  Msg = incomingReadings.cmd;
  Msg_Value = incomingReadings.value;

  nowMillis_DataRec = millis();

  if (Msg != "")
  {
    name_from_contr = incomingReadings.contr_name;
    cmd_from_contr = Msg;

    if (Msg_Value != "")
    {
      cmd_value_from_contr = Msg_Value;

      if (Msg == "BS" && Msg_Value == "0")
        EmergencyStop();
    }
    else
      Serial.println(incomingReadings.contr_name + Msg);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  status == ESP_NOW_SEND_SUCCESS;

  if (isCheckingServer)
  {
    Serial.println("CheckServerConnected:" + String(status));
    if (status == 0)
      isWiFiConnected = true;
    else
      isWiFiConnected = false;
  }
}

// Connect to Controll panel
void Task_1_sendData(void *pvParameters)
{
  while (true)
  {
    // Call ESP-Now receive data function
    if (Station_Type == 0)
    {
      esp_now_register_recv_cb(OnDataRecv);

      if (cmd_from_contr != "")
      {
        if (cmd_value_from_contr != "")
        {
          int valueContr = cmd_value_from_contr.toDouble();

          if (!isStop)
          {
            // Serial.print("msg:" + cmd_from_contr);
            // Serial.println(", value:" + cmd_value_from_contr);

            if (cmd_from_contr == "SS" && valueContr >= 101 && valueContr <= 106)
            {
              isStop = true;
            }
          }
        }
      }
    }

    CheckStop();

    // Task1休息，delay(1)不可省略
    delay(150);
  }
}

#define Enc_X_outputA 12 // 定義 X axis encoder A pin
#define Enc_X_outputB 14 // 定義 X axis encoder B pin
#define Enc_Y_outputA 27 // 定義 Y axis encoder A pinDIR_True
#define Enc_Y_outputB 26 // 定義 Y axis encoder B pin
#define Enc_Z_outputA 25 // 定義 Z axis encoder A pin
#define Enc_Z_outputB 33 // 定義 Z axis encoder B pin

int counter = 0;  // 定義 counter 為 int 類型變數，且初始值為0
int aState_X;     // 定義 aState 為 int 類型變數
int aLastState_X; // 定義 aLastState 為 int 類型變數
int aState_Y;     // 定義 aState 為 int 類型變數
int aLastState_Y; // 定義 aLastState 為 int 類型變數
int aState_Z;     // 定義 aState 為 int 類型變數
int aLastState_Z; // 定義 aLastState 為 int 類型變數

// bool X_DIR_True = true;
// bool X_DIR_False = false;
// bool Y_DIR_True = true;
// bool Y_DIR_False = false;
// bool Z_DIR_True = true;
// bool Z_DIR_False = false;

int Encoder_Motor_Step_X = 60;
int Encoder_Motor_Step_Y = 60;
int Encoder_Motor_Step_Z = 60;

bool isMotorManualCtr = true;

// int Encoder_Motor_Step = 60;
void Task_1_Encoder(void *pvParameters)
{
  while (true)
  {
    if (isMotorManualCtr)
    {
      // Encoder X
      {
        if (!IsEncDirtReverse_X)
          aState_X = digitalRead(Enc_X_outputA); // 將outputA的讀取值 設給 aState
        else
          aState_X = digitalRead(Enc_X_outputB); // 將outputA的讀取值 設給 aState

        // 條件判斷，當aState 不等於 aLastState時發生
        if (aState_X != aLastState_X)
        {
          MotorCC_A = MotorCC_X;

          // 條件判斷，當outputB讀取值 不等於 aState時發生
          if (!IsEncDirtReverse_X)
          {
            if (digitalRead(Enc_X_outputB) != aState_X)
              Move_Motor(X_DIR_Pin, X_STP_Pin, X_DIR_True, Encoder_Motor_Step_X, delayBetweenStep_X, 0, false, 0);
            else
              Move_Motor(X_DIR_Pin, X_STP_Pin, X_DIR_False, Encoder_Motor_Step_X, delayBetweenStep_X, 0, false, 0);
          }
          else
          {
            if (digitalRead(Enc_X_outputA) != aState_X)
              Move_Motor(X_DIR_Pin, X_STP_Pin, X_DIR_True, Encoder_Motor_Step_X, delayBetweenStep_X, 0, false, 0);
            else
              Move_Motor(X_DIR_Pin, X_STP_Pin, X_DIR_False, Encoder_Motor_Step_X, delayBetweenStep_X, 0, false, 0);
          }
        }

        aLastState_X = aState_X; // 將aState 最後的值 設給 aLastState
      }

      // Encoder Y
      {
        if (!IsEncDirtReverse_Y)
          aState_Y = digitalRead(Enc_Y_outputA); // 將outputA的讀取值 設給 aState
        else
          aState_Y = digitalRead(Enc_Y_outputB); // 將outputA的讀取值 設給 aState

        MotorCC_A = MotorCC_Y;

        // 條件判斷，當aState 不等於 aLastState時發生
        if (aState_Y != aLastState_Y)
        {
          // 條件判斷，當outputB讀取值 不等於 aState時發生
          if (!IsEncDirtReverse_Y)
          {
            if (digitalRead(Enc_Y_outputB) != aState_Y)
              Move_Motor(Y_DIR_Pin, Y_STP_Pin, Y_DIR_True, Encoder_Motor_Step_Y, delayBetweenStep_Y, 0, false, 0);
            else
              Move_Motor(Y_DIR_Pin, Y_STP_Pin, Y_DIR_False, Encoder_Motor_Step_Y, delayBetweenStep_Y, 0, false, 0);
          }
          else
          {
            if (digitalRead(Enc_Y_outputA) != aState_Y)
              Move_Motor(Y_DIR_Pin, Y_STP_Pin, Y_DIR_True, Encoder_Motor_Step_Y, delayBetweenStep_Y, 0, false, 0);
            else
              Move_Motor(Y_DIR_Pin, Y_STP_Pin, Y_DIR_False, Encoder_Motor_Step_Y, delayBetweenStep_Y, 0, false, 0);
          }
        }

        aLastState_Y = aState_Y; // 將aState 最後的值 設給 aLastState
      }

      // Encoder Z
      {
        if (!IsEncDirtReverse_Z)
          aState_Z = digitalRead(Enc_Z_outputA); // 將outputA的讀取值 設給 aState
        else
          aState_Z = digitalRead(Enc_Z_outputB); // 將outputA的讀取值 設給 aState

        MotorCC_A = MotorCC_Z;

        // 條件判斷，當aState 不等於 aLastState時發生
        if (aState_Z != aLastState_Z)
        {
          // 條件判斷，當outputB讀取值 不等於 aState時發生
          if (!IsEncDirtReverse_Z)
          {
            if (digitalRead(Enc_Z_outputB) != aState_Z)
              Move_Motor(Z_DIR_Pin, Z_STP_Pin, Z_DIR_True, Encoder_Motor_Step_Z, delayBetweenStep_Z, 0, false, 0);
            else
              Move_Motor(Z_DIR_Pin, Z_STP_Pin, Z_DIR_False, Encoder_Motor_Step_Z, delayBetweenStep_Z, 0, false, 0);
          }
          else
          {
            if (digitalRead(Enc_Z_outputA) != aState_Z)
              Move_Motor(Z_DIR_Pin, Z_STP_Pin, Z_DIR_True, Encoder_Motor_Step_Z, delayBetweenStep_Z, 0, false, 0);
            else
              Move_Motor(Z_DIR_Pin, Z_STP_Pin, Z_DIR_False, Encoder_Motor_Step_Z, delayBetweenStep_Z, 0, false, 0);
          }
        }

        aLastState_Z = aState_Z; // 將aState 最後的值 設給 aLastState
      }
    }
    else if (isKeepGetIL)
    {
      // SendIL();
      delay(50);
      continue;
    }
    else
      delay(50); // default: 50

    CheckStop();
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------
// int tarPZ = Encoder_Motor_Step_X * tan(11.5 * (PI / 180));
void Task_1_EncoderTilt(void *pvParameters)
{
  while (true)
  {
    if (isMotorManualCtr)
    {
      // Encoder X
      {
        aState_X = digitalRead(Enc_X_outputA); // 將outputA的讀取值 設給 aState

        // 條件判斷，當aState 不等於 aLastState時發生
        if (aState_X != aLastState_X)
        {
          MotorCC_A = MotorCC_X;

          // 條件判斷，當outputB讀取值 不等於 aState時發生
          if (digitalRead(Enc_X_outputB) != aState_X)
          {
            // int tarPX = Pos_Now.X + Encoder_Motor_Step_X;

            // Move_Motor_abs_all(tarPX, Pos_Now.Y, tarPZ, delayBetweenStep_X);
            Move_Motor(X_DIR_Pin, X_STP_Pin, X_DIR_True, Encoder_Motor_Step_X, delayBetweenStep_X, 0, false, 0);
            // Move_Motor(Z_DIR_Pin, Z_STP_Pin, X_DIR_False, tarPZ, delayBetweenStep_X, 0, false, 0);
          }

          else
          {
            // int tarPX = Pos_Now.X - En/coder_Motor_Step_X;
            // int tarPZ = Encoder_Motor_Step_X * tan(11.5 * (PI / 180));
            // Move_Motor_abs_all(Pos_Now.X - Encoder_Motor_Step_X, Pos_Now.Y, tarPZ, delayBetweenStep_X);
            Move_Motor(X_DIR_Pin, X_STP_Pin, X_DIR_False, Encoder_Motor_Step_X, delayBetweenStep_X, 0, false, 0);
            // Move_Motor(Z_DIR_Pin, Z_STP_Pin, X_DIR_True, tarPZ, delayBetweenStep_X, 0, false, 0);
          }
        }

        aLastState_X = aState_X; // 將aState 最後的值 設給 aLastState
      }

      // Encoder Y
      {
        aState_Y = digitalRead(Enc_Y_outputA); // 將outputA的讀取值 設給 aState

        MotorCC_A = MotorCC_Y;

        // 條件判斷，當aState 不等於 aLastState時發生
        if (aState_Y != aLastState_Y)
        {
          // 條件判斷，當outputB讀取值 不等於 aState時發生
          if (digitalRead(Enc_Y_outputB) != aState_Y)
          {
            Move_Motor(Y_DIR_Pin, Y_STP_Pin, Y_DIR_True, Encoder_Motor_Step_Y, delayBetweenStep_Y, 0, false, 0);
          }
          else
          {
            Move_Motor(Y_DIR_Pin, Y_STP_Pin, Y_DIR_False, Encoder_Motor_Step_Y, delayBetweenStep_Y, 0, false, 0);
          }
        }

        aLastState_Y = aState_Y; // 將aState 最後的值 設給 aLastState
      }

      // Encoder Z
      {
        aState_Z = digitalRead(Enc_Z_outputA); // 將outputA的讀取值 設給 aState

        MotorCC_A = MotorCC_Z;

        // 條件判斷，當aState 不等於 aLastState時發生
        if (aState_Z != aLastState_Z)
        {
          // 條件判斷，當outputB讀取值 不等於 aState時發生
          if (digitalRead(Enc_Z_outputB) != aState_Z)
          {
            Move_Motor(Z_DIR_Pin, Z_STP_Pin, Z_DIR_True, Encoder_Motor_Step_Z, delayBetweenStep_Z, 0, false, 0);
          }
          else
          {
            Move_Motor(Z_DIR_Pin, Z_STP_Pin, Z_DIR_False, Encoder_Motor_Step_Z, delayBetweenStep_Z, 0, false, 0);
          }
        }

        aLastState_Z = aState_Z; // 將aState 最後的值 設給 aLastState
      }
    }
    else
      delay(50);

    CheckStop();
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------
double maxIL_in_FineScan = 0;
double minIL_in_FineScan = -100;

bool Scan_AllRange_TwoWay(int XYZ, int count, int motorStep, int stableDelay,
                          bool Direction, int delayBetweenStep, double StopPDValue,
                          int Get_PD_Points, int Trips, String msg, double slope = 0.001,
                          double Tilt_X = 0, double Tilt_Y = 0, double Tilt_Z = 0)
{
  int DIR_Pin = 0; // 0:X, 1:Y, 2:Z
  int STP_Pin = 0;
  int backlash = 0;
  bool MotorCC_A = Direction; // initial direction
  int trip = 1;
  int dataCount = 3;
  int dataCount_ori;
  int indexofBestIL = 0;
  double PD_Value[4 * count + 1];
  long Step_Value[4 * count + 1];
  double Gradient_IL_Step[4 * count + 1];
  int GradientDownCount = 0;
  int GradientCount = 0;
  bool isGradCountFixGauss = false;
  double GradientTarget = slope; // default: 0.001
  unsigned long timer_1 = 0;
  timer_1 = millis();

  dataCount = 2 * count + 1;
  dataCount_ori = dataCount;

  switch (XYZ)
  {
  case X_Dir:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = X_backlash;
    GradientTarget = FS_GradientTarget_X; // 0.003
    msg = "X" + msg;
    delay(1);
    break;
  case Y_Dir:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = Y_backlash;
    GradientTarget = FS_GradientTarget_Y; // 0.002
    msg = "Y" + msg;
    delay(1);
    break;
  case Z_Dir:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = Z_backlash;
    GradientTarget = FS_GradientTarget_Z; // 0.003
    msg = "Z" + msg;
    delay(1);
    break;
  }

  bool iniWifiStatus = isWiFiConnected;
  if (!is_AutoCuring && Station_Type != 3)
  {
    isWiFiConnected = false;
  }

  MSGOutput("Scan Two Way");

  MSGOutput("isWiFiConnected : " + String(isWiFiConnected));
  CMDOutput(">>" + msg + String(trip));

  MSGOutput("Scan Step: " + String(motorStep));
  MSGOutput("StopValue:" + String(StopPDValue));
  MSGOutput("Backlash: " + String(backlash));
  MSGOutput("GradientTarget : " + String(GradientTarget, 4));

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);
  MSGOutput("Initial PD: " + String(PD_initial));

  // MSGOutput("ts:" + String(millis() - timer_1) + " ms");

  maxIL_in_FineScan = PD_initial;
  minIL_in_FineScan = PD_initial;

  if (PD_initial >= StopPDValue)
    return true;

  //-------------------------------------------Jump to Trip_1 initial position-------------------------------------

  step(STP_Pin, motorStep * count, delayBetweenStep, Direction); // First Jump
  delay(stableDelay * 1.5);                                      // Default : 100
  PD_Now = Get_IL_DAC(Get_PD_Points);

  DataOutput(PD_Now);
  DataOutput(XYZ, PD_Now); // int xyz, double pdValue

  // MSGOutput("ts:" + String(millis() - timer_1) + " ms");

  Serial.println("Jump IL: " + String(PD_Now));

  MotorCC_A = !MotorCC_A; // Reverse direction

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
    if (isStop)
      return true;

    if (i == 0)
    {
      PD_Value[i] = PD_Now;
      Step_Value[i] = Get_Position(XYZ);
      continue;
    }

    // 馬達移動
    step(STP_Pin, motorStep, delayBetweenStep, MotorCC_A);
    delay(stableDelay);

    // 記錄IL
    PD_Value[i] = Get_IL_DAC(Get_PD_Points);

    // 記錄位置
    Step_Value[i] = Get_Position(XYZ);

    if (PD_Value[i] > IL_Best_Trip1)
    {
      indexofBestIL = i;
      IL_Best_Trip1 = PD_Value[i];
      Pos_Best_Trip1 = Get_Position(XYZ);
    }

    // Update Min, Max IL in Scan Process
    if (PD_Value[i] > maxIL_in_FineScan)
      maxIL_in_FineScan = PD_Value[i];
    if (PD_Value[i] < minIL_in_FineScan)
      minIL_in_FineScan = PD_Value[i];

    DataOutput(PD_Value[i]);
    DataOutput(XYZ, PD_Value[i]); // int xyz, double pdValue

    // MSGOutput("ts : " + String(millis() - timer_1) + " ms");

    // Gradient analyze
    if (i > 0)
    {
      // 計算目前點斜率
      Gradient_IL_Step[i - 1] = (PD_Value[i] - PD_Value[i - 1]) / (motorStep / MotorStepRatio);

      if (XYZ == Z_Dir)
      {
        Gradient_IL_Step[i - 1] *= (FS_Steps_Z / FS_Steps_X);
      }

      MSGOutput("SlopeNow : " + String(Gradient_IL_Step[i - 1]));

      if (Gradient_IL_Step[i - 1] <= (GradientTarget * -1))
      {
        GradientCount = 0;

        GradientDownCount++;

        // 曲線前半段正向上升，並已過最佳點
        if (isGradCountFixGauss && GradientDownCount > 2 && PD_Value[i] > -4)
        {
          // Curfit
          if (indexofBestIL != 0 && Pos_Best_Trip1 != Get_Position(XYZ))
          {
            double x[3], y[3];
            for (int k = -1; k < 2; k++)
            {
              x[k + 1] = Step_Value[indexofBestIL + k]; // idex * step = real steps
              y[k + 1] = PD_Value[indexofBestIL + k];   // fill this with your sensor data
            }
            Pos_Best_Trip1 = Curfit(x, y, 3);

            MSGOutput("Best pos in Trip_1 (curfit) : " + String(Pos_Best_Trip1));
          }

          MSGOutput("Pass best IL (Slope)");

          break;
        }

        // 曲線反向下降
        else if (GradientDownCount > 4 && IL_Best_Trip1 > -20)
        {
          // Curfit
          if (indexofBestIL != 0 && Pos_Best_Trip1 != Get_Position(XYZ))
          {
            double x[3], y[3];
            for (int k = -1; k < 2; k++)
            {
              x[k + 1] = Step_Value[indexofBestIL + k]; // idex * step = real steps
              y[k + 1] = PD_Value[indexofBestIL + k];   // fill this with your sensor data
            }
            Pos_Best_Trip1 = Curfit(x, y, 3);

            MSGOutput("Best pos in Trip_1 (curfit) : " + String(Pos_Best_Trip1));
          }

          MSGOutput("Back Direction");

          break;
        }
      }
      else
      {
        GradientCount++;
        GradientDownCount = 0;

        // 判斷圖形是否為順向曲線
        if (GradientCount > 3 && !isGradCountFixGauss)
          isGradCountFixGauss = true;
      }

      // 以連續正斜率並目前斜率在目標斜率範圍內，停止耦光 default: -2
      if (i > 4 && PD_Value[i] > -3 && Gradient_IL_Step[i - 1] <= GradientTarget && GradientCount > 3)
      {
        MSGOutput("Gradient (" + String(Gradient_IL_Step[i - 1], 4) + ") <= " + String(GradientTarget, 4));

        if (XYZ == Z_Dir && GradientCount > 4)
        {
          MSGOutput("i:" + String(i) + ", Pos_Best_Trip1(Row):" + String(Pos_Best_Trip1));
          double x[5], y[5];
          for (int k = i - 4; k <= i; k++)
          {
            x[k - (i - 4)] = Step_Value[k]; // idex * step = real steps
            y[k - (i - 4)] = PD_Value[k];   // fill this with your sensor data
          }
          Pos_Best_Trip1 = Curfit_2(x, y, 5);
          MSGOutput("Best pos in Trip_1 (Curfit) : " + String(Pos_Best_Trip1));
        }

        DataOutput(PD_Value[i]);

        CMDOutput("t:" + String((millis() - timer_1) * 0.001, 2));

        isWiFiConnected = iniWifiStatus;

        return true;
      }
    }

    // 結束Trip 1, 進行curfitting
    if (false && IL_Best_Trip1 >= -2 && PD_Value[i] <= (IL_Best_Trip1 - 1.5) && Trips == 1 && i > 4)
    {
      Serial.println("IL < (IL-1.5): " + String(PD_Value[i]));

      // Curfit
      if (indexofBestIL != 0 && Pos_Best_Trip1 != Get_Position(XYZ))
      {
        double x[3], y[3];
        for (int k = -1; k < 2; k++)
        {
          x[k + 1] = Step_Value[indexofBestIL + k]; // idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];   // fill this with your sensor data
        }
        Pos_Best_Trip1 = Curfit(x, y, 3);

        MSGOutput("Best pos in Trip_1 (curfit) : " + String(Pos_Best_Trip1));
      }

      break;
    }

    // IL 大於 Stop threshold 判斷
    if (PD_Value[i] >= StopPDValue)
    {
      MSGOutput("Better than StopValue");
      return true;
    }

    if (Trips == 0 && i > 3)
    {
      if ((PD_Value[i] <= PD_Value[i - 1] || abs(PD_Value[i] - PD_Value[i - 1]) <= 0.02) && PD_Value[i] >= -1.8)
      {
        MSGOutput("Over best IL in trip 1");
        PD_Now = Get_IL_DAC(2 * Get_PD_Points);
        MSGOutput("Final IL: " + String(PD_Now));
        CMDOutput("t:" + String((millis() - timer_1) * 0.001, 2));
        DataOutput(PD_Now);
        return true;
      }
    }

    // 擴增點數
    if (i == (dataCount - 1) && Pos_Best_Trip1 == Get_Position(XYZ))
    {
      Serial.println("Datacount+3");
      dataCount = dataCount + 3;
      data_plus_time = data_plus_time + 1;

      motorStep *= 2;

      if (dataCount - dataCount_ori > 20 || data_plus_time > 5)
      {
        Serial.println("Data plus time: " + String(data_plus_time));
        CMDOutput("t:" + String((millis() - timer_1) * 0.001, 2));

        return true;
      }
    }

    // 結束Trip 1, 進行curfitting
    else if (indexofBestIL != 0 && i == (dataCount - 1) && Pos_Best_Trip1 != Get_Position(XYZ))
    {
      double x[3], y[3];
      for (int k = -1; k < 2; k++)
      {
        x[k + 1] = Step_Value[indexofBestIL + k]; // idex * step = real steps
        y[k + 1] = PD_Value[indexofBestIL + k];   // fill this with your sensor data
      }
      Pos_Best_Trip1 = Curfit(x, y, 3);
      MSGOutput("Best pos in Trip_1 (curfit) : " + String(Pos_Best_Trip1));
    }
  }

  trip++;

  double IL_Best_Trip2 = 0;
  long Pos_Best_Trip2 = 0;
  long Pos_Ini_Trip2 = 0;

  //------------------------------------------------------Trip_2 ------------------------------------------------------------
  MSGOutput(" --- Trip 2 --- ");
  MSGOutput("Trip Now: " + String(trip) + ", Trips Setting: " + String(Trips));

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

    MotorCC_A = !MotorCC_A; // Reverse direction

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

      step(STP_Pin, motorStep, delayBetweenStep, MotorCC_A);
      delay(stableDelay);

      PD_Value[i] = Get_IL_DAC(Get_PD_Points);
      Step_Value[i] = Get_Position(XYZ);

      if (PD_Value[i] > IL_Best_Trip2)
      {
        indexofBestIL = i;
        IL_Best_Trip2 = PD_Value[i];
        Pos_Best_Trip2 = Get_Position(XYZ);
      }

      // Update Min, Max IL in Scan Process
      if (PD_Value[i] > maxIL_in_FineScan)
        maxIL_in_FineScan = PD_Value[i];
      if (PD_Value[i] < minIL_in_FineScan)
        minIL_in_FineScan = PD_Value[i];

      DataOutput(PD_Value[i]);
      DataOutput(XYZ, PD_Value[i]); // int xyz, double pdValue
      DataSent_Server("PD Power:" + String(PD_Value[i]));

      // Gradient analyze
      if (i > 0)
      {
        Gradient_IL_Step[i - 1] = (PD_Value[i] - PD_Value[i - 1]) / motorStep;

        GradientCount = (Gradient_IL_Step[i - 1] <= -0.01) ? 0 : GradientCount + 1;

        if (i > 3)
        {
          if (PD_Value[i] > -1.35 && Gradient_IL_Step[i - 1] <= GradientTarget && GradientCount > 3 && PD_Value[i] >= IL_Best_Trip1)
          {
            MSGOutput("Gradient (" + String(Gradient_IL_Step[i - 1], 4) + ") <= Target : " + String(GradientTarget, 4));

            CMDOutput("t:" + String((millis() - timer_1) * 0.001, 2));

            isWiFiConnected = iniWifiStatus;

            return true;
          }
        }
      }

      if (false && IL_Best_Trip1 >= -2.5 && PD_Value[i] <= (IL_Best_Trip1 - 1.5) && Trips == 1)
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
        double x[3], y[3];
        for (int k = -1; k < 2; k++)
        {
          x[k + 1] = Step_Value[indexofBestIL + k]; // idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];   // fill this with your sensor data
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

  MSGOutput(" --- Trip 3 --- ");

  double PD_Best = IL_Best_Trip1;
  int deltaPos = 0;
  int BestPos = 0;

  {
    // Best IL in Trip 2
    if (IL_Best_Trip2 > IL_Best_Trip1 && (IL_Best_Trip2 - IL_Best_Trip1) > 0.05 && Trips != 1)
    {
      if (isStop)
        return true;

      MotorCC_A = !MotorCC_A; // Reverse direction
      digitalWrite(DIR_Pin, MotorCC_A);
      delay(1);

      MSGOutput("Best pos in Trip_2 : " + String(Pos_Best_Trip2)); //------------Best in Trip_2----------------

      // if (AQ_Scan_Compensation_Steps_Z_A != 0)
      // {
      //   if (XYZ == 2)
      //     Pos_Best_Trip2 = Pos_Best_Trip2 - (AQ_Scan_Compensation_Steps_Z_A * MotorStepRatio);

      //   MSGOutput("Best pos in Trip_2 (Compensation) : " + String(Pos_Best_Trip2));
      // }

      PD_Best = IL_Best_Trip2;

      BestPos = Pos_Best_Trip2;
    }

    else //------------Best in Trip_1----------------
    {
      MSGOutput("Best in Trip_1 : " + String(Pos_Best_Trip1));
      MSGOutput("Position Now : " + String(Get_Position(XYZ)));

      if (Pos_Best_Trip1 == Get_Position(XYZ))
      {
        PD_Now = Get_IL_DAC(2 * Get_PD_Points);

        CMDOutput("t:" + String((millis() - timer_1) * 0.001, 2));

        return (abs(PD_Now - IL_Best_Trip1) <= 0.12 || PD_Now > IL_Best_Trip1);
      }

      // Best IL in Trip 1 and Trip setting is 1
      if (Trips == 1)
      {
        MSGOutput("Jump to Trip Initial Pos : " + String(Pos_Ini_Trip1));
        Move_Motor_abs(XYZ, Pos_Ini_Trip1); // Jump to Trip_1 start position

        delay(stableDelay * 1.5); // 100

        PD_Now = Get_IL_DAC(Get_PD_Points);
        DataOutput(PD_Now);
        DataOutput(XYZ, PD_Now); // int xyz, double pdValue
      }

      MSGOutput("Best in Trip_1 : " + String(Pos_Best_Trip1));

      BestPos = Pos_Best_Trip1;
    }

    // Move to the best IL position in trip 1 & 2
    if (!isTrip3Jump)
    {
      int failCount = 0;
      double preIL = Get_IL_DAC(Get_PD_Points);

      while (true)
      {
        if (isStop)
          return true;

        // Feed a step to close to the best IL position
        if (deltaPos >= motorStep)
        {
          deltaPos = deltaPos - motorStep;

          Move_Motor(DIR_Pin, STP_Pin, MotorCC_A, motorStep, delayBetweenStep, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)

          PD_Now = Get_IL_DAC(Get_PD_Points);
          DataOutput(XYZ, PD_Now); // int xyz, double pdValue

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
          if (preIL >= (PD_Best - 0.09) && preIL >= PD_Now)
          {
            MSGOutput("Over IL_Best before");
            break;
          }

          preIL = (PD_Now < preIL) ? preIL + 1 : 0;

          if (preIL >= 4)
            break;

          preIL = PD_Now;
        }

        // Feed final step to the best IL position
        else if (deltaPos > 0 && deltaPos < motorStep)
        {
          Move_Motor(DIR_Pin, STP_Pin, MotorCC_A, deltaPos, delayBetweenStep, 0); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
          PD_Now = Get_IL_DAC(Get_PD_Points);
          DataOutput(XYZ, PD_Now); // int xyz, double pdValue
          break;
        }
        else
          break;
      }
    }
    else
    {
      Move_Motor_abs(XYZ, BestPos); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
      delay(stableDelay * 1.5);
      PD_Now = Get_IL_DAC(Get_PD_Points);
      DataOutput(PD_Now);
      DataOutput(XYZ, PD_Now); // int xyz, double pdValue
      MSGOutput("Trip 3 Jump : " + String(BestPos));
    }
  }

  MSGOutput("Best IL: " + String(PD_Best));
  MSGOutput("Final IL: " + String(PD_Now));

  CMDOutput("t:" + String(((millis() - timer_1) * 0.001), 2));

  isWiFiConnected = iniWifiStatus;

  return !(PD_Now < PD_Best - 0.5);
}

//------------------------------------------------------------------------------------------------------------------------------------------

bool Line_Scan_3D(int XYZ, int count, int motorStep, int stableDelay,
                  bool Direction, int delayBetweenStep, double StopPDValue,
                  int Get_PD_Points, int Trips, String msg, double slope = 0.001,
                  double Tilt_X = 0, double Tilt_Y = 0, double Tilt_Z = 0)
{
  int DIR_Pin = 0; // 0:X, 1:Y, 2:Z
  int STP_Pin = 0;
  int backlash = 0;
  bool MotorCC_A = Direction; // initial direction
  int trip = 1;
  int dataCount = 3;
  int dataCount_ori;
  int indexofBestIL = 0;
  double PD_Value[4 * count + 1];
  // long Step_Value[4 * count + 1];
  struct_Motor_Pos Pos_Virtual[4 * count + 1];
  struct_Motor_Pos Pos_Real[4 * count + 1];
  double Gradient_IL_Step[4 * count + 1];
  int GradientDownCount = 0;
  int GradientCount = 0;
  bool isGradCountFixGauss = false;
  double GradientTarget = slope; // default: 0.001
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
    GradientTarget = FS_GradientTarget_X; // 0.003
    break;
  case Y_Dir:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = Y_backlash;
    GradientTarget = FS_GradientTarget_Y; // 0.002
    break;
  case Z_Dir:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = Z_backlash;
    GradientTarget = FS_GradientTarget_Z; // 0.003
    break;
  }

  bool iniWifiStatus = isWiFiConnected;
  if (!is_AutoCuring && Station_Type != 3)
  {
    isWiFiConnected = false;
  }

  MSGOutput("Scan Two Way");
  MSGOutput("Scan Step: " + String(motorStep));
  MSGOutput("StopValue:" + String(StopPDValue));
  MSGOutput("Backlash: " + String(backlash));
  MSGOutput("GradientTarget : " + String(GradientTarget, 4));
  MSGOutput("isWiFiConnected : " + String(isWiFiConnected));
  CMDOutput(">>" + msg + String(trip));

  double PD_initial = Get_IL_DAC(3);
  MSGOutput("Initial PD: " + String(PD_initial));
  Serial.println("Pos : " + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z) + "=" + String(PD_initial));

  maxIL_in_FineScan = PD_initial;
  minIL_in_FineScan = PD_initial;

  if (PD_initial >= StopPDValue)
    return true;

  struct_Motor_Pos OriginalPos;
  OriginalPos.X = Pos_Now.X;
  OriginalPos.Y = Pos_Now.Y;
  OriginalPos.Z = Pos_Now.Z;

  struct_Motor_Pos TargetPos;
  TargetPos.X = 0;
  TargetPos.Y = 0;
  TargetPos.Z = 0;

  // Target position Matrix
  BLA::Matrix<3, 1> M_A;

  // Rotation Matrix
  BLA::Matrix<3, 3> M_X;
  BLA::Matrix<3, 3> M_Y;
  BLA::Matrix<3, 3> M_Z;

  double tx = Tilt_X * (PI / 180.0);
  double ty = Tilt_Y * (PI / 180.0);
  double tz = Tilt_Z * (PI / 180.0);

  M_X = {1, 0, 0, 0, cos(tx), -sin(tx), 0, sin(tx), cos(tx)}; // Rotation Matrix on X axis
  M_Y = {cos(ty), 0, sin(ty), 0, 1, 0, -sin(ty), 0, cos(ty)}; // Rotation Matrix on Y axis
  M_Z = {cos(tz), -sin(tz), 0, sin(tz), cos(tz), 0, 0, 0, 1};

  //-------------------------------------------Jump to Trip_1 initial position-------------------------------------
  digitalWrite(DIR_Pin, MotorCC_A);
  delay(5);

  if (true)
  {
    if (MotorCC_A)
    {
      if (XYZ == 0)
        TargetPos.X += motorStep * count;
      else if (XYZ == 1)
        TargetPos.Y += motorStep * count;
      else if (XYZ == 2)
        TargetPos.Z += motorStep * count;
    }
    else
    {
      if (XYZ == 0)
        TargetPos.X -= motorStep * count;
      else if (XYZ == 1)
        TargetPos.Y -= motorStep * count;
      else if (XYZ == 2)
        TargetPos.Z -= motorStep * count;
    }

    // 旋轉目標點
    M_A = {TargetPos.X, TargetPos.Y, TargetPos.Z};

    BLA::Matrix<3, 1> M = M_X * M_Y * M_Z * M_A;

    // 馬達移動(加上初始位移量)
    Move_Motor_abs_all(M(0) + OriginalPos.X, M(1) + OriginalPos.Y, M(2) + OriginalPos.Z, delayBetweenStep); // 初始位移
    // step(STP_Pin, motorStep * count, delayBetweenStep); // First Jump
    delay(250); // Default : 100
    PD_Now = Get_IL_DAC(Get_PD_Points);

    DataOutput(PD_Now);
    DataOutput(XYZ, PD_Now); // int xyz, double pdValue

    Serial.println("Jump IL: " + String(PD_Now));

    Serial.println("Pos : " + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z) + "=" + String(PD_Now));

    // Move_Motor_abs_all(OriginalPos.X, OriginalPos.Y, OriginalPos.Z, delayBetweenStep); // 初始位移
    // return 0;
  }

  MotorCC_A = !MotorCC_A; // Reverse direction
  digitalWrite(DIR_Pin, MotorCC_A);
  delay(5); // Default: 5

  //-------------------------------------------------------Trip_1 -----------------------------------------------

  if (PD_Now >= StopPDValue)
  {
    maxIL_in_FineScan = 0;
    minIL_in_FineScan = -100;
    return true;
  }

  double IL_Best_Trip1 = PD_Now;

  struct_Motor_Pos BestPosXYZ, Pos_Best_Trip1, Pos_Ini_Trip1, CurfitPos_OnAxis;
  BestPosXYZ = Get_Position();
  Pos_Best_Trip1 = Get_Position();
  Pos_Ini_Trip1 = Get_Position();
  CurfitPos_OnAxis = Get_Position();

  // Serial.println("Virtual Points");

  // 建立虛擬點陣列
  for (int i = 0; i < (4 * count + 1); i++)
  {
    PD_Value[i] = -10000;

    if (i == 0)
    {
      // Pos_Virtual[i] = Pos_Ini_Trip1;
      Pos_Virtual[i].X = TargetPos.X;
      Pos_Virtual[i].Y = TargetPos.Y;
      Pos_Virtual[i].Z = TargetPos.Z;
    }
    else
    {
      if (MotorCC_A)
      {
        if (XYZ == 0)
          Pos_Virtual[i].X = Pos_Virtual[i - 1].X + motorStep;
        else if (XYZ == 1)
          Pos_Virtual[i].Y = Pos_Virtual[i - 1].Y + motorStep;
        else if (XYZ == 2)
          Pos_Virtual[i].Z = Pos_Virtual[i - 1].Z + motorStep;
      }
      else
      {
        if (XYZ == 0)
          Pos_Virtual[i].X = Pos_Virtual[i - 1].X - motorStep;
        else if (XYZ == 1)
          Pos_Virtual[i].Y = Pos_Virtual[i - 1].Y - motorStep;
        else if (XYZ == 2)
          Pos_Virtual[i].Z = Pos_Virtual[i - 1].Z - motorStep;
      }
    }

    // Serial.println("Pos : " + String(Pos_Virtual[i].X) + "," + String(Pos_Virtual[i].Y) + "," + String(Pos_Virtual[i].Z) + "=" + String(PD_Value[i]));
  }

  Serial.println("Real Points");

  // return 0;

  double IL_Best_Trip = PD_Now;

  int data_plus_time = 0;

  for (int i = 0; i < dataCount; i++)
  {
    if (isStop)
      return true;

    if (i >= (4 * count))
    {
      break;
    }

    if (i == 0)
    {
      PD_Value[i] = PD_Now;

      // 記錄位置
      Pos_Real[i].X = Pos_Now.X;
      Pos_Real[i].Y = Pos_Now.Y;
      Pos_Real[i].Z = Pos_Now.Z;

      continue;
    }

    // 旋轉目標點
    M_A = {Pos_Virtual[i].X, Pos_Virtual[i].Y, Pos_Virtual[i].Z};

    BLA::Matrix<3, 1> M = M_X * M_Y * M_Z * M_A;

    // 馬達移動(加上初始位移量)
    Move_Motor_abs_all(M(0) + OriginalPos.X, M(1) + OriginalPos.Y, M(2) + OriginalPos.Z, delayBetweenStep); // 初始位移

    // 馬達移動
    // Move_Motor_abs_all(M(0), M(1), M(2), delayBetweenStep);
    delay(stableDelay);

    if (i > 1 && PD_Value[i - 1] > -2)
      delay(30);

    // 記錄IL
    PD_Value[i] = Get_IL_DAC(Get_PD_Points); // 2500

    // 記錄位置
    Pos_Real[i].X = Pos_Now.X;
    Pos_Real[i].Y = Pos_Now.Y;
    Pos_Real[i].Z = Pos_Now.Z;
    // Get_Position(Pos_Real[i]);

    if (PD_Value[i] > IL_Best_Trip1)
    {
      indexofBestIL = i;
      IL_Best_Trip1 = PD_Value[i];
      Pos_Best_Trip1 = Get_Position();
    }

    // Update Min, Max IL in Scan Process
    if (PD_Value[i] > maxIL_in_FineScan)
      maxIL_in_FineScan = PD_Value[i];
    if (PD_Value[i] < minIL_in_FineScan)
      minIL_in_FineScan = PD_Value[i];

    DataOutput(PD_Value[i]);
    DataOutput(XYZ, PD_Value[i]); // int xyz, double pdValue

    // Gradient analyze
    if (i > 0)
    {
      // 計算目前點斜率
      Gradient_IL_Step[i - 1] = (PD_Value[i] - PD_Value[i - 1]) / (motorStep / MotorStepRatio);

      if (XYZ == Z_Dir)
      {
        Gradient_IL_Step[i - 1] *= (FS_Steps_Z / FS_Steps_X);
      }

      if (Gradient_IL_Step[i - 1] <= (GradientTarget * -1))
      {
        GradientCount = 0;

        GradientDownCount++;

        // 曲線前半段正向上升，並已過最佳點
        if (isGradCountFixGauss && PD_Value[i] > -4)
        {
          // Curfit
          if (indexofBestIL != 0 && !Compare_Position(Pos_Best_Trip1, Pos_Now))
          {
            for (int h = 0; h <= 2; h++)
            {
              double x[3];
              double y[3];
              for (int k = -1; k < 2; k++)
              {
                x[k + 1] = Get_1D_Position(Pos_Real[indexofBestIL + k], h); // idex * step = real steps
                y[k + 1] = PD_Value[indexofBestIL + k];                     // fill this with your sensor data
              }

              if (x[0] == x[1] && x[1] == x[2])
              {
                // 軸上的理想點
                if (h == 0)
                  CurfitPos_OnAxis.X = x[0];
                else if (h == 1)
                  CurfitPos_OnAxis.Y = x[0];
                else if (h == 2)
                  CurfitPos_OnAxis.Z = x[0];
                continue;
              }

              if (y[0] == y[1] && y[1] == y[2])
              {
                // 軸上的理想點
                if (h == 0)
                  CurfitPos_OnAxis.X = x[1];
                else if (h == 1)
                  CurfitPos_OnAxis.Y = x[1];
                else if (h == 2)
                  CurfitPos_OnAxis.Z = x[1];
                continue;
              }

              long result = Curfit(x, y, 3);

              // 若結果超出範圍
              {
                if (result < x[0] || result > x[2])
                {
                  // 軸上的理想點
                  if (h == 0)
                    CurfitPos_OnAxis.X = x[1];
                  else if (h == 1)
                    CurfitPos_OnAxis.Y = x[1];
                  else if (h == 2)
                    CurfitPos_OnAxis.Z = x[1];

                  // MSGOutput("No fit");
                  MSGOutput("No fit (outrange):" + String(x[1]));

                  continue;
                }
              }

              // 軸上的理想點
              if (h == 0)
                CurfitPos_OnAxis.X = result;
              else if (h == 1)
                CurfitPos_OnAxis.Y = result;
              else if (h == 2)
                CurfitPos_OnAxis.Z = result;
            }

            Pos_Best_Trip1 = CurfitPos_OnAxis;

            MSGOutput("Best curfit IL position in Trip_1 is: " + Show_Position(Pos_Best_Trip1));
          }

          MSGOutput("Pass best IL");

          break;
        }

        // 曲線反向下降
        else if (GradientDownCount > 4 && IL_Best_Trip1 > -20)
        {
          // Curfit
          if (indexofBestIL != 0 && !Compare_Position(Pos_Best_Trip1, Pos_Now))
          {
            for (int h = 0; h <= 2; h++)
            {
              double x[3];
              double y[3];
              for (int k = -1; k < 2; k++)
              {
                x[k + 1] = Get_1D_Position(Pos_Real[indexofBestIL + k], h); // idex * step = real steps
                y[k + 1] = PD_Value[indexofBestIL + k];                     // fill this with your sensor data
              }

              if (x[0] == x[1] && x[1] == x[2])
              {
                // 軸上的理想點
                if (h == 0)
                  CurfitPos_OnAxis.X = x[0];
                else if (h == 1)
                  CurfitPos_OnAxis.Y = x[0];
                else if (h == 2)
                  CurfitPos_OnAxis.Z = x[0];

                MSGOutput("No fit");

                continue;
              }

              if (y[0] == y[1] && y[1] == y[2])
              {
                // 軸上的理想點
                if (h == 0)
                  CurfitPos_OnAxis.X = x[1];
                else if (h == 1)
                  CurfitPos_OnAxis.Y = x[1];
                else if (h == 2)
                  CurfitPos_OnAxis.Z = x[1];

                MSGOutput("No fit");

                continue;
              }

              long result = Curfit(x, y, 3);

              MSGOutput("FitArray_Pos:" + String(x[0]) + ", " + String(x[1]) + ", " + String(x[2]));

              // 若結果超出範圍
              {
                if (result < x[0] || result > x[2])
                {
                  // 軸上的理想點
                  if (h == 0)
                    CurfitPos_OnAxis.X = x[1];
                  else if (h == 1)
                    CurfitPos_OnAxis.Y = x[1];
                  else if (h == 2)
                    CurfitPos_OnAxis.Z = x[1];

                  // MSGOutput("No fit");
                  MSGOutput("No fit (outrange):" + String(x[1]));

                  continue;
                }

                if (result >= x[0] && result <= x[2])
                {
                }
                else
                {
                  // 軸上的理想點
                  if (h == 0)
                    CurfitPos_OnAxis.X = x[1];
                  else if (h == 1)
                    CurfitPos_OnAxis.Y = x[1];
                  else if (h == 2)
                    CurfitPos_OnAxis.Z = x[1];

                  // MSGOutput("No fit");
                  MSGOutput("No fit (outrange):" + String(x[1]));

                  continue;
                }
              }

              MSGOutput("fit:" + String(result));

              // 軸上的理想點
              if (h == 0)
                CurfitPos_OnAxis.X = result;
              else if (h == 1)
                CurfitPos_OnAxis.Y = result;
              else if (h == 2)
                CurfitPos_OnAxis.Z = result;
            }

            Pos_Best_Trip1 = CurfitPos_OnAxis;

            MSGOutput("Best curfit IL position in Trip_1 is: " + Show_Position(Pos_Best_Trip1));
          }

          MSGOutput("GradientDown - Back Direction");

          break;
        }
      }
      else
      {
        GradientCount++;
        GradientDownCount = 0;

        // 判斷圖形是否為順向曲線(擬合高斯)
        if (GradientCount > 3 && !isGradCountFixGauss)
          isGradCountFixGauss = true;
      }

      // 以連續正斜率並目前斜率在目標斜率範圍內，停止耦光 default: -2
      if (i > 4 && PD_Value[i] > -3 && Gradient_IL_Step[i - 1] <= GradientTarget && GradientCount > 3)
      {
        MSGOutput("Gradient (" + String(Gradient_IL_Step[i - 1], 4) + ") <= Target : " + String(GradientTarget, 4));

        if (XYZ == Z_Dir && GradientCount > 4)
        {
          MSGOutput("i:" + String(i) + ", Pos_Best_Trip1(Curfit):" + Show_Position(Pos_Best_Trip1));

          for (int h = 0; h <= 2; h++)
          {
            double x[5];
            double y[5];
            for (int k = i - 4; k <= i; k++)
            {
              x[k - (i - 4)] = Get_1D_Position(Pos_Virtual[k], h); // idex * step = real steps
              y[k - (i - 4)] = PD_Value[k];                        // fill this with your sensor data
            }

            if (x[0] == x[1] && x[1] == x[2] && x[2] == x[3] && x[3] == x[4])
            {
              // 軸上的理想點
              if (h == 0)
                CurfitPos_OnAxis.X = x[0];
              else if (h == 1)
                CurfitPos_OnAxis.Y = x[0];
              else if (h == 2)
                CurfitPos_OnAxis.Z = x[0];
              continue;
            }

            if (y[0] == y[1] && y[1] == y[2] && y[2] == y[3] && y[3] == y[4])
            {
              // 軸上的理想點
              if (h == 0)
                CurfitPos_OnAxis.X = x[1];
              else if (h == 1)
                CurfitPos_OnAxis.Y = x[1];
              else if (h == 2)
                CurfitPos_OnAxis.Z = x[1];
              continue;
            }

            long result = Curfit_2(x, y, 3);

            // 若結果超出範圍
            {
              if (result < x[0] || result > x[2])
              {
                // 軸上的理想點
                if (h == 0)
                  CurfitPos_OnAxis.X = x[1];
                else if (h == 1)
                  CurfitPos_OnAxis.Y = x[1];
                else if (h == 2)
                  CurfitPos_OnAxis.Z = x[1];

                // MSGOutput("No fit");
                MSGOutput("No fit (outrange):" + String(x[1]));

                continue;
              }
            }

            // 軸上的理想點
            if (h == 0)
              CurfitPos_OnAxis.X = result;
            else if (h == 1)
              CurfitPos_OnAxis.Y = result;
            else if (h == 2)
              CurfitPos_OnAxis.Z = result;
          }

          Pos_Best_Trip1 = CurfitPos_OnAxis;

          MSGOutput("Best pos in Trip_1 (curfit) is: " + Show_Position(Pos_Best_Trip1));
        }

        DataOutput(PD_Value[i]);

        timer_2 = millis();
        double ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        isWiFiConnected = iniWifiStatus;

        return true;
      }
    }

    // 結束Trip 1, 進行curfitting
    // if (IL_Best_Trip1 >= -2 && PD_Value[i] <= (IL_Best_Trip1 - 1.5) && Trips == 1 && i > 4)
    // {
    //   Serial.println("IL < (IL-1.5): " + String(PD_Value[i]));

    //   // Curfit
    //   if (indexofBestIL != 0 && !Compare_Position(Pos_Best_Trip1, Pos_Now))
    //   {
    //     for (int h = 0; h <= 2; h++)
    //     {
    //       double x[3];
    //       double y[3];
    //       for (int k = -1; k < 2; k++)
    //       {
    //         x[k + 1] = Get_1D_Position(Pos_Real[indexofBestIL + k], h); // idex * step = real steps
    //         y[k + 1] = PD_Value[indexofBestIL + k];                     // fill this with your sensor data
    //       }

    //       if (x[0] == x[1] && x[1] == x[2])
    //       {
    //         // 軸上的理想點
    //         if (h == 0)
    //           CurfitPos_OnAxis.X = x[0];
    //         else if (h == 1)
    //           CurfitPos_OnAxis.Y = x[0];
    //         else if (h == 2)
    //           CurfitPos_OnAxis.Z = x[0];
    //         continue;
    //       }

    //       if (y[0] == y[1] && y[1] == y[2])
    //       {
    //         // 軸上的理想點
    //         if (h == 0)
    //           CurfitPos_OnAxis.X = x[1];
    //         else if (h == 1)
    //           CurfitPos_OnAxis.Y = x[1];
    //         else if (h == 2)
    //           CurfitPos_OnAxis.Z = x[1];
    //         continue;
    //       }

    //       long result = Curfit(x, y, 3);

    //       // 若結果超出範圍
    //       {
    //         if (result < x[0] || result > x[2])
    //         {
    //           // 軸上的理想點
    //           if (h == 0)
    //             CurfitPos_OnAxis.X = x[1];
    //           else if (h == 1)
    //             CurfitPos_OnAxis.Y = x[1];
    //           else if (h == 2)
    //             CurfitPos_OnAxis.Z = x[1];

    //           // MSGOutput("No fit");
    //           MSGOutput("No fit (outrange):" + String(x[1]));

    //           continue;
    //         }
    //       }

    //       // 軸上的理想點
    //       if (h == 0)
    //         CurfitPos_OnAxis.X = result;
    //       else if (h == 1)
    //         CurfitPos_OnAxis.Y = result;
    //       else if (h == 2)
    //         CurfitPos_OnAxis.Z = result;
    //     }

    //     Pos_Best_Trip1 = CurfitPos_OnAxis;

    //     MSGOutput("Best curfit IL position in Trip_1 is: " + Show_Position(Pos_Best_Trip1));
    //   }

    //   break;
    // }

    // IL 大於 Stop threshold 判斷
    if (PD_Value[i] >= StopPDValue)
    {
      MSGOutput("Better than StopValue");
      return true;
    }

    if (Trips == 0 && i > 3)
    {
      if ((PD_Value[i] <= PD_Value[i - 1] || abs(PD_Value[i] - PD_Value[i - 1]) <= 0.02) && PD_Value[i] >= -1.8)
      {
        MSGOutput("Over best IL in trip 1");
        PD_Now = Get_IL_DAC(2 * Get_PD_Points);
        MSGOutput("Final IL: " + String(PD_Now));
        timer_2 = millis();
        double ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        DataOutput(PD_Now);
        return true;
      }
    }

    // 擴增點數
    if (i == (dataCount - 1) && Compare_Position(Pos_Best_Trip1, Pos_Now))
    {
      Serial.println("Datacount+3");
      dataCount = dataCount + 3;
      data_plus_time = data_plus_time + 1;

      motorStep *= 2;

      if (dataCount - dataCount_ori > 20 || data_plus_time > 5)
      {
        Serial.println("Data plus time: " + String(data_plus_time));
        timer_2 = millis();
        double ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }
    }

    // 結束Trip 1, 進行curfitting
    else if (indexofBestIL != 0 && i == (dataCount - 1) && !Compare_Position(Pos_Best_Trip1, Pos_Now))
    {
      for (int h = 0; h <= 2; h++)
      {
        double x[3];
        double y[3];
        for (int k = -1; k < 2; k++)
        {
          x[k + 1] = Get_1D_Position(Pos_Real[indexofBestIL + k], h); // idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];                     // fill this with your sensor data
        }

        if (x[0] == x[1] && x[1] == x[2])
        {
          // 軸上的理想點
          if (h == 0)
            CurfitPos_OnAxis.X = x[0];
          else if (h == 1)
            CurfitPos_OnAxis.Y = x[0];
          else if (h == 2)
            CurfitPos_OnAxis.Z = x[0];
          continue;
        }

        if (y[0] == y[1] && y[1] == y[2])
        {
          // 軸上的理想點
          if (h == 0)
            CurfitPos_OnAxis.X = x[1];
          else if (h == 1)
            CurfitPos_OnAxis.Y = x[1];
          else if (h == 2)
            CurfitPos_OnAxis.Z = x[1];
          continue;
        }

        long result = Curfit(x, y, 3);

        // 若結果超出範圍
        {
          if (result < x[0] || result > x[2])
          {
            // 軸上的理想點
            if (h == 0)
              CurfitPos_OnAxis.X = x[1];
            else if (h == 1)
              CurfitPos_OnAxis.Y = x[1];
            else if (h == 2)
              CurfitPos_OnAxis.Z = x[1];

            // MSGOutput("No fit");
            MSGOutput("No fit (outrange):" + String(x[1]));

            continue;
          }
        }

        // 軸上的理想點
        if (h == 0)
          CurfitPos_OnAxis.X = result;
        else if (h == 1)
          CurfitPos_OnAxis.Y = result;
        else if (h == 2)
          CurfitPos_OnAxis.Z = result;
      }

      Pos_Best_Trip1 = CurfitPos_OnAxis;

      MSGOutput("Best curfit IL position in Trip_1 is: " + Show_Position(Pos_Best_Trip1));
      MSGOutput("End Trip1");
    }
  }

  trip++;

  double IL_Best_Trip2 = 0;
  struct_Motor_Pos Pos_Best_Trip2, Pos_Ini_Trip2;

  //------------------------------------------------------Trip_2 ------------------------------------------------------------
  MSGOutput(" --- Trip 2 --- ");
  MSGOutput("Trip Now: " + String(trip) + ", Trips Setting: " + String(Trips));

  GradientCount = 0;

  if (Trips != 1)
  {
    CMDOutput("~:" + msg + String(trip));

    IL_Best_Trip2 = PD_Now;

    Pos_Best_Trip2 = Get_Position();
    Pos_Ini_Trip2 = Get_Position();

    // 建立虛擬點陣列
    for (int i = 0; i < dataCount; i++)
    {
      PD_Value[i] = -80;

      if (i == 0)
      {
        Pos_Virtual[i].X = 0;
        Pos_Virtual[i].Y = 0;
        Pos_Virtual[i].Z = 0;
      }
      else
      {
        if (MotorCC_A)
        {
          if (XYZ == 0)
            Pos_Virtual[i].X = Pos_Virtual[i - 1].X + motorStep;
          else if (XYZ == 1)
            Pos_Virtual[i].Y = Pos_Virtual[i - 1].Y + motorStep;
          else if (XYZ == 2)
            Pos_Virtual[i].Z = Pos_Virtual[i - 1].Z + motorStep;
        }
        else
        {
          if (XYZ == 0)
            Pos_Virtual[i].X = Pos_Virtual[i - 1].X - motorStep;
          else if (XYZ == 1)
            Pos_Virtual[i].Y = Pos_Virtual[i - 1].Y - motorStep;
          else if (XYZ == 2)
            Pos_Virtual[i].Z = Pos_Virtual[i - 1].Z - motorStep;
        }
      }
    }

    MotorCC_A = !MotorCC_A; // Reverse direction
    digitalWrite(DIR_Pin, MotorCC_A);
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
        continue;
      }

      // 旋轉目標點
      M_A = {Pos_Virtual[i].X, Pos_Virtual[i].Y, Pos_Virtual[i].Z};

      BLA::Matrix<3, 1> M = M_X * M_Y * M_Z * M_A;

      // 馬達移動(加上初始位移量)
      Move_Motor_abs_all(M(0) + OriginalPos.X, M(1) + OriginalPos.Y, M(2) + OriginalPos.Z, delayBetweenStep); // 初始位移

      delay(stableDelay);

      if (i > 1 && PD_Value[i - 1] > -2)
        delay(30);

      PD_Value[i] = Get_IL_DAC(Get_PD_Points);

      if (PD_Value[i] > IL_Best_Trip2)
      {
        indexofBestIL = i;
        IL_Best_Trip2 = PD_Value[i];
        Pos_Best_Trip2 = Get_Position();
      }

      // Update Min, Max IL in Scan Process
      if (PD_Value[i] > maxIL_in_FineScan)
        maxIL_in_FineScan = PD_Value[i];
      if (PD_Value[i] < minIL_in_FineScan)
        minIL_in_FineScan = PD_Value[i];

      DataOutput(PD_Value[i]);
      DataOutput(XYZ, PD_Value[i]); // int xyz, double pdValue

      // Gradient analyze
      if (i > 0)
      {
        Gradient_IL_Step[i - 1] = (PD_Value[i] - PD_Value[i - 1]) / motorStep;

        if (Gradient_IL_Step[i - 1] <= -0.01)
          GradientCount = 0;
        else
          GradientCount++;

        if (i > 3)
        {
          if (PD_Value[i] > -1.35 && Gradient_IL_Step[i - 1] <= GradientTarget && GradientCount > 3 && PD_Value[i] >= IL_Best_Trip1)
          {
            MSGOutput("Gradient (" + String(Gradient_IL_Step[i - 1], 4) + ") <= Target : " + String(GradientTarget, 4));

            timer_2 = millis();
            double ts = (timer_2 - timer_1) * 0.001;
            CMDOutput("t:" + String(ts, 2));

            isWiFiConnected = iniWifiStatus;

            return true;
          }
        }
      }

      if (IL_Best_Trip1 >= -2.5 && PD_Value[i] <= (IL_Best_Trip1 - 1.5) && Trips == 1)
      {
        Serial.println("IL < (IL-1.5): " + String(PD_Value[i]));
        break;
      }

      if (PD_Value[i] >= StopPDValue)
      {
        MSGOutput("Better than StopValue");
        return true;
      }

      if (indexofBestIL != 0 && i == (dataCount - 1) && !Compare_Position(Pos_Best_Trip2, Pos_Now))
      {
        double x[3];
        double y[3];
        for (int k = -1; k < 2; k++)
        {
          // x[k + 1] = Step_Value[indexofBestIL + k]; // idex * step = real steps
          x[k + 1] = Get_1D_Position(Pos_Virtual[indexofBestIL + k], XYZ); // idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];                          // fill this with your sensor data
        }

        long result = Curfit(x, y, 3);

        // 軸上的理想點
        if (XYZ == 0)
          CurfitPos_OnAxis.X = result;
        else if (XYZ == 1)
          CurfitPos_OnAxis.Y = result;
        else if (XYZ == 2)
          CurfitPos_OnAxis.Z = result;

        // 矩陣旋轉理想點
        M_A = {CurfitPos_OnAxis.X, CurfitPos_OnAxis.Y, CurfitPos_OnAxis.Z};

        BLA::Matrix<3, 1> M = M_X * M_Y * M_Z * M_A;

        Pos_Best_Trip2.X = M(0) + OriginalPos.X;
        Pos_Best_Trip2.Y = M(1) + OriginalPos.Y;
        Pos_Best_Trip2.Z = M(2) + OriginalPos.Z;

        MSGOutput("Best curfit IL position in Trip_2 is: " + Show_Position(Pos_Best_Trip2));
      }
    }
  }
  else
    trip--;

  trip++;
  CMDOutput("~:" + msg + String(trip));

  //------------------------------------Trip_3 -------------------------------------------------------

  MSGOutput(" --- Trip 3 --- ");

  double PD_Best = IL_Best_Trip1;
  int deltaPos = 0;
  int BestPos = 0;

  if (true)
  {
    // Best IL in Trip 2
    if (IL_Best_Trip2 > IL_Best_Trip1 && (IL_Best_Trip2 - IL_Best_Trip1) > 0.05 && Trips != 1)
    {
      if (isStop)
        return true;

      MotorCC_A = !MotorCC_A; // Reverse direction
      digitalWrite(DIR_Pin, MotorCC_A);
      delay(15);

      MSGOutput("Best pos in Trip_2 : " + Show_Position(Pos_Best_Trip2)); //------------Best in Trip_2----------------

      PD_Best = IL_Best_Trip2;

      BestPosXYZ = Pos_Best_Trip2;
    }

    else //------------Best in Trip_1----------------
    {
      MSGOutput("Best in Trip_1 : " + Show_Position(Pos_Best_Trip1));
      MSGOutput("Position Now : " + String(Get_Position(XYZ)));

      if (Compare_Position(Pos_Best_Trip1, Pos_Now))
      {
        PD_Now = Get_IL_DAC(2 * Get_PD_Points);

        timer_2 = millis();
        double ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        if (abs(PD_Now - IL_Best_Trip1) <= 0.12 || PD_Now > IL_Best_Trip1)
          return true;
        else
          return false;
      }

      // Best IL in Trip 1 and Trip setting is 1
      if (Trips == 1)
      {
        MSGOutput("Jump to Trip Initial Pos : " + Show_Position(Pos_Ini_Trip1));
        Move_Motor_abs_all(Pos_Ini_Trip1.X, Pos_Ini_Trip1.Y, Pos_Ini_Trip1.Z, delayBetweenStep); // Jump to Trip_1 start position

        delay(300); // 100

        PD_Now = Get_IL_DAC(Get_PD_Points);
        DataOutput(PD_Now);
        DataOutput(XYZ, PD_Now); // int xyz, double pdValue
      }

      BestPosXYZ = Pos_Best_Trip1;
    }

    // Move to the best IL position in trip 1 & 2
    if (!isTrip3Jump)
    {
      int failCount = 0;
      double preIL = Get_IL_DAC(Get_PD_Points);

      while (true)
      {
        if (isStop)
          return true;

        // Feed a step to close to the best IL position
        if (deltaPos >= motorStep)
        {
          deltaPos = deltaPos - motorStep;

          Move_Motor(DIR_Pin, STP_Pin, MotorCC_A, motorStep, delayBetweenStep, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)

          PD_Now = Get_IL_DAC(Get_PD_Points);
          DataOutput(XYZ, PD_Now); // int xyz, double pdValue

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
          if (preIL >= (PD_Best - 0.09) && preIL >= PD_Now)
          {
            MSGOutput("Over IL_Best before");
            break;
          }

          if (PD_Now < preIL)
            preIL++;
          else
            preIL = 0;

          if (preIL >= 4)
            break;

          preIL = PD_Now;
        }

        // Feed final step to the best IL position
        else if (deltaPos > 0 && deltaPos < motorStep)
        {
          Move_Motor(DIR_Pin, STP_Pin, MotorCC_A, deltaPos, delayBetweenStep, 0); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
          PD_Now = Get_IL_DAC(Get_PD_Points);
          DataOutput(XYZ, PD_Now); // int xyz, double pdValue
          break;
        }
        else
          break;
      }
    }
    else
    {
      Move_Motor_abs_all(BestPosXYZ.X, BestPosXYZ.Y, BestPosXYZ.Z, delayBetweenStep); // Jump to Trip_1 start position

      delay(300);
      PD_Now = Get_IL_DAC(Get_PD_Points);
      DataOutput(PD_Now);
      DataOutput(XYZ, PD_Now); // int xyz, double pdValue
      MSGOutput("Trip 3 Jump : " + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z) + "=" + String(PD_initial));
    }
  }

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

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(20); // 設定序列埠接收資料時的最大等待時間

  EEPROM.begin(4096); // 宣告使用EEPROM 4096 個位置

  // I2C Setting for 16 bits adc (Get PD value)
  I2CADS.begin(I2C_SDA, I2C_SCL, 400000);
  ads.setGain(GAIN_TWO); // 2x gain   +/- 2.048V  1 bit = 0.0625mV
  ads.setDataRate(RATE_ADS1115_860SPS);
  if (!ads.begin(0x48, &I2CADS))
  {
    Serial.println("Failed to initialize ADS.");
    isGetPower = false;
  }

  // Wire.begin();
  // Wire.setClock(400000);
  // pinMode(interruptPin, INPUT_PULLUP);
  // if (!ads.init())
  // {
  //   Serial.println("ADS1115 not connected!");
  // }

  // /* Set the voltage range of the ADC to adjust the gain
  //  * Please note that you must not apply more than VDD + 0.3V to the input pins!  */
  // ads.setVoltageRange_mV(ADS1115_RANGE_2048); // comment line/change parameter to change range

  // /* Set the inputs to be compared  */
  // ads.setCompareChannels(ADS1115_COMP_0_GND); // comment line/change parameter to change channels

  // ads.setAlertPinMode(ADS1115_ASSERT_AFTER_1); // needed in this sketch to enable alert pin (doesn't matter if you choose after 1,2 or 4)

  // /* Set the conversion rate in SPS (samples per second)  */
  // ads.setConvRate(ADS1115_860_SPS); // comment line/change parameter to change SPS

  // ads.setMeasureMode(ADS1115_CONTINUOUS); // the conversion ready alert pin also works in continuous mode

  // ads.setAlertPinToConversionReady(); // needed for this sketch

  // attachInterrupt(digitalPinToInterrupt(interruptPin), convReadyAlert, FALLING);
  // oldTime = micros();

  // convReady = true;

  // Serial.println("ADS1115 Example Sketch - Continuious, Conversion Ready Alert Pin controlled");
  // Serial.println();

  String epString = ReadInfoEEPROM(EP_Station_Type, 8);
  Station_Type = isNumberic(epString) ? epString.toInt() : Station_Type;
  MSGOutput("Station Type: " + String(Station_Type));

#pragma region : WiFi Server Setting

  // 初始化 ESP-NOW
  if (true)
  {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != 0)
    {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    else
      Serial.println("Initializing ESP-NOW");

    ThisAddr = WiFi.macAddress();
    // Serial.printf("ESP32 Board MAC Address: %s\n", ThisAddr);
    Serial.print("ESP32 Board MAC Address:  ");
    Serial.println(ThisAddr);

    char Colon = ':';
    int startP = 0;
    uint8_t value;
    int indexCount = 0;

    // This Address : String to Hex
    while (indexCount < sizeof(ThisAddress))
    {
      int index = ThisAddr.indexOf(Colon, startP);
      if (index != -1)
      {
        String subS = ThisAddr.substring(startP, index);
        value = strtoul(subS.c_str(), NULL, 16); // string to Hex
        startP = index + 1;

        ThisAddress[indexCount] = value;
        // Serial.println("value:" + String(value, HEX));
      }
      else // last one
      {
        String subS = ThisAddr.substring(startP);
        value = strtoul(subS.c_str(), NULL, 16);
        ThisAddress[indexCount] = value;
        // Serial.println("value:" + String(value, HEX));
      }
      indexCount++;
    }

    // 設置發送數據回傳函數
    esp_now_register_send_cb(OnDataSent);

    // 绑定數據接收端
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));           // initialize peer if esp32 library version is 2.0.1 (no need in version 1.0.6)
    memcpy(peerInfo.peer_addr, ControllerAddress, 6); // Register peer
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // 绑定Server數據接收端
    esp_now_peer_info_t peerInfo_server;
    memset(&peerInfo_server, 0, sizeof(peerInfo_server)); // initialize peer if esp32 library version is 2.0.1 (no need in version 1.0.6)
    memcpy(peerInfo_server.peer_addr, ServerAddress, 6);  // Register peer
    peerInfo_server.channel = 0;
    peerInfo_server.encrypt = false;

    // // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer_controller");
      return;
    }

    if (esp_now_add_peer(&peerInfo_server) != ESP_OK)
    {
      Serial.println("Failed to add peer_server");
      return;
    }
  }

#pragma endregion

#pragma region pinMode Setting

  pinMode(X_STP_Pin, OUTPUT);
  pinMode(X_DIR_Pin, OUTPUT);
  pinMode(Y_STP_Pin, OUTPUT);
  pinMode(Y_DIR_Pin, OUTPUT);
  pinMode(Z_STP_Pin, OUTPUT);
  pinMode(Z_DIR_Pin, OUTPUT);

  pinMode(AWO_Pin, OUTPUT);

  // VOA Station - Heater
  if (Station_Type == 1)
  {
    pinMode(Heater_Start_Pin, OUTPUT);
    pinMode(Heater_Stop_Pin, OUTPUT);
  }
  else if (Station_Type == 3 || Station_Type == 4)
  {
    pinMode(Enc_X_outputA, INPUT_PULLUP);
    pinMode(Enc_X_outputB, INPUT_PULLUP);
    pinMode(Enc_Y_outputA, INPUT_PULLUP);
    pinMode(Enc_Y_outputB, INPUT_PULLUP);
    pinMode(Enc_Z_outputA, INPUT_PULLUP);
    pinMode(Enc_Z_outputB, INPUT_PULLUP);

    aLastState_X = digitalRead(Enc_X_outputA); // 將初始outputA的讀取值 設給 aLastState
    aLastState_Y = digitalRead(Enc_Y_outputA); // 將初始outputA的讀取值 設給 aLastState
    aLastState_Z = digitalRead(Enc_Z_outputA); // 將初始outputA的讀取值 設給 aLastState
  }

  pinMode(Tablet_PD_mode_Trigger_Pin, OUTPUT);    // Control Tablet Mode
  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode

#pragma endregion

  Serial.println("~~ Auto-Align System ~~");

#pragma region EEPROM Setting

  String eepromString;

  // Initialize PD_Ref_Array
  int iniEP = 700;
  if (true)
  {
    for (size_t i = 0; i < sizeof(EP_PD_Ref_Array) / sizeof(EP_PD_Ref_Array[0]); i++)
    {
      EP_PD_Ref_Array[i][0] = iniEP;
      EP_PD_Ref_Array[i][1] = (iniEP + 8);
      iniEP += 16;

      eepromString = ReadInfoEEPROM(EP_PD_Ref_Array[i][0], 8);
      PD_Ref_Array[i][0] = isNumberic(eepromString) ? eepromString.toDouble() : PD_Ref_Array[i][0];

      eepromString = ReadInfoEEPROM(EP_PD_Ref_Array[i][1], 8);
      PD_Ref_Array[i][1] = isNumberic(eepromString) ? eepromString.toDouble() : PD_Ref_Array[i][1];

      if (false)
        Serial.printf("[%d, %d]:[%.0f, %.2f]\n", EP_PD_Ref_Array[i][0], EP_PD_Ref_Array[i][1], PD_Ref_Array[i][0], PD_Ref_Array[i][1]);
    }
  }

  // Show eeprom values
  if (false)
  {
    for (int i = 0; i < 1000; i = i + 8)
    {
      eepromString = ReadInfoEEPROM(i, 8); // Reading EEPROM(int start_position, int data_length)
      MSGOutput("EEPROM(" + String(i) + ") - " + eepromString);
    }
  }

  // Show properties
  if (true)
  {
    eepromString = ReadInfoEEPROM(EP_PD_Ref, 8); // Reading EEPROM(int start_position, int data_length)
    ref_Dac = isNumberic(eepromString) ? eepromString.toDouble() : ref_Dac;
    ref_IL = ILConverter(ref_Dac);
    MSGOutput("Ref IL: " + String(ref_IL));

    ID = ReadInfoEEPROM(EP_Board_ID, 8);
    MSGOutput("Board ID: " + ReadInfoEEPROM(8, 8));

    Station_ID = ReadInfoEEPROM(EP_Station_ID, 8);
    MSGOutput("Station ID: " + ReadInfoEEPROM(16, 8));
    sendmsg_server.client_name = Station_ID;

    eepromString = ReadInfoEEPROM(EP_Station_Type, 8);
    Station_Type = isNumberic(eepromString) ? eepromString.toInt() : Station_Type;
    MSGOutput("Station_Type: " + String(Station_Type));

    eepromString = ReadInfoEEPROM(EP_X_backlash, 8);
    X_backlash = isNumberic(eepromString) ? eepromString.toInt() : X_backlash;
    MSGOutput("X_backlash: " + String(X_backlash));

    eepromString = ReadInfoEEPROM(EP_Y_backlash, 8);
    Y_backlash = isNumberic(eepromString) ? eepromString.toInt() : Y_backlash;
    MSGOutput("Y_backlash: " + String(Y_backlash));

    eepromString = ReadInfoEEPROM(EP_Z_backlash, 8);
    Z_backlash = isNumberic(eepromString) ? eepromString.toInt() : Z_backlash;
    MSGOutput("Z_backlash: " + String(Z_backlash));

    eepromString = ReadInfoEEPROM(EP_delayBetweenStep_X, 8);
    delayBetweenStep_X = isNumberic(eepromString) ? eepromString.toInt() : delayBetweenStep_X;
    MSGOutput("delayBetweenStep_X: " + String(delayBetweenStep_X));

    eepromString = ReadInfoEEPROM(EP_delayBetweenStep_Y, 8);
    delayBetweenStep_Y = isNumberic(eepromString) ? eepromString.toInt() : delayBetweenStep_Y;
    MSGOutput("delayBetweenStep_Y: " + String(delayBetweenStep_Y));

    eepromString = ReadInfoEEPROM(EP_delayBetweenStep_Z, 8);
    delayBetweenStep_Z = isNumberic(eepromString) ? eepromString.toInt() : delayBetweenStep_Z;
    MSGOutput("delayBetweenStep_Z: " + String(delayBetweenStep_Z));

    eepromString = ReadInfoEEPROM(EP_Target_IL, 8);
    Target_IL = isNumberic(eepromString) ? eepromString.toDouble() : Target_IL;
    MSGOutput("Target IL: " + String(Target_IL));

    eepromString = ReadInfoEEPROM(EP_FW_Version, 8);
    MSGOutput("FW Ver: " + eepromString);

    // eepromString = ReadInfoEEPROM(EP_AA_ScanFinal_Scan_Delay_X_A, 8);
    // AA_ScanFinal_Scan_Delay_X_A = isNumberic(eepromString) ? eepromString.toInt() : AA_ScanFinal_Scan_Delay_X_A;
    // MSGOutput("AA_ScanFinal_Scan_Delay_X_A: " + String(AA_ScanFinal_Scan_Delay_X_A));

    eepromString = ReadInfoEEPROM(EP_Motor_DIR_X, 8);
    IsDirtReverse_X = isNumberic(eepromString) ? eepromString.toInt() : IsDirtReverse_X;
    MSGOutput("IsDirtReverse_X: " + String(IsDirtReverse_X));

    eepromString = ReadInfoEEPROM(EP_Motor_DIR_Y, 8);
    IsDirtReverse_Y = isNumberic(eepromString) ? eepromString.toInt() : IsDirtReverse_Y;
    MSGOutput("IsDirtReverse_Y: " + String(IsDirtReverse_Y));

    eepromString = ReadInfoEEPROM(EP_Motor_DIR_Z, 8);
    IsDirtReverse_Z = isNumberic(eepromString) ? eepromString.toInt() : IsDirtReverse_Z;
    MSGOutput("IsDirtReverse_Y: " + String(IsDirtReverse_Z));

    eepromString = ReadInfoEEPROM(EP_Encoder_Motor_Step_X, 8);
    Encoder_Motor_Step_X = isNumberic(eepromString) ? eepromString.toInt() : Encoder_Motor_Step_X;
    MSGOutput("Encoder_Motor_Step_X: " + String(Encoder_Motor_Step_X));

    eepromString = ReadInfoEEPROM(EP_Encoder_Motor_Step_Y, 8);
    Encoder_Motor_Step_Y = isNumberic(eepromString) ? eepromString.toInt() : Encoder_Motor_Step_Y;
    MSGOutput("Encoder_Motor_Step_Y: " + String(Encoder_Motor_Step_Y));

    eepromString = ReadInfoEEPROM(EP_Encoder_Motor_Step_Z, 8);
    Encoder_Motor_Step_Z = isNumberic(eepromString) ? eepromString.toInt() : Encoder_Motor_Step_Z;
    MSGOutput("Encoder_Motor_Step_Z: " + String(Encoder_Motor_Step_Z));

    // Set Encoder DIR
    eepromString = ReadInfoEEPROM(EP_Encoder_DIR_XYZ, 8);
    if (Contains(eepromString, ","))
    {
      String sx = eepromString.substring(0, eepromString.indexOf(','));
      if (isNumberic(sx))
      {
        IsEncDirtReverse_X = sx.toInt();
      }

      eepromString.remove(0, eepromString.indexOf(',') + 1);

      String sy = eepromString.substring(0, eepromString.indexOf(','));
      if (isNumberic(sy))
      {
        IsEncDirtReverse_Y = sy.toInt();
      }

      eepromString.remove(0, eepromString.indexOf(',') + 1);

      String sz = eepromString.substring(0, eepromString.indexOf(','));
      if (isNumberic(sz))
      {
        IsEncDirtReverse_Z = sz.toInt();
      }

      MSGOutput("Set Encoder DIR:" + WR_EEPROM(EP_Encoder_DIR_XYZ,
                                               String(IsEncDirtReverse_X) + "," + String(IsEncDirtReverse_Y) + "," + String(IsEncDirtReverse_Z)));
    }

    eepromString = ReadInfoEEPROM(EP_Get_PD_Points, 8);
    Get_PD_Points = isNumberic(eepromString) ? eepromString.toInt() : Get_PD_Points;
    Get_PD_Points = Get_PD_Points == 0 ? 1 : Get_PD_Points;
    MSGOutput("Get_PD_Points: " + String(Get_PD_Points));

    eepromString = ReadInfoEEPROM(EP_AQ_Scan_Compensation_Steps_Z_A, 8);
    AQ_Scan_Compensation_Steps_Z_A = isNumberic(eepromString) ? eepromString.toInt() : AQ_Scan_Compensation_Steps_Z_A;
    MSGOutput("AQ_Scan_Compensation_Steps_Z_A: " + String(AQ_Scan_Compensation_Steps_Z_A));

    eepromString = ReadInfoEEPROM(EP_AQ_Total_TimeSpan, 8);
    AQ_Total_TimeSpan = isNumberic(eepromString) ? eepromString.toInt() : AQ_Total_TimeSpan;
    MSGOutput("AQ_Total_TimeSpan: " + String(AQ_Total_TimeSpan));

    eepromString = ReadInfoEEPROM(EP_AQ_Scan_Steps_Z_A, 8);
    AQ_Scan_Steps_Z_A = isNumberic(eepromString) ? eepromString.toInt() : AQ_Scan_Steps_Z_A;
    MSGOutput("AQ_Scan_Steps_Z_A: " + String(AQ_Scan_Steps_Z_A));

    eepromString = ReadInfoEEPROM(EP_AQ_Scan_Steps_Z_B, 8);
    AQ_Scan_Steps_Z_B = isNumberic(eepromString) ? eepromString.toInt() : AQ_Scan_Steps_Z_B;
    MSGOutput("AQ_Scan_Steps_Z_B: " + String(AQ_Scan_Steps_Z_B));

    eepromString = ReadInfoEEPROM(EP_AQ_Scan_Steps_Z_C, 8);
    AQ_Scan_Steps_Z_C = isNumberic(eepromString) ? eepromString.toInt() : AQ_Scan_Steps_Z_C;
    MSGOutput("AQ_Scan_Steps_Z_C: " + String(AQ_Scan_Steps_Z_C));

    eepromString = ReadInfoEEPROM(EP_AQ_Scan_Steps_Z_D, 8);
    AQ_Scan_Steps_Z_D = isNumberic(eepromString) ? eepromString.toInt() : AQ_Scan_Steps_Z_D;
    MSGOutput("AQ_Scan_Steps_Z_D: " + String(AQ_Scan_Steps_Z_D));

    eepromString = ReadInfoEEPROM(EP_MotorStepRatio, 8);
    MotorStepRatio = isNumberic(eepromString) ? eepromString.toDouble() : MotorStepRatio;
    MotorStepRatio = MotorStepRatio == 0 ? 1 : MotorStepRatio;
    MSGOutput("MotorStepRatio: " + String(MotorStepRatio));

    eepromString = ReadInfoEEPROM(EP_MotorStepDelayRatio, 8);
    MotorStepDelayRatio = isNumberic(eepromString) ? eepromString.toDouble() : MotorStepDelayRatio;
    MotorStepDelayRatio = MotorStepDelayRatio == 0 ? 1 : MotorStepDelayRatio;
    MSGOutput("MotorStepDelayRatio: " + String(MotorStepDelayRatio));

    eepromString = ReadInfoEEPROM(EP_FS_Count_X, 8);
    FS_Count_X = isNumberic(eepromString) ? eepromString.toInt() : FS_Count_X;
    MSGOutput("FS_Count_X: " + String(FS_Count_X));

    eepromString = ReadInfoEEPROM(EP_FS_Steps_X, 8);
    FS_Steps_X = isNumberic(eepromString) ? eepromString.toInt() : FS_Steps_X;
    MSGOutput("FS_Steps_X: " + String(FS_Steps_X));

    eepromString = ReadInfoEEPROM(EP_FS_Stable_X, 8);
    FS_Stable_X = isNumberic(eepromString) ? eepromString.toInt() : FS_Stable_X;
    MSGOutput("FS_Stable_X: " + String(FS_Stable_X));

    eepromString = ReadInfoEEPROM(EP_FS_DelaySteps_X, 8);
    FS_DelaySteps_X = isNumberic(eepromString) ? eepromString.toInt() : FS_DelaySteps_X;
    MSGOutput("FS_DelaySteps_X: " + String(FS_DelaySteps_X));

    eepromString = ReadInfoEEPROM(EP_FS_Avg_X, 8);
    FS_Avg_X = isNumberic(eepromString) ? eepromString.toInt() : FS_Avg_X;
    MSGOutput("FS_Avg_X: " + String(FS_Avg_X));

    eepromString = ReadInfoEEPROM(EP_FS_Count_Y, 8);
    FS_Count_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_Count_Y;
    MSGOutput("FS_Count_Y: " + String(FS_Count_Y));

    eepromString = ReadInfoEEPROM(EP_FS_Steps_Y, 8);
    FS_Steps_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_Steps_Y;
    MSGOutput("FS_Steps_Y: " + String(FS_Steps_Y));

    eepromString = ReadInfoEEPROM(EP_FS_Stable_Y, 8);
    FS_Stable_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_Stable_Y;
    MSGOutput("FS_Stable_Y: " + String(FS_Stable_Y));

    eepromString = ReadInfoEEPROM(EP_FS_DelaySteps_Y, 8);
    FS_DelaySteps_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_DelaySteps_Y;
    MSGOutput("FS_DelaySteps_Y: " + String(FS_DelaySteps_Y));

    eepromString = ReadInfoEEPROM(EP_FS_Avg_Y, 8);
    FS_Avg_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_Avg_Y;
    MSGOutput("FS_Avg_Y: " + String(FS_Avg_Y));

    eepromString = ReadInfoEEPROM(EP_FS_Count_Z, 8);
    FS_Count_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_Count_Z;
    MSGOutput("FS_Count_Z: " + String(FS_Count_Z));

    eepromString = ReadInfoEEPROM(EP_FS_Steps_Z, 8);
    FS_Steps_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_Steps_Z;
    MSGOutput("FS_Steps_Z: " + String(FS_Steps_Z));

    eepromString = ReadInfoEEPROM(EP_FS_Stable_Z, 8);
    FS_Stable_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_Stable_Z;
    MSGOutput("FS_Stable_Z: " + String(FS_Stable_Z));

    eepromString = ReadInfoEEPROM(EP_FS_DelaySteps_Z, 8);
    FS_DelaySteps_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_DelaySteps_Z;
    MSGOutput("FS_DelaySteps_Z: " + String(FS_DelaySteps_Z));

    eepromString = ReadInfoEEPROM(EP_FS_Avg_Z, 8);
    FS_Avg_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_Avg_Z;
    MSGOutput("FS_Avg_Z: " + String(FS_Avg_Z));

    eepromString = ReadInfoEEPROM(EP_FS_Trips_X, 8);
    FS_Trips_X = isNumberic(eepromString) ? eepromString.toInt() : FS_Trips_X;
    MSGOutput("FS_Trips_X: " + String(FS_Trips_X));

    eepromString = ReadInfoEEPROM(EP_FS_Trips_Y, 8);
    FS_Trips_Y = isNumberic(eepromString) ? eepromString.toInt() : FS_Trips_Y;
    MSGOutput("FS_Trips_Y: " + String(FS_Trips_Y));

    eepromString = ReadInfoEEPROM(EP_FS_Trips_Z, 8);
    FS_Trips_Z = isNumberic(eepromString) ? eepromString.toInt() : FS_Trips_Z;
    MSGOutput("FS_Trips_Z: " + String(FS_Trips_Z));

    eepromString = ReadInfoEEPROM(EP_FS_GradientTarget_X, 8);
    FS_GradientTarget_X = isNumberic(eepromString) ? eepromString.toDouble() : FS_GradientTarget_X;
    MSGOutput("FS_GradientTarget_X: " + String(FS_GradientTarget_X));

    eepromString = ReadInfoEEPROM(EP_FS_GradientTarget_Y, 8);
    FS_GradientTarget_Y = isNumberic(eepromString) ? eepromString.toDouble() : FS_GradientTarget_Y;
    MSGOutput("FS_GradientTarget_Y: " + String(FS_GradientTarget_Y));

    eepromString = ReadInfoEEPROM(EP_FS_GradientTarget_Z, 8);
    FS_GradientTarget_Z = isNumberic(eepromString) ? eepromString.toDouble() : FS_GradientTarget_Z;
    MSGOutput("FS_GradientTarget_Z: " + String(FS_GradientTarget_Z));

    eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Steps_Z_A, 8);
    AA_ScanRough_Feed_Steps_Z_A = isNumberic(eepromString) ? eepromString.toInt() : AA_ScanRough_Feed_Steps_Z_A;
    MSGOutput("AA_ScanRough_Feed_Steps_Z_A: " + String(AA_ScanRough_Feed_Steps_Z_A));

    eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Steps_Z_B, 8);
    AA_ScanRough_Feed_Steps_Z_B = isNumberic(eepromString) ? eepromString.toInt() : AA_ScanRough_Feed_Steps_Z_B;
    MSGOutput("AA_ScanRough_Feed_Steps_Z_B: " + String(AA_ScanRough_Feed_Steps_Z_B));

    eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Ratio_Z_A, 8);
    AA_ScanRough_Feed_Ratio_Z_A = isNumberic(eepromString) ? eepromString.toDouble() : AA_ScanRough_Feed_Ratio_Z_A;
    MSGOutput("AA_ScanRough_Feed_Ratio_Z_A: " + String(AA_ScanRough_Feed_Ratio_Z_A));

    eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Ratio_Z_B, 8);
    AA_ScanRough_Feed_Ratio_Z_B = isNumberic(eepromString) ? eepromString.toDouble() : AA_ScanRough_Feed_Ratio_Z_B;
    MSGOutput("AA_ScanRough_Feed_Ratio_Z_B: " + String(AA_ScanRough_Feed_Ratio_Z_B));

    eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Ratio_Z_C, 8);
    AA_ScanRough_Feed_Ratio_Z_C = isNumberic(eepromString) ? eepromString.toDouble() : AA_ScanRough_Feed_Ratio_Z_C;
    MSGOutput("AA_ScanRough_Feed_Ratio_Z_C: " + String(AA_ScanRough_Feed_Ratio_Z_C));

    eepromString = ReadInfoEEPROM(EP_AA_ScanRough_Feed_Ratio_Z_D, 8);
    AA_ScanRough_Feed_Ratio_Z_D = isNumberic(eepromString) ? eepromString.toDouble() : AA_ScanRough_Feed_Ratio_Z_D;
    MSGOutput("AA_ScanRough_Feed_Ratio_Z_D: " + String(AA_ScanRough_Feed_Ratio_Z_D));
  }

#pragma endregion

  isWiFiConnected = true;
  isCheckingServer = true;
  DataSent_Server("ID?");
  Serial.println("isServerConnected:" + String(isWiFiConnected));
  isCheckingServer = false;

  nowMillis_DataRec = millis();

  disableCore0WDT(); // Disable watch dog
  disableCore1WDT(); // Disable watch dog

  // Station_Type = 8;

  // Check Server is connected
  if (Station_Type == 0)
  {
    // 在core 0啟動 Task_1_sendData(接收遙控盒訊息)
    xTaskCreatePinnedToCore(
        Task_1_sendData, /* 任務實際對應的Function */
        "Task_1",        /* 任務名稱 */
        10000,           /* 堆疊空間 */
        NULL,            /* 無輸入值 */
        12,              /* 優先序0(0為最低) */
        &Task_1,         /* 對應的任務變數位址 */
        0);              /*指定在核心0執行 */
  }
  else if (Station_Type == 3 || Station_Type == 4)
  {
    if (IsDirtReverse_X == 1)
    {
      X_DIR_True = false;
      X_DIR_False = true;
    }

    if (IsDirtReverse_Y == 1)
    {
      Y_DIR_True = false;
      Y_DIR_False = true;
    }

    if (IsDirtReverse_Z == 1)
    {
      Z_DIR_True = false;
      Z_DIR_False = true;
    }

    if (Station_Type == 3)
      GetPower_Mode = 4;
    else if (Station_Type == 4)
      GetPower_Mode = 1;

    MSGOutput("Set GetPowerMode is :" + String(GetPower_Mode));

    isWiFiConnected = false;

    // 在core 0啟動 Task_2_sendData (接收遙控盒訊息)
    xTaskCreatePinnedToCore(
        Task_1_Encoder, /* 任務實際對應的Function */
        "Task_1",       /* 任務名稱 */
        10000,          /* 堆疊空間 */
        NULL,           /* 無輸入值 */
        12,             /* 優先序0(0為最低) */
        &Task_1,        /* 對應的任務變數位址 */
        0);             /*指定在核心0執行 */
  }

  motorType = Phase_5;
  if (motorType == HomeMade)
    isTrip3Jump = false;

  DataOutput(false);
}

unsigned long previousMillis = 0;
const long interval = 10; // default:100

String Txt_SpiralSetting = "";
String Txt_LineScanSetting = "";
int LoopCount = 0;

// int count;

void loop()
{
  // unsigned long currentMillis = millis();
  // currentMillis - previousMillis >= interval;

  // if (currentMillis - previousMillis >= interval)
  {
    // if (isKeepGetIL)
    // {
    //   while (!isStop)
    //   {
    //     SendIL();
    //     delay(1);
    //   }
    // }

    // LoopCount++;
    // previousMillis = currentMillis;

    // count = 0;
    // while (count < 20)
    // { // get 4 readings
    //   if (true)
    //   {
    //     currTime = micros();
    //     readData[count].voltage = ads.getRawResult(); // alternative: getResult_mV for Millivolt
    //     readData[count].cvtTime = currTime - oldTime;
    //     count++; // get next reading
    //     oldTime = currTime;
    //     convReady = false; // reset ready indicator
    //   }
    // }
    // Serial << "Time uSec  Voltage" << endl;
    // for (uint8_t i = 0; i < 20; i++)
    // { // display 4 readings
    //   Serial << readData[i].cvtTime << "  " << readData[i].voltage << endl;
    // }
    // delay(1000);       // delay to change voltage
    // convReady = false; // reset ready indicator
    // oldTime = micros();
    // return;

    // Re-Initialize
    isStop = false;
    if (isStop)
      Serial.println("IsStop : " + String(isStop));

    ButtonSelected = -1;

    // Keyboard Detect
    //  ButtonSelected = KeyValueConverter();
    if (ButtonSelected != -1)
      Motor_Continous_Mode = 1;

    String cmd = "";
    String rsMsg = "";
    if (Serial.available())
    {
      rsMsg = Serial.readString();
      cmd = rsMsg;
      if (cmd != "")
      {
        Motor_Continous_Mode = 2;
        // MSGOutput("GetCmd : " + String(cmd));
      }
    }

    if (cmd_from_contr != "")
    {
      cmd = cmd_from_contr;

      if (cmd_value_from_contr != "")
      {
        int valueContr = cmd_value_from_contr.toDouble();

        if (!isStop)
        {
          Serial.print("msg:" + cmd_from_contr);
          Serial.println(", value:" + cmd_value_from_contr);
        }

        // Motor move cmd from controller
        if (cmd == "BS")
        {
          ButtonSelected = valueContr;
          Motor_Continous_Mode = 3;
        }
      }
    }

    cmd_No = Function_Classification(cmd, ButtonSelected);
    cmd_No = Function_Msg_Classification(cmd, ButtonSelected);

    cmd_No = Function_Excecutation(cmd, cmd_No);

    // isGetPower
    // if (isGetPower && (LoopCount >= 5) && cmd_No <= 100)
    // {
    //   LoopCount = 0;
    //   if (true)
    //   {
    //     double value;

    //     switch (GetPower_Mode)
    //     {
    //     case 1:
    //       value = Cal_PD_Input_IL(Get_PD_Points);
    //       break;

    //     case 2:
    //       value = Cal_PD_Input_Dac(Get_PD_Points);
    //       break;

    //     case 3:
    //       value = Cal_PD_Input_Row_IL(Get_PD_Points);
    //       break;

    //     case 4:
    //       value = Cal_PD_Input_Row_Dac(Get_PD_Points);
    //       break;

    //     default:
    //       break;
    //     }

    //     MSGOutput("PD_Power:" + String(value));       // dB
    //     DataSent_Server("PD_Power:" + String(value)); // Send to Server and check is connected or not !!
    //   }
    // }

    cmd_from_contr = "";
    cmd_value_from_contr = "";
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

String Region, msg;
bool Fine_Scan(int axis, bool Trip2Stop)
{
  MSGOutput("");
  MSGOutput("Fine Scan ");

  if (Station_Type == 3 || Station_Type == 4)
  {
    StopValue = Target_IL;
    isMotorManualCtr = false;
    isWiFiConnected = true;
  }

  MSGOutput("Stop Value: " + String(StopValue));

  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
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
      K_OK = Scan_AllRange_TwoWay(0, FS_Count_X, FS_Steps_X * MotorStepRatio, FS_Stable_X, MotorCC_X, FS_DelaySteps_X, StopValue, FS_Avg_X, FS_Trips_X, "X Fine-Scan,Trip_");
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
      K_OK = Scan_AllRange_TwoWay(2, FS_Count_Z, FS_Steps_Z * MotorStepRatio, FS_Stable_Z, MotorCC_Z, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Fine-Scan,Trip_");
      CMDOutput("%:");

      if (!K_OK)
      {
        CMDOutput("AS");
        Scan_AllRange_TwoWay(2, FS_Count_Z, FS_Steps_Z * MotorStepRatio, FS_Stable_Z, MotorCC_Z, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Re-Scan,Trip_");
        CMDOutput("%:");
      }

      break;
    }

    if (Q_Time != 0)
      MSGOutput("Scan at QTime:" + String(Q_Time));
  }
  // Case 4: all actions should be excuted
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

  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode
  delay(5);
  MSGOutput("Fine Scan End");

  // isLCD = true;
  // LCD_Update_Mode = 0;
  // LCD_PageNow = 100;

  if (Station_Type == 3 || Station_Type == 4)
  {
    isMotorManualCtr = true;
  }

  isWiFiConnected = initial_wifi_status;
  MSGOutput("initial_wifi_status:" + String(initial_wifi_status));
}

//------------------------------------------------------------------------------------------------------------------------------------------

void AutoAlign()
{
  StopValue = Target_IL;

  if (Station_Type == 0)                                                                                                    // CTF
    Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, AA_SpiralRough_Feed_Steps_Z_A * MotorStepRatio, 12 * MotorStepDelayRatio, 150); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
  else
    Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, AA_SpiralRough_Feed_Steps_Z_A / 6 * MotorStepRatio, 12 * MotorStepDelayRatio, 150); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)

  unsigned long time1 = 0, time2 = 0, time3 = 0, time4 = 0, time5 = 0, time6 = 0, time7 = 0;
  double PD_LV1, PD_LV2, PD_LV3, PD_LV4, PD_LV5, PD_LV6;
  double PD_Now = 0;
  time1 = millis();
  MSGOutput(" ");
  CMDOutput("AA"); // Auto Align

  bool Init_WifiStatus = isWiFiConnected;
  isWiFiConnected = false;

  MSGOutput(" ");
  CMDOutput("AS"); // Align Start

#pragma region - Spiral 1

  double ini_X = Get_Position(0);
  double ini_Y = Get_Position(1);

  MSGOutput("... Spiral ...");

  // Spiral - Rough - 1
  int matrix_level = 10;
  CMDOutput("^X");
  CMDOutput("R:" + String(M_Level * 2 + 1));
  CMDOutput("C:" + String(M_Level * 2 + 1));

  delayBetweenStep = 10 * MotorStepDelayRatio;                      // default:15
  MinMotroStep = AA_SpiralRough_Spiral_Steps_XY_A * MotorStepRatio; // 2000

  Region = "Sprial(Rough)";
  AutoAlign_Spiral(matrix_level, -35, 0); // Input : (Sprial Level, Threshold, stable)  Threshold:42
  CMDOutput("X^");

  delay(250);
  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  DataOutput(PD_Now);

  if (PD_Now < -42)
  {
    MSGOutput("IL (" + String(PD_Now) + ") < -42 dB ");
    Move_Motor_abs(0, ini_X);
    Move_Motor_abs(1, ini_Y);
    return;
  }

  ini_X = Get_Position(0);
  ini_Y = Get_Position(1);

  matrix_level = 2;
  CMDOutput("^X");
  CMDOutput("R:" + String(M_Level * 2 + 1));
  CMDOutput("C:" + String(M_Level * 2 + 1));

  delayBetweenStep = 10 * MotorStepDelayRatio;                          // default:15
  MinMotroStep = AA_SpiralRough_Spiral_Steps_XY_A / 3 * MotorStepRatio; // 2000

  Region = "Sprial(Rough 2)";
  AutoAlign_Spiral(matrix_level, -25, 0); // Input : (Sprial Level, Threshold, stable)  Threshold:42
  CMDOutput("X^");

  delay(250);
  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  DataOutput(PD_Now);

  if (PD_Now < -40)
  {
    MSGOutput("IL (" + String(PD_Now) + ") < -40 dB ");
    Move_Motor_abs(0, ini_X);
    Move_Motor_abs(1, ini_Y);
    return;
  }

  int Threshold = -21.4; // default: 144
  stableDelay = 0;       // default : 25

  msg = Region + ",Y Scan, Trip_";

  if (PD_Now > -9)
    MinMotroStep = 25;
  else if (PD_Now > -16 && PD_Now <= -9)
    MinMotroStep = 30;
  else if (PD_Now > -22 && PD_Now <= -16)
    MinMotroStep = 40;
  else
    MinMotroStep = 50;
  AutoAlign_Scan_DirectionJudge_V2(Y_Dir, 20, Threshold, MinMotroStep * MotorStepRatio, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, -4.7, msg);

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
    MinMotroStep = 120; // default : 80
  else
    MinMotroStep = 180;
  AutoAlign_Scan_DirectionJudge_V2(X_Dir, 17, Threshold, MinMotroStep * MotorStepRatio, stableDelay, MotorCC_X, delayBetweenStep, Get_PD_Points, -4.7, msg);

  CMDOutput("%:");

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  PD_LV1 = PD_Now;
  time2 = millis();
  MSGOutput("Sprial(Rough) TimeSpan : " + String((time2 - time1) / 1000) + " s");
  MSGOutput(" ");

  // CheckStop();
  if (isStop)
    return;

  MSGOutput(String(PD_LV1));
  MSGOutput(String(isStop));

  // ---------------------------------------------------Spiral Fine Scan-------------------------------------------------
  if (PD_LV1 < -40) // default: 300
  {
    Region = "Sprial(Fine)";
    int matrix_level = 10;
    CMDOutput("^X");
    CMDOutput("R:" + String(M_Level * 2 + 1));
    CMDOutput("C:" + String(M_Level * 2 + 1));

    delayBetweenStep = 25 * MotorStepDelayRatio;
    MinMotroStep = AA_SpiralFine_Spiral_Steps_XY_A * MotorStepRatio; // 1500

    AutoAlign_Spiral(matrix_level, -37, 0); // Input : (Sprial Level, Threshold, stable)  Threshold:166

    CMDOutput("X^");
  }

#pragma endregion

  PD_LV2 = Cal_PD_Input_IL(Get_PD_Points);
  time3 = millis();

  // CheckStop();
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
    cmd = ""; // Reset command from serial port
  }

#pragma region - Rough Scan

  MSGOutput(" ");
  MSGOutput("... X, Y, Z Scan(Rough) ...");
  Region = "Scan(Rough)";
  double PD_Before = 0;
  double PD_After = 0;
  delayBetweenStep = 50; // Default 50
  Threshold = 124;
  stableDelay = 10; // Default 25
  int scanPoints = 8;
  // int Target_IL_Rough = -3; // Default : -3
  int delta_X, delta_Y, X_pos_before, Y_pos_before;
  double PDValue_After_Scan = -60;

  // -------------------------------------------------Rough-Scan-----------------------------------------------------
  if (PD_Now < -4 && true)
  {
    MSGOutput("Scan(Rough)");
    for (int i = 0; i < 15; i++)
    {
      // Scan(Rough) : Feed Z Loop
      if (true)
      {
        if (i > 0 && abs(PD_After - PD_Before) < 0.1) // 1.2
        {
          MSGOutput("Break_Loop... :" + String(PD_After) + "," + String(PD_Before));
          return;
        }
        else
          MSGOutput("XYZ_Scan(Rough) Round :" + String(i + 1) + ",After:" + String(PD_After) + ",Before:" + String(PD_Before));

        PD_Before = Cal_PD_Input_IL(Get_PD_Points);

        if (PD_Before > Target_IL_Rough)
          break;

        stableDelay = 100;
        double PD_Z_before = 0;

        CMDOutput("AS");
        CMDOutput(">>Z Feed,Trip_1");

        CMDOutput("StopValue: " + String(Target_IL_Rough));

        PD_Now = Cal_PD_Input_IL(Get_PD_Points);

        DataOutput();
        DataOutput(2, PD_Now); // int xyz, double pdValue
        DataSent_Server("PD Power:" + String(PD_Now));

        for (int r = 0; r < 5; r++)
        {
          // CheckStop();
          if (isStop)
            return;

          PD_Z_before = Cal_PD_Input_IL(Get_PD_Points);
          MSGOutput("PD_Z_before:" + String(PD_Z_before));

          int motorStep = AA_ScanRough_Feed_Steps_Z_A; // default: 10000
          if (PD_Z_before < -38)
          {
            motorStep = AA_ScanRough_Feed_Steps_Z_A;
            Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, motorStep * MotorStepRatio, 20 * MotorStepDelayRatio, stableDelay);
          }
          else
          {
            double ratio_idx = AA_ScanRough_Feed_Ratio_Z_A;
            if (PD_Z_before > -37.5 && PD_Z_before <= -22.7)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_A; // default: 3.2
            else if (PD_Z_before > -22.7 && PD_Z_before <= -18.2)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_B; // default: 2.9
            else if (PD_Z_before > -18.2 && PD_Z_before <= -9)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_C; // default: 2.5
            else if (PD_Z_before > -9)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_D; // default: 1.8

            // default: 0.5
            motorStep = abs(ratio_idx * (588 + (55 * abs(PD_Z_before)))); // Default: 588-55*pd
            if (motorStep < AA_ScanRough_Feed_Steps_Z_B)
              motorStep = AA_ScanRough_Feed_Steps_Z_B; // default: 500

            MSGOutput("ratio_idx: " + String(ratio_idx));

            Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, motorStep * MotorStepRatio, 50 * MotorStepDelayRatio, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
          }

          MSGOutput("motorStep: " + String(motorStep));
          MSGOutput("Z_feed:" + String(motorStep * MotorStepRatio));

          PD_Now = Cal_PD_Input_IL(Get_PD_Points);

          DataOutput(PD_Now);
          DataOutput(2, PD_Now); // int xyz, double pdValue
          DataSent_Server("PD Power:" + String(PD_Now));

          if (PD_Now > Target_IL_Rough)
          {
            MSGOutput("Over_Stop_Value");
            break;
          }

          if (PD_Now <= PD_Z_before || (PD_Z_before - PD_Now) > 30 || (abs(PD_Z_before - PD_Now) <= 0.5 && PD_Now > -10))
          {
            MSGOutput("Z_feed_break,Now:" + String(PD_Now) + ",Before:" + String(PD_Z_before) + ",Z:" + String(Pos_Now.Z));
            MSGOutput(" ");
            break;
          }
          else
            MSGOutput("Z_feed,Now:" + String(PD_Now) + ",Before:" + String(PD_Z_before) + ",Z:" + String(Pos_Now.Z));
        }

        CMDOutput("%:");
      }

      Threshold = 120;
      scanPoints = 11;
      stableDelay = 25; // default: 25

      // Scan(Rough) : Spiral, if IL<-54
      if (PD_Now < -40)
      {
        MSGOutput("Spiral:z_feed_region");

        CMDOutput("^X");
        CMDOutput("R:" + String(M_Level * 2 + 1));
        CMDOutput("C:" + String(M_Level * 2 + 1));

        MinMotroStep = 300; // 350
        if (!AutoAlign_Spiral(11, -36.4, 0))
        {
          CMDOutput("X^");
          MSGOutput("Spiral:Target_IL_not_found");
          MSGOutput(" ");

          return;
        }
        CMDOutput("X^");

        MSGOutput(" ");

        // CheckStop();
        if (isStop)
          return;

        stableDelay = 0;

        Region = "Scan(Rough)(1)";

        msg = Region + ",Y_Scan" + ",Trip_";

        AutoAlign_Scan_DirectionJudge_V2(Y_Dir, 20, Threshold, 150 * MotorStepRatio, stableDelay * MotorStepDelayRatio, MotorCC_Y, delayBetweenStep, Get_PD_Points, 279, msg);

        CMDOutput("%:");

        msg = Region + ",X_Scan" + ",Trip_";

        AutoAlign_Scan_DirectionJudge_V2(X_Dir, 25, Threshold, 150 * MotorStepRatio, stableDelay * MotorStepDelayRatio, MotorCC_X, delayBetweenStep, Get_PD_Points, 279, msg);

        CMDOutput("%:");

        PD_Now = Cal_PD_Input_IL(Get_PD_Points);
        if (PD_Now < -54)
          return;

        double b, a;
        int rd = 0;
        for (int s = 0; s < rd; s++)
        {
          b = Cal_PD_Input_IL(Get_PD_Points);

          // CheckStop();
          if (isStop)
            return;

          Region = "Scan(Rough)(2)";

          //(int XYZ, int count, int Threshold, int motorStep, int stableDelay,
          // bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg)

          msg = Region + ",Y_Scan" + ",Trip_";
          AutoAlign_Scan_DirectionJudge_V2(Y_Dir, 15, Threshold, 80 * MotorStepRatio, stableDelay * MotorStepDelayRatio, MotorCC_Y, delayBetweenStep, Get_PD_Points, Target_IL_Rough, msg);
          CMDOutput("%:");

          msg = Region + ",X_Scan" + ",Trip_";
          AutoAlign_Scan_DirectionJudge_V2(X_Dir, 13, Threshold, 100 * MotorStepRatio, stableDelay * MotorStepDelayRatio, MotorCC_X, delayBetweenStep, Get_PD_Points, Target_IL_Rough, msg);
          CMDOutput("%:");

          a = Cal_PD_Input_IL(Get_PD_Points);
          if (abs(a - b) < 5)
            break;
        }
      }

      PD_After = Cal_PD_Input_IL(Get_PD_Points);
      if (PD_After > Target_IL_Rough)
      {
        MSGOutput("Over_Stop_Value");
        break;
      }

      // ----------------------------------------- Scan(Rough) : XY Scan ------------------------------------------------
      if (PD_Now < Target_IL_Rough)
      {
        delayBetweenStep = 20; // 80
        MinMotroStep = 350;
        stableDelay = 20; // default:AA_ScanFinal_Scan_Delay_X_A

        // CheckStop();
        if (isStop)
          return;

        struct_AlignResult alignRst;

        Region = "Scan(Rough)(3)";

        msg = Region + ",Y_Scan" + ",Trip_";
        if (PD_After > -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_A; // default:25
        else if (PD_After > -16 && PD_After <= -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_B; // default:30
        else if (PD_After > -22.7 && PD_After <= -16)
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_C; // default:40
        else
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_D; // default:70

        if (PD_After > -5)
        {
          stableDelay = 50;
        }

        CMDOutput("AS");
        MSGOutput("Gap:" + String(MinMotroStep * MotorStepRatio));
        alignRst = AutoAlign_Scan_DirectionJudge_V2(Y_Dir, 20, Threshold, MinMotroStep * MotorStepRatio,
                                                    stableDelay * MotorStepDelayRatio,
                                                    MotorCC_Y, delayBetweenStep, Get_PD_Points, Target_IL_Rough, msg);
        CMDOutput("%:");

        // if (!alignRst.IsResultGood)
        //   return; // For Test

        if (alignRst.Best_IL > Target_IL_Rough)
        {
          Serial.println("Better_than_stopValue:" + String(PD_After) + ",stopvalue:" + String(Target_IL_Rough));
          break;
        }

        PD_After = alignRst.Result_IL;

        msg = Region + ",X_Scan" + ",Trip_";
        if (PD_After > -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_A; // default:25
        else if (PD_After > -16 && PD_After <= -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_B; // default:30
        else if (PD_After > -22.7 && PD_After <= -16)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_C; // default:80
        else if (PD_After > -30 && PD_After <= -22.7)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_D; // default:100
        else
          MinMotroStep = AA_ScanRough_Scan_Steps_X_E; // default:120

        if (PD_After > -5)
        {
          stableDelay = 50;
        }

        CMDOutput("AS");
        MSGOutput("Gap:" + String(MinMotroStep * MotorStepRatio));
        alignRst = AutoAlign_Scan_DirectionJudge_V2(X_Dir, 20, Threshold, MinMotroStep * MotorStepRatio,
                                                    stableDelay * MotorStepDelayRatio, MotorCC_X,
                                                    delayBetweenStep, Get_PD_Points, Target_IL_Rough, msg);

        // if (!alignRst.IsResultGood)
        //   return; // For Test

        PD_After = alignRst.Result_IL;

        MSGOutput("X:" + String(Get_Position(0)));
        MSGOutput("Y:" + String(Get_Position(1)));
        MSGOutput("Z:" + String(Get_Position(2)));

        CMDOutput("%:");
      }
      else
      {
        MSGOutput("Over_Stop_Value");
        break;
      }

      PD_After = Cal_PD_Input_IL(Get_PD_Points);

      DataOutput(PD_After);

      // Scan(Rough) : stop condition
      {
        // if (PD_After > -2) // default:-2
        // {
        //   MSGOutput("Scan(Rough)(A)-IL>-2:" + String(PD_After) + "," + String(PDValue_After_Scan));
        //   break;
        // }

        if (PD_After > Target_IL_Rough)
        {
          Serial.println("Better_than_stopValue:" + String(PD_After) + ",stopvalue:" + String(Target_IL_Rough));
          break;
        }

        if (abs(PD_After - PDValue_After_Scan) < 0.1 && PD_After > -10) // default:16
        {
          MSGOutput("Scan(Rough)(A)-Pass_best_Z_position");
          MSGOutput(String(PD_After) + "," + String(PDValue_After_Scan));
          break;
        }

        if (PD_After < PDValue_After_Scan && PD_After > -8) // default:10
        {
          MSGOutput("Scan(Rough)(B)-Pass_best_Z_position:" + String(PD_After) + "," + String(PDValue_After_Scan));
          break;
        }

        if (PD_After < -42) // default:16
        {
          MSGOutput("Scan(Rough)(C)-High_loss");
          Serial.println(String(PD_After) + "," + String(PDValue_After_Scan));
          break;
        }

        if (Serial.available())
        {
          String cmd = Serial.readStringUntil('\n');
          cmd_No = Function_Excecutation(cmd, Function_Classification(cmd, -1));
          cmd = ""; // Reset command from serial port
        }

        PDValue_After_Scan = PD_After;

        if (i == 1)
        {
          X_pos_before = Pos_Now.X;
          Y_pos_before = Pos_Now.Y;
        }
        else if (i > 1)
        {
          delta_X = Pos_Now.X - X_pos_before;
          delta_Y = Pos_Now.Y - Y_pos_before;
          X_pos_before = Pos_Now.X;
          Y_pos_before = Pos_Now.Y;
          Serial.println("------- Delta_X:" + String(delta_X) + ",Delta_Y:" + String(delta_Y));
        }

        // if (PD_After < -54)
        // {
        //   Serial.println("Rough_Scan:High_loss");
        //   return;
        // }
      }
    }
  }

#pragma endregion

  PD_LV3 = Cal_PD_Input_IL(Get_PD_Points);
  time4 = millis();

  // CheckStop();
  if (isStop)
    return;

#pragma region Fine Scan

  // Scan(Fine)
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
        cmd = ""; // Reset command from serial port
      }
      if (isStop)
        return;

      Fine_Scan(Y_Dir, false);

      DataOutput(false);

      if (Serial.available())
      {
        String cmd = Serial.readStringUntil('\n');
        cmd_No = Function_Excecutation(cmd, Function_Classification(cmd, -1));
        cmd = ""; // Reset command from serial port
      }
      if (isStop)
        return;

      Fine_Scan(X_Dir, false);

      DataOutput(false);

      PD_After = Cal_PD_Input_IL(2 * Get_PD_Points);

      if (PD_After <= PD_Before || abs(PD_Before - PD_After) <= 0.12)
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

  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode

  if (isStop)
    return;

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
#pragma region Initialization
  CMDOutput("ST" + String(MinMotroStep));
  Serial.println("StopValue:" + String(StopValue));
  Serial.println("stableDelay:" + String(stableDelay));
  MSGOutput("Spiral Step:" + String(MinMotroStep));

  MSGOutput("Position:" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z));

  double ts = 0;
  unsigned long timer_1 = 0, timer_2 = 0;
  timer_1 = millis();

  double PD_BestIL = -100;
  int PD_BestIL_Position[2];
  int PD_Best_Pos_Abs[2];

  double SpiralStop_Threshold = StopValue; // Default : 198
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
  PD_Best_Pos_Abs[0] = Pos_Now.X;
  PD_Best_Pos_Abs[1] = Pos_Now.Y;
  Serial.println("Inital:(" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + ") = " + String(PD_BestIL));

#pragma endregion

  if (PD_Now >= SpiralStop_Threshold)
  {
    Serial.println("Over Threshold: " + String(PD_Now) + ", Threshold: " + String(SpiralStop_Threshold));
    return true;
  }

#pragma region Layer Loops

  for (int n = 1; abs(n) < (M + 1); n++)
  {
    CheckStop();
    if (isStop)
      return false;

    CMDOutput("ML", false);
    Serial.println("Matrix Layers: " + String(n));

    if (n > m)
      m++;

    y--;
    MotorCC_A = false;
    Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC_A, MinMotroStep, delayBetweenStep, stableDelay, false);

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now); //[0,-1]

    if (PD_Now > PD_BestIL)
    {
      PD_BestIL = PD_Now;
      PD_BestIL_Position[0] = x;
      PD_BestIL_Position[1] = y;
      PD_Best_Pos_Abs[0] = Pos_Now.X;
      PD_Best_Pos_Abs[1] = Pos_Now.Y;

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

    CheckStop();
    if (isStop)
      return false;

    // To Left

    MotorCC_A = false;

    while (x >= (-n))
    {
      Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC_A, MinMotroStep, delayBetweenStep, stableDelay, false);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = Pos_Now.X;
        PD_Best_Pos_Abs[1] = Pos_Now.Y;

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

    CheckStop();
    if (isStop)
      return false;

    // Up

    MotorCC_A = true;

    int nM = n;
    while (y <= (nM))
    {
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC_A, MinMotroStep, delayBetweenStep, stableDelay, false);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = Pos_Now.X;
        PD_Best_Pos_Abs[1] = Pos_Now.Y;

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

    CheckStop();
    if (isStop)
      return false;

    // To Right

    MotorCC_A = true;

    while (x <= (n))
    {
      Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC_A, MinMotroStep, delayBetweenStep, stableDelay, false);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = Pos_Now.X;
        PD_Best_Pos_Abs[1] = Pos_Now.Y;

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

    CheckStop();
    if (isStop)
      return false;

    // Down

    MotorCC_A = false;

    while (y >= (-n))
    {
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC_A, MinMotroStep, delayBetweenStep, stableDelay, false);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);
      // Serial.println("Position: " + String(Pos_Now.X) + ", " + String(Pos_Now.Y) + ", " + String(Pos_Now.Z));

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = Pos_Now.X;
        PD_Best_Pos_Abs[1] = Pos_Now.Y;

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

#pragma endregion

  CMDOutput("ML");
  Serial.println("Matrix Layers: Max");

  CheckStop();
  if (isStop)
    return false;

  int delta_X = 0, delta_Y = 0;

  if (!sprial_JumpToBest)
  {
    PD_BestIL_Position[0] = 0;
    PD_BestIL_Position[1] = 0; // Jump to (0,0)
  }

#pragma region Jump to best IL position

  if (PD_BestIL_Position[0] <= 2 * M && PD_BestIL_Position[1] <= 2 * M)
  {
    Move_Motor_abs(0, PD_Best_Pos_Abs[0]);
    Move_Motor_abs(1, PD_Best_Pos_Abs[1]);

    delay(200);

    double finalIL = Cal_PD_Input_IL(Get_PD_Points);
    Serial.println("Final IL : " + String(finalIL));

    Serial.println("Position: " + String(Pos_Now.X) + ", " + String(Pos_Now.Y) + ", " + String(Pos_Now.Z));
    // DataOutput(finalIL);

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

#pragma endregion

  return false;
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
    backlash = 60;    // default: 95
    Modify_Ratio = 1; // default:1.6
    delay(stableDelay);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = 40;    // default: 85
    Modify_Ratio = 1; // default:1.3
    delay(stableDelay);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = 500; // default: 1100
    Modify_Ratio = 1.5;
    delay(stableDelay);
    break;
  }

  MotorCC_A = dir;
  // digitalWrite(DIR_Pin, MotorCC_A);
  // delay(5);

  step(STP_Pin, (backlash + 20), delayBetweenStep, MotorCC_A); // Backlash about 40 pulse
  delay(stableDelay * 2);

  // Reverse
  MotorCC_A = !MotorCC_A;
  // digitalWrite(DIR_Pin, MotorCC_A);
  // delay(10);
  step(STP_Pin, (backlash * Modify_Ratio + 20), delayBetweenStep, MotorCC_A); // Backlash about 40 pulse
  delay(stableDelay * 2);

  switch (XYZ)
  {
  case 0:
    MotorCC_X = MotorCC_A;
    break;
  case 1:
    MotorCC_Y = MotorCC_A;
    break;
  case 2:
    MotorCC_Z = MotorCC_A;
    break;
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

long Curfit(double x1[], double y1[], int dataCount)
{
  // Serial.println("Curfitting");
  char buf[4];
  int xpower = 0;
  int order = 2;

  double x[dataCount]; // idex * step = real steps
  double y[dataCount]; // fill this with your sensor data

  double center_x = x1[0];
  for (int i = 0; i < dataCount; i++)
  {
    x[i] = x1[i] - center_x;
    y[i] = y1[i];
  }

  int step_distance = abs(x[1] - x[0]);

  double coeffs[order + 1];

  int ret = fitCurve(order, sizeof(y) / sizeof(double), x, y, sizeof(coeffs) / sizeof(double), coeffs);

  if (ret == 0)
  { // Returned value is 0 if no error
    uint8_t c = 'a';

    long result_x = (-1 * coeffs[1]) / (2 * coeffs[0]);

    return result_x + center_x;
  }
  else
  {
    return x1[1];
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

long Curfit_2(double x1[], double y1[], int dataCount)
{
  // Serial.println("Curfitting");
  char buf[4];
  int xpower = 0;
  int order = 2;

  double x[dataCount]; // idex * step = real steps
  double y[dataCount]; // fill this with your sensor data

  for (int i = 0; i < dataCount; i++)
  {
    x[i] = x1[i];
    y[i] = y1[i];
  }

  double coeffs[order + 1];

  int ret = fitCurve(order, sizeof(y) / sizeof(double), x, y, sizeof(coeffs) / sizeof(double), coeffs);

  if (ret == 0)
  { // Returned value is 0 if no error
    long result_x = (-1 * coeffs[1]) / (2 * coeffs[0]);
    // Serial.println("Curfit X is : " + String(result_x));

    return result_x;
  }
  else
  {
    return x1[1];
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

struct_AlignResult AlignResult_V2;

struct_AlignResult AutoAlign_Scan_DirectionJudge_V2(int axisDir, int count, int Threshold, int motorStep, int stableDelay,
                                                    bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg)
{
  int DIR_Pin = 0;
  int STP_Pin = 0;
  MotorCC_A = Direction; // direction first
  int backlash = 40, trip = 1;
  bool isReverse = false;
  double ts = 0;
  unsigned long timer_1 = 0, timer_2 = 0;
  timer_1 = millis();
  isWiFiConnected = false;

  MSGOutput("Scan V2");
  Serial.println("stableDelay: " + String(stableDelay));
  Serial.println("StopPDValue: " + String(StopPDValue));
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

  CMDOutput(">>" + msg + String(trip)); // Trip_1------------------------------------------------------------

  double PD_Best = -64,
         PD_Trip2_Best = -64,
         PD_Now = -64;

  double PD_Value[count * 2];
  double PD_Rvrs_Value[count];
  int PD_Best_Pos = 0;
  int PD_Best_Pos_Trip2 = 0;

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points * 5);
  PD_initial = Cal_PD_Input_IL(Get_PD_Points * 5);
  DataOutput(axisDir, PD_initial); // int xyz, double pdValue

  if (PD_initial >= StopPDValue)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    PD_Best = PD_initial;

    AlignResult_V2.Result_IL = PD_Best;
    AlignResult_V2.Best_IL = PD_Best;
    AlignResult_V2.IsResultGood = true;
    AlignResult_V2.Result_Pos = Pos_Now;

    return AlignResult_V2;
  }

  Move_Motor(DIR_Pin, STP_Pin, MotorCC_A, motorStep * 4, delayBetweenStep, 200, true);

  PD_Now = Cal_PD_Input_IL(Get_PD_Points * 2);
  DataOutput(axisDir, PD_Now); // int xyz, double pdValue

  Serial.println("Initial: " + String(PD_initial) + ", After:" + String(PD_Now));

  if (PD_Now >= StopPDValue)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    PD_Best = PD_Now;

    AlignResult_V2.Result_IL = PD_Best;
    AlignResult_V2.Best_IL = PD_Best;
    AlignResult_V2.IsResultGood = true;
    AlignResult_V2.Result_Pos = Pos_Now;

    return AlignResult_V2;
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
    digitalWrite(DIR_Pin, MotorCC_A);
    delay(10);
    MSGOutput("Dir: Forward");

    PD_Value[0] = Cal_PD_Input_IL(Get_PD_Points);
  }
  else
  {
    Dir = false;
    PD_Best = PD_initial;
    BackLash_Reverse(axisDir, MotorCC_A, stableDelay);
    MSGOutput("Dir: Reverse");
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
      {
        AlignResult_V2.Result_IL = PD_Now;
        AlignResult_V2.Best_IL = PD_Best;
        AlignResult_V2.IsResultGood = true;
        AlignResult_V2.Result_Pos = Pos_Now;
        return AlignResult_V2;
      }

      if (i != 0)
      {
        step(STP_Pin, motorStep, delayBetweenStep, MotorCC_A);
        delay(stableDelay);
      }

      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);
      DataOutput(axisDir, PD_Value[i]); // int xyz, double pdValue

      if (i == 0)
      {
        PD_Trip2_Best = PD_Value[i];
        PD_Best_Pos_Trip2 = Get_Position(axisDir);
      }

      if (PD_Value[i] >= StopPDValue) // Condition 1
      {
        Serial.println("Stop Condition 1 : Over StopPDValue");

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        Serial.println("");

        PD_Best = PD_Value[i];

        AlignResult_V2.Result_IL = PD_Value[i];
        AlignResult_V2.Best_IL = PD_Best;
        AlignResult_V2.IsResultGood = true;
        AlignResult_V2.Result_Pos = Pos_Now;
        return AlignResult_V2;
      }

      if (PD_Value[i] > PD_Best)
      {
        PD_Best = PD_Value[i];
        PD_Best_Pos = i;

        if (PD_Best > -5 && stableDelay < 50)
        {
          stableDelay = 50;
          Serial.println("stableDelay: " + String(stableDelay));
        }
      }

      if (PD_Value[i] > PD_Trip2_Best)
      {
        PD_Trip2_Best = PD_Value[i];
        PD_Best_Pos_Trip2 = Get_Position(axisDir);
      }

      if (i >= 2 && PD_Value[i] <= PD_Value[i - 1]) // Condition 2
      {
        trend_count++;
        if (trend_count > 7)
        {
          Serial.println("Stop Condition 2-1 : Pass best IL");
          break;
        }
        else if (Dir && trend_count > 3)
        {
          Serial.println("Stop Condition 2-2 : Pass best IL");
          break;
        }
        else if (trend_count > 3 && i >= 9) // default: i >= 12
        {
          Serial.println("Stop Condition 2-3 : Pass best IL");
          break;
        }
      }

      if (i >= 6 && abs(PD_Value[i] - PD_Value[i - 1]) <= 0.03 && abs(PD_Value[i - 1] - PD_Value[i - 2]) <= 0.03 && abs(PD_Value[i - 2] - PD_Value[i - 3]) <= 0.03 && abs(PD_Value[i - 3] - PD_Value[i - 4]) <= 0.03) // Condition 3
      {
        if (PD_Value[i] < PD_Best)
        {
          Serial.println("Stop Condition 3 : ERROR Status");
          break;
        }
      }

      if (PD_Value[i] < -45) // Condition 5
      {
        Serial.println("Stop Condition 5 : Miss Target");
        break;
      }

      if (i == count - 1 && PD_Value[i] == PD_Best) // 未通過最高點
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
  CMDOutput("~:" + msg + String(trip)); // Trip_3------------------------------------------------------------

  // Jump to best position
  if (true)
  {
    delay(120);
    Move_Motor_abs(axisDir, PD_Best_Pos_Trip2); // Jump to Trip_2 start position
    delay(350);
    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput(axisDir, PD_Now); // int xyz, double pdValue
    MSGOutput("Jump to best position");
  }
  // else if (PD_Now < PD_Best - 0.5)
  // {
  //   if (PD_Best <= 3)
  //   {
  //     Move_Motor_abs(axisDir, Pos_Ini_Trip2); // Jump to Trip_2 start position
  //     delay(180);
  //     DataOutput(axisDir, Cal_PD_Input_IL(Get_PD_Points)); // int xyz, double pdValue
  //   }

  //   long pNow = PD_Best_Pos_Trip2;
  //   if (xyz == 0)
  //     pNow = Pos_Now.X;
  //   else if (xyz == 1)
  //     pNow = Pos_Now.Y;
  //   else if (xyz == 2)
  //     pNow = Pos_Now.Z;

  //   if (PD_Best_Pos_Trip2 != pNow)
  //   {
  //     Move_Motor_abs(axisDir, PD_Best_Pos_Trip2); // Jump to Trip_2 start position
  //     delay(180);
  //     DataOutput(axisDir, Cal_PD_Input_IL(Get_PD_Points)); // int xyz, double pdValue
  //     MSGOutput("Back to best position");
  //   }
  // }

  // delay(stableDelay);

  // PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  // if (abs(PD_Now - PD_Best) < 0.4)
  // {
  //   timer_2 = millis();
  //   ts = (timer_2 - timer_1) * 0.001;
  //   CMDOutput("t:" + String(ts, 2));

  //   AlignResult_V2.Result_IL = PD_Now;
  //   AlignResult_V2.IsResultGood = true;
  //   AlignResult_V2.Result_Pos = Pos_Now;

  //   return AlignResult_V2;
  // }

  // timer_2 = millis();
  // ts = (timer_2 - timer_1) * 0.001;
  // Serial.print("TS:");
  // Serial.println(ts, 2);
  // Serial.println(" ");
  // CMDOutput("t:" + String(ts, 2));

  AlignResult_V2.Result_IL = PD_Now;
  AlignResult_V2.Best_IL = PD_Best;
  AlignResult_V2.IsResultGood = true;
  AlignResult_V2.Result_Pos = Pos_Now;

  if (PD_Best > -5 && PD_Now < PD_Best - 1)
  {
    AlignResult_V2.IsResultGood = false;
  }

  return AlignResult_V2;
}

//------------------------------------------------------------------------------------------------------------------------------------------

bool AutoAlign_Scan_DirectionJudge_V3(int XYZ, int count, int motorStep, int stableDelay,
                                      bool Direction, int delayBetweenStep, double StopPDValue, int Get_PD_Points, int Trips, String msg)
{
  int DIR_Pin = 0;
  int STP_Pin = 0;
  int backlash = 40;
  MotorCC_A = Direction; // initial direction
  int trip = 1;
  int dataCount = count + 1;
  int dataCount_ori;
  int indexofBestIL = 0;
  double PD_Value[4 * count + 1];
  long Step_Value[4 * count + 1];
  double Gradient_IL_Step[4 * count + 1];
  int GradientCount = 0;
  double GradientTarget = 0.003; // default: 0.007
  unsigned long timer_1 = 0, timer_2 = 0;
  timer_1 = millis();

  dataCount_ori = dataCount;

  switch (XYZ)
  {
  case X_Dir:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = X_backlash;
    GradientTarget = FS_GradientTarget_X; // 0.003
    delay(5);
    break;
  case Y_Dir:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = Y_backlash;
    GradientTarget = FS_GradientTarget_Y; // 0.002
    delay(5);
    break;
  case Z_Dir:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = Z_backlash;
    GradientTarget = FS_GradientTarget_Z; // 0.003
    delay(5);
    break;
  }

  bool iniWifiStatus = isWiFiConnected;
  if (!is_AutoCuring)
    isWiFiConnected = false;

  MSGOutput("Scan V3");
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

  if (PD_initial >= StopPDValue)
    return true;

  //-------------------------------------------Jump to Trip_1 initial position-------------------------------------
  // digitalWrite(DIR_Pin, MotorCC_A);
  // delay(5);

  // if (true)
  {
    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput();
    DataOutput(XYZ, PD_Now); // int xyz, double pdValue
  }

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
    if (isStop)
      return true;

    if (i == 0)
    {
      PD_Value[i] = PD_Now;
      Step_Value[i] = Get_Position(XYZ);
      continue;
    }

    step(STP_Pin, motorStep, delayBetweenStep, MotorCC_A);
    delay(stableDelay);

    if (i > 1 && PD_Value[i - 1] > -2)
      delay(30);

    if (i > 0 && PD_Value[i - 1] > -2)
      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points * 3); // 2500
    else
      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);

    Step_Value[i] = Get_Position(XYZ);

    if (PD_Value[i] > IL_Best_Trip1)
    {
      indexofBestIL = i;
      IL_Best_Trip1 = PD_Value[i];
      Pos_Best_Trip1 = Get_Position(XYZ);
    }

    // Update Min, Max IL in Scan Process
    if (PD_Value[i] > maxIL_in_FineScan)
      maxIL_in_FineScan = PD_Value[i];
    if (PD_Value[i] < minIL_in_FineScan)
      minIL_in_FineScan = PD_Value[i];

    DataOutput();
    DataOutput(XYZ, PD_Value[i]); // int xyz, double pdValue
    DataSent_Server("PD Power:" + String(PD_Value[i]));

    // Gradient analyze
    if (i > 0)
    {
      // 計算目前點斜率
      Gradient_IL_Step[i - 1] = (PD_Value[i] - PD_Value[i - 1]) / (motorStep / MotorStepRatio);

      if (XYZ == Z_Dir)
      {
        Gradient_IL_Step[i - 1] *= (FS_Steps_Z / FS_Steps_X);
      }

      if (Gradient_IL_Step[i - 1] <= (GradientTarget * -1))
        GradientCount = 0;
      else
        GradientCount++;

      if (i > 5)
      {
        if (PD_Value[i] > -2 && Gradient_IL_Step[i - 1] <= GradientTarget && GradientCount > 3) // default: -1.6
        {
          MSGOutput("Gradient <= GradientTarget : " + String(Gradient_IL_Step[i - 1], 4));

          if (XYZ == Z_Dir && GradientCount > 4)
          {
            MSGOutput("i:" + String(i) + ", Pos_Best_Trip1(Curfit):" + String(Pos_Best_Trip1));
            double x[5];
            double y[5];
            for (int k = i - 4; k <= i; k++)
            {
              x[k - (i - 4)] = Step_Value[k]; // idex * step = real steps
              y[k - (i - 4)] = PD_Value[k];   // fill this with your sensor data
              Serial.println("Point : " + String(x[k - (i - 4)]) + " , " + String(y[k - (i - 4)]));
            }
            Pos_Best_Trip1 = Curfit_2(x, y, 5);
            MSGOutput("Best pos in Trip_1 (curfit) is: " + String(Pos_Best_Trip1));
          }

          DataOutput();

          timer_2 = millis();
          double ts = (timer_2 - timer_1) * 0.001;
          CMDOutput("t:" + String(ts, 2));

          isWiFiConnected = iniWifiStatus;

          return true;
        }
      }
    }

    if (IL_Best_Trip1 >= -2 && PD_Value[i] <= (IL_Best_Trip1 - 1.5) && Trips == 1 && i > 4)
    {
      Serial.println("IL < (IL-1.5): " + String(PD_Value[i]));

      // Curfit
      if (indexofBestIL != 0 && Pos_Best_Trip1 != Get_Position(XYZ))
      {
        double x[3];
        double y[3];
        for (int k = -1; k < 2; k++)
        {
          x[k + 1] = Step_Value[indexofBestIL + k]; // idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];   // fill this with your sensor data
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

    if (Trips == 0 && i > 3)
    {
      if ((PD_Value[i] <= PD_Value[i - 1] || abs(PD_Value[i] - PD_Value[i - 1]) <= 0.02) && PD_Value[i] >= -1.8)
      {
        MSGOutput("Over best IL in trip 1");
        PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);
        MSGOutput("Final IL: " + String(PD_Now));
        timer_2 = millis();
        double ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        DataOutput();
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
        x[k + 1] = Step_Value[indexofBestIL + k]; // idex * step = real steps
        y[k + 1] = PD_Value[indexofBestIL + k];   // fill this with your sensor data
        Serial.println("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
      }
      Pos_Best_Trip1 = Curfit(x, y, 3);
      MSGOutput("Best IL position in Trip_1 is: " + String(Pos_Best_Trip1));
    }
  }

  //------------------------------------Trip_2 ------------------------------------------------------------

  //------------------------------------Trip_3 -------------------------------------------------------

  trip++;
  MSGOutput(" --- Trip 2 --- ");
  MSGOutput("trip: " + String(trip));
  MSGOutput("Trips: " + String(Trips));

  CMDOutput("~:" + msg + String(trip));

  double PD_Best = IL_Best_Trip1;
  int deltaPos = 0;

  // 5-Phase motor
  {
    PD_Best = IL_Best_Trip1;
    Pos_Best_Trip = Pos_Best_Trip1;

    MSGOutput("Jump to best IL position : " + String(Pos_Best_Trip));

    Move_Motor_abs(XYZ, Pos_Best_Trip); // Jump to Trip_2 start position

    delay(250);

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput();
    DataOutput(XYZ, PD_Now); // int xyz, double pdValue
  }

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

int Function_Classification(String cmd, int ButtonSelected)
{
  if (cmd != "" && ButtonSelected < 0)
  {
    cmd.trim();
    // MSGOutput("get_cmd:" + String(cmd));

    String cmdUpper = cmd;
    cmdUpper.toUpperCase();

// Keyboard - Motor Control
#pragma region - Keyboard - Motor Control
    if (cmd == "Xp")
    {
      cmd_No = 105;
      return cmd_No;
    }
    else if (cmd == "Xm")
    {
      cmd_No = 102;
      return cmd_No;
    }
    else if (cmd == "Yp")
    {
      cmd_No = 104;
      return cmd_No;
    }
    else if (cmd == "Ym")
    {
      cmd_No = 106;
      return cmd_No;
    }
    else if (cmd == "Zp")
    {
      cmd_No = 103;
      return cmd_No;
    }
    else if (cmd == "Zm")
    {
      cmd_No = 101;
      return cmd_No;
    }

    // Jog
    else if (Contains(cmd, "Jog_"))
    {
      // cmd.remove(0, 4);
      cmd = ExtractCmd(cmd, "Jog_");

      byte dirPin, stpPin;
      bool dirt;
      int delayNow = delayBetweenStep_Y;

      if (Contains(cmd, "X"))
      {
        dirPin = X_DIR_Pin;
        stpPin = X_STP_Pin;
        delayNow = delayBetweenStep_X;
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
        delayNow = delayBetweenStep_Z;
      }

      cmd.remove(0, 1);

      if (Contains(cmd, "m"))
        dirt = false;
      else if (Contains(cmd, "p"))
        dirt = true;

      cmd.remove(0, 2);

      isCheckStop = true;

      Move_Motor(dirPin, stpPin, dirt, cmd.toDouble(), delayNow, 0, true, 8); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    }

    // Abs
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

      isCheckStop = true;

      Move_Motor_abs(xyz, cmd.toDouble());

      return 0;
    }

    // Abs All
    else if (Contains(cmd, "AbsAll_"))
    {
      cmd.remove(0, 7);

      long travel_x = 0, travel_y = 0, travel_z = 0;

      travel_x = cmd.substring(0, cmd.indexOf('_')).toDouble();

      cmd.remove(0, cmd.indexOf('_') + 1);

      travel_y = cmd.substring(0, cmd.indexOf('_')).toDouble();

      cmd.remove(0, cmd.indexOf('_') + 1);

      travel_z = cmd.substring(0, cmd.indexOf('_')).toDouble();

      isCheckStop = true;

      Move_Motor_abs_all(travel_x, travel_y, travel_z);

      DataOutput(false);
    }

#pragma endregion

    // Set auto-align / auto-curing Parameter
    else if (Contains(cmd, "Set::"))
    {
      int indexOffSet = cmd.indexOf("Set::");

      cmd.remove(0, 5 + indexOffSet);

      String ParaName = cmd.substring(0, cmd.indexOf('='));
      cmd.remove(0, cmd.indexOf('=') + 1);
      cmd.trim();

      Serial.println("ParaName:" + ParaName + ", Value:" + String(cmd.toDouble()));

      if (!isNumberic(cmd))
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

      // else if (ParaName == "AA_ScanFinal_Scan_Delay_X_A")
      // {
      //   AA_ScanFinal_Scan_Delay_X_A = cmd.toInt();
      //   Serial.println("Write EEPROM AA_ScanFinal_Scan_Delay_X_A: " + WR_EEPROM(80, cmd));
      // }

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
      else if (ParaName == "FS_GradientTarget_X")
      {
        FS_GradientTarget_X = cmd.toDouble();
        Serial.println("Write EEPROM FS_GradientTarget_X: " + WR_EEPROM(EP_FS_GradientTarget_X, cmd));
      }
      else if (ParaName == "FS_GradientTarget_Y")
      {
        FS_GradientTarget_Y = cmd.toDouble();
        Serial.println("Write EEPROM FS_GradientTarget_Y: " + WR_EEPROM(EP_FS_GradientTarget_Y, cmd));
      }
      else if (ParaName == "FS_GradientTarget_Z")
      {
        FS_GradientTarget_Z = cmd.toDouble();
        Serial.println("Write EEPROM FS_GradientTarget_Z: " + WR_EEPROM(EP_FS_GradientTarget_Z, cmd));
      }
    }

    // Command No.
    else if (Contains(cmd, "cmd"))
    {
      cmd = ExtractCmd(cmd, "cmd");

      // cmd.remove(0, 3);
      cmd_No = cmd.toInt();
      delay(2);
    }
  }
  else if (ButtonSelected >= 0)
  {
    // Keyboard No. to Cmd Set No.
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

int Function_Msg_Classification(String cmd, int ButtonSelected)
{
  if (cmd != "" && ButtonSelected < 0)
  {
    cmd.trim();

    String cmdUpper = cmd;
    cmdUpper.toUpperCase();

#pragma region - String Command
    //(CScan) Scan Twoway Command
    if (Contains(cmd, "CScan_"))
    {
      cmd = ExtractCmd(cmd, "CScan_");

      Txt_LineScanSetting = cmd;
      Serial.println(Txt_LineScanSetting);

      return 198;
    }

    //(SScan) Spiral Scan Command
    else if (Contains(cmd, "SScan_"))
    {
      cmd = ExtractCmd(cmd, "SScan_");

      Txt_SpiralSetting = cmd;
      Serial.println(Txt_SpiralSetting);

      return 199;
    }

    // Set BackLash Command
    else if (Contains(cmd, "_BL:"))
    {
      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 5);

        X_backlash = cmd.toInt();

        MSGOutput("Set X BackLash: " + WR_EEPROM(EP_X_backlash, cmd));
      }

      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 5);

        Y_backlash = cmd.toInt();

        MSGOutput("Set Y BackLash: " + WR_EEPROM(EP_Y_backlash, cmd));
      }

      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 5);

        Z_backlash = cmd.toInt();

        MSGOutput("Set Z BackLash: " + WR_EEPROM(EP_Z_backlash, cmd));
      }
    }

    // Get IL Command
    else if (cmd == "IL?")
    {
      switch (GetPower_Mode)
      {
      case 1:
        MSGOutput("IL:" + String(Cal_PD_Input_IL(Get_PD_Points)));
        break;

      case 2:
        MSGOutput("IL:" + String(Cal_PD_Input_Dac(Get_PD_Points)));
        break;

      case 3:
        MSGOutput("IL:" + String(Cal_PD_Input_Row_IL(Get_PD_Points)));
        break;

      case 4:
        MSGOutput("IL:" + String(Cal_PD_Input_Row_Dac(Get_PD_Points)));
        break;

      default:
        MSGOutput("IL:" + String(Cal_PD_Input_IL(Get_PD_Points)));
        break;
      }
    }

    else if (Contains(cmd, "GetILMode"))
    {
      isMotorManualCtr = false;
      isCheckStop = false;
      isKeepGetIL = true;
      Serial.println("GetILMode");

      return 0;
    }

    // Get Ref Command
    else if (cmd == "REF?" || cmd == "Ref?" || cmd == "ref?")
    {
      String eepromString = ReadInfoEEPROM(0, 8); //(int start_position, int data_length)

      Serial.println("Get_Ref:" + eepromString); //(start_position, data_length)  // Reading Data from EEPROM

      eepromString.trim();
      ref_Dac = eepromString.toDouble();
      ref_IL = ILConverter(ref_Dac);

      Serial.println("Dac:" + eepromString);  //(start_position, data_length)  // Reading Data from EEPROM
      Serial.println("IL:" + String(ref_IL)); //(start_position, data_length)  // Reading Data from EEPROM
    }

    // Set Ref Command
    else if (Contains(cmd, "Set_Ref:"))
    {
      cmd = ExtractCmd(cmd, "Set_Ref:");

      CleanEEPROM(EP_PD_Ref, 8);               // Clean EEPROM(int startPosition, int datalength)
      WriteInfoEEPROM(String(cmd), EP_PD_Ref); //(data, start_position)  // Write Data to EEPROM
      EEPROM.commit();

      String eepromString = ReadInfoEEPROM(EP_PD_Ref, 8); //(int start_position, int data_length)

      Serial.printf("PD ref: %s\n", eepromString);

      ref_Dac = eepromString.toDouble();
      ref_IL = ILConverter(ref_Dac);
    }

    // Set Target IL
    else if (Contains(cmd, "Set_Target_IL:"))
    {
      cmd = ExtractCmd(cmd, "Set_Target_IL:");
      Target_IL = WR_EEPROM(72, cmd).toDouble();
      MSGOutput("Set_Target_IL:" + String(Target_IL));
    }

    // Set Motor Step Ratio
    else if (Contains(cmd, "Set_MotorStepRatio:"))
    {
      cmd = ExtractCmd(cmd, "Set_MotorStepRatio:");
      MotorStepRatio = WR_EEPROM(208, cmd).toDouble();
      MSGOutput("Set_MotorStepRatio:" + String(MotorStepRatio));
    }

    // Set Motor Step Delay Ratio
    else if (Contains(cmd, "Set_MotorStepDelayRatio:"))
    {
      cmd = ExtractCmd(cmd, "Set_MotorStepDelayRatio:");
      MotorStepDelayRatio = WR_EEPROM(216, cmd).toDouble();
      MSGOutput("Set_MotorStepDelayRatio:" + String(MotorStepDelayRatio));
    }

    // Get Motor position now
    else if (Contains(cmd, "POS?"))
    {
      MSGOutput("Position:" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z));
    }

    // Reset Motor Position
    else if (Contains(cmd, "POS_RST"))
    {
      Pos_Now.X = 0;
      Pos_Now.Y = 0;
      Pos_Now.Z = 0;
      MSGOutput("Position:" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z));
    }

    // Set Motor Driver Current On/Off
    else if (Contains(cmd, "AWO"))
    {
      cmd = ExtractCmd(cmd, "AWO");

      if (Contains(cmd, "1"))
      {
        digitalWrite(AWO_Pin, 0);
        MSGOutput("Motor Driver ON");
      }
      else if (Contains(cmd, "0"))
      {
        digitalWrite(AWO_Pin, 1);
        MSGOutput("Motor Driver OFF");
      }
    }

    // Set Manual Control Motor Speed
    else if (Contains(cmd, "SPD"))
    {
      cmd = ExtractCmd(cmd, "SPD");

      if (cmd.startsWith("X") || cmd.startsWith("Y") || cmd.startsWith("Z"))
      {
        String value = cmd;
        value.remove(0, 2); // Include empty char deleted

        if (isInteger(value))
        {
          if (Contains(cmd, "X"))
          {
            delayBetweenStep_X = value.toInt();
            // WR_EEPROM(EP_delayBetweenStep_X, value);
            WriteInfoEEPROM(EP_delayBetweenStep_X, value, 8); // Write Data to EEPROM (data, start_position, MaxDataLength)
            MSGOutput("Set Motor X Speed:" + value);
          }
          else if (Contains(cmd, "Y"))
          {
            delayBetweenStep_Y = value.toInt();
            // WR_EEPROM(EP_delayBetweenStep_Y, value);
            WriteInfoEEPROM(EP_delayBetweenStep_Y, value, 8); // Write Data to EEPROM (data, start_position, MaxDataLength)
            MSGOutput("Set Motor Y Speed:" + value);
          }
          else if (Contains(cmd, "Z"))
          {
            delayBetweenStep_Z = value.toInt();
            // WR_EEPROM(EP_delayBetweenStep_Z, value);
            WriteInfoEEPROM(EP_delayBetweenStep_Z, value, 8); // Write Data to EEPROM (data, start_position, MaxDataLength)
            MSGOutput("Set Motor Z Speed:" + value);
          }
        }
      }

      else if (Contains(cmd, "?"))
      {
        MSGOutput("Motor Speed (x, y, z): (" + ReadInfoEEPROM(EP_delayBetweenStep_X, 8) + "," + ReadInfoEEPROM(EP_delayBetweenStep_Y, 8) + "," + ReadInfoEEPROM(EP_delayBetweenStep_Z, 8) + ")");
      }

      else
      {
        if (Contains(cmd, ","))
        {
          String sx = cmd.substring(0, cmd.indexOf(','));
          if (isInteger(sx))
          {
            delayBetweenStep_X = sx.toInt();

            WriteInfoEEPROM(EP_delayBetweenStep_X, sx, 8); // Write Data to EEPROM (data, start_position)
            // CleanEEPROM(EP_delayBetweenStep_X, 8);      // Clean EEPROM(int startPosition, int datalength)
            // WriteInfoEEPROM(sx, EP_delayBetweenStep_X); // Write Data to EEPROM (data, start_position)
            // WR_EEPROM(EP_delayBetweenStep_X, sx);
          }
          else
            MSGOutput("Set Motor Speed X Failed: " + sx);

          cmd.remove(0, cmd.indexOf(',') + 1);

          String sy = cmd.substring(0, cmd.indexOf(','));
          if (isInteger(sy))
          {
            delayBetweenStep_Y = sy.toInt();
            // CleanEEPROM(EP_delayBetweenStep_Y, 8);      // Clean EEPROM(int startPosition, int datalength)
            WriteInfoEEPROM(EP_delayBetweenStep_Y, sy, 8); // Write Data to EEPROM (data, start_position)
            // WR_EEPROM(EP_delayBetweenStep_Y, sy);
          }
          else
            MSGOutput("Set Motor Speed Y Failed: " + sy);

          cmd.remove(0, cmd.indexOf(',') + 1);

          String sz = cmd.substring(0, cmd.indexOf(','));
          if (isInteger(sz))
          {
            delayBetweenStep_Z = sz.toInt();
            // CleanEEPROM(EP_delayBetweenStep_Z, 8);      // Clean EEPROM(int startPosition, int datalength)
            WriteInfoEEPROM(EP_delayBetweenStep_Z, sz, 8); // Write Data to EEPROM (data, start_position)
            // WR_EEPROM(EP_delayBetweenStep_Z, sz);
          }
          else
            MSGOutput("Set Motor Speed Z Failed: " + sz);

          MSGOutput("Set Motor Speed (x, y, z): (" + String(delayBetweenStep_X) + "," + String(delayBetweenStep_Y) + "," + String(delayBetweenStep_Z) + ")");
        }
        else
        {
          if (isInteger(cmd))
          {
            int dbt = cmd.toInt();
            delayBetweenStep_X = dbt;
            delayBetweenStep_Y = dbt;
            delayBetweenStep_Z = dbt;
            WR_EEPROM(EP_delayBetweenStep_X, cmd);
            WR_EEPROM(EP_delayBetweenStep_Y, cmd);
            WR_EEPROM(EP_delayBetweenStep_Z, cmd);
            MSGOutput("Set Motor Speed (x, y, z): (" + String(delayBetweenStep_X) + "," + String(delayBetweenStep_Y) + "," + String(delayBetweenStep_Z) + ")");
          }
          else
            MSGOutput("Set Motor Speed Failed: " + cmd);
        }
      }

      return 0;
    }

    // Set Manual-Encoder Control Motor Speed
    else if (Contains(cmd, "DIR"))
    {
      cmd = ExtractCmd(cmd, "DIR");

      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 2); // Include empty char deleted
        IsDirtReverse_X = cmd.toInt();
        WR_EEPROM(EP_Motor_DIR_X, cmd);
        MSGOutput("Set Motor X DIR:" + cmd);
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 2); // Include empty char deleted
        IsDirtReverse_Y = cmd.toInt();
        WR_EEPROM(EP_Motor_DIR_Y, cmd);
        MSGOutput("Set Motor Y DIR:" + cmd);
      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 2); // Include empty char deleted
        IsDirtReverse_Z = cmd.toInt();
        WR_EEPROM(EP_Motor_DIR_Z, cmd);
        MSGOutput("Set Motor Z DIR:" + cmd);
      }
      else if (Contains(cmd, "?"))
      {
        MSGOutput("DIR (x, y, z): (" + ReadInfoEEPROM(EP_Motor_DIR_X, 8) + "," + ReadInfoEEPROM(EP_Motor_DIR_Y, 8) + "," + ReadInfoEEPROM(EP_Motor_DIR_Z, 8) + ")");
        return 0;
      }
      else
      {
        if (Contains(cmd, ","))
        {
          String sx = cmd.substring(0, cmd.indexOf(','));
          if (isNumberic(sx))
          {
            IsDirtReverse_X = sx.toInt();
            WR_EEPROM(EP_Motor_DIR_X, sx);
          }

          cmd.remove(0, cmd.indexOf(',') + 1);

          String sy = cmd.substring(0, cmd.indexOf(','));
          if (isNumberic(sy))
          {
            IsDirtReverse_Y = sy.toInt();
            WR_EEPROM(EP_Motor_DIR_Y, sy);
          }

          cmd.remove(0, cmd.indexOf(',') + 1);

          String sz = cmd.substring(0, cmd.indexOf(','));
          if (isNumberic(sz))
          {
            IsDirtReverse_Z = sz.toInt();
            WR_EEPROM(EP_Motor_DIR_Z, sz);
          }

          MSGOutput("Set DIR (x, y, z): (" + String(IsDirtReverse_X) + "," + String(IsDirtReverse_Y) + "," + String(IsDirtReverse_Z) + ")");
        }
        else
        {
          if (isNumberic(cmd))
          {
            int dbt = cmd.toInt();
            IsDirtReverse_X = dbt;
            IsDirtReverse_Y = dbt;
            IsDirtReverse_Z = dbt;
            WR_EEPROM(EP_Motor_DIR_X, cmd);
            WR_EEPROM(EP_Motor_DIR_Y, cmd);
            WR_EEPROM(EP_Motor_DIR_Z, cmd);
            MSGOutput("Set Motor DIR:" + cmd);
          }
        }
      }

      if (IsDirtReverse_X == 1)
      {
        X_DIR_True = false;
        X_DIR_False = true;
      }
      else
      {
        X_DIR_True = true;
        X_DIR_False = false;
      }

      if (IsDirtReverse_Y == 1)
      {
        Y_DIR_True = false;
        Y_DIR_False = true;
      }
      else
      {
        Y_DIR_True = true;
        Y_DIR_False = false;
      }

      if (IsDirtReverse_Z == 1)
      {
        Z_DIR_True = false;
        Z_DIR_False = true;
      }
      else
      {
        Z_DIR_True = true;
        Z_DIR_False = false;
      }
    }

    // Set Manual-Encoder Control Motor Speed
    else if (Contains(cmd, "EDR"))
    {
      cmd = ExtractCmd(cmd, "EDR");

      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 2); // Include empty char deleted
        IsEncDirtReverse_X = cmd.toInt();
        // WR_EEPROM(EP_Encoder_DIR_XYZ,
        // String(IsEncDirtReverse_X) + "," + String(IsEncDirtReverse_Y) + "," + String(IsEncDirtReverse_Z));
        MSGOutput("Set Encoder DIR:" + WR_EEPROM(EP_Encoder_DIR_XYZ,
                                                 String(IsEncDirtReverse_X) + "," + String(IsEncDirtReverse_Y) + "," + String(IsEncDirtReverse_Z)));
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 2); // Include empty char deleted
        IsEncDirtReverse_Y = cmd.toInt();
        MSGOutput("Set Encoder DIR:" + WR_EEPROM(EP_Encoder_DIR_XYZ,
                                                 String(IsEncDirtReverse_X) + "," + String(IsEncDirtReverse_Y) + "," + String(IsEncDirtReverse_Z)));
      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 2); // Include empty char deleted
        IsEncDirtReverse_Z = cmd.toInt();
        MSGOutput("Set Encoder DIR:" + WR_EEPROM(EP_Encoder_DIR_XYZ,
                                                 String(IsEncDirtReverse_X) + "," + String(IsEncDirtReverse_Y) + "," + String(IsEncDirtReverse_Z)));
      }
      else if (Contains(cmd, "?"))
      {
        MSGOutput("ENC DIR Revrs (x, y, z): (" + ReadInfoEEPROM(EP_Encoder_DIR_XYZ, 8) + ")");
        return 0;
      }
      else
      {
        if (Contains(cmd, ","))
        {
          String sx = cmd.substring(0, cmd.indexOf(','));
          if (isNumberic(sx))
          {
            IsEncDirtReverse_X = sx.toInt();
          }

          cmd.remove(0, cmd.indexOf(',') + 1);

          String sy = cmd.substring(0, cmd.indexOf(','));
          if (isNumberic(sy))
          {
            IsEncDirtReverse_Y = sy.toInt();
          }

          cmd.remove(0, cmd.indexOf(',') + 1);

          String sz = cmd.substring(0, cmd.indexOf(','));
          if (isNumberic(sz))
          {
            IsEncDirtReverse_Z = sz.toInt();
          }

          MSGOutput("Set Encoder DIR:" + WR_EEPROM(EP_Encoder_DIR_XYZ,
                                                   String(IsEncDirtReverse_X) + "," + String(IsEncDirtReverse_Y) + "," + String(IsEncDirtReverse_Z)));
        }
        else
        {
          if (isNumberic(cmd))
          {
            int dbt = cmd.toInt();
            IsEncDirtReverse_X = dbt;
            IsEncDirtReverse_Y = dbt;
            IsEncDirtReverse_Z = dbt;
            MSGOutput("Set Encoder DIR:" + WR_EEPROM(EP_Encoder_DIR_XYZ,
                                                     String(IsEncDirtReverse_X) + "," + String(IsEncDirtReverse_Y) + "," + String(IsEncDirtReverse_Z)));
          }
        }
      }
    }

    // Set Manual-Encoder Control Motor Step
    else if (Contains(cmd, "ENC"))
    {
      cmd = ExtractCmd(cmd, "ENC");

      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 2); // Include empty char deleted
        Encoder_Motor_Step_X = cmd.toInt();
        WR_EEPROM(EP_Encoder_Motor_Step_X, cmd);
        MSGOutput("Set Motor X ENC Step:" + cmd);
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 2); // Include empty char deleted
        Encoder_Motor_Step_Y = cmd.toInt();
        WR_EEPROM(EP_Encoder_Motor_Step_Y, cmd);
        MSGOutput("Set Motor Y ENC Step:" + cmd);
      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 2); // Include empty char deleted
        Encoder_Motor_Step_Z = cmd.toInt();
        WR_EEPROM(EP_Encoder_Motor_Step_Z, cmd);
        MSGOutput("Set Motor Z ENC Step:" + cmd);
      }
      else if (Contains(cmd, "?"))
      {
        MSGOutput("ENC (x, y, z): (" + ReadInfoEEPROM(EP_Encoder_Motor_Step_X, 8) + "," + ReadInfoEEPROM(EP_Encoder_Motor_Step_Y, 8) + "," + ReadInfoEEPROM(EP_Encoder_Motor_Step_Z, 8) + ")");
      }
      else if (Contains(cmd, ","))
      {
        Serial.println("Set ENC (x, y, z): (" + cmd + ")");

        String ss = cmd.substring(0, cmd.indexOf(','));
        if (isNumberic(ss))
        {
          Encoder_Motor_Step_X = ss.toInt();
          WR_EEPROM(EP_Encoder_Motor_Step_X, String(Encoder_Motor_Step_X));
        }

        cmd.remove(0, cmd.indexOf(',') + 1);

        ss = cmd.substring(0, cmd.indexOf(','));
        if (isNumberic(ss))
        {
          Encoder_Motor_Step_Y = ss.toInt();
          WR_EEPROM(EP_Encoder_Motor_Step_Y, String(Encoder_Motor_Step_Y));
        }

        cmd.remove(0, cmd.indexOf(',') + 1);

        ss = cmd.substring(0, cmd.indexOf(','));
        if (isNumberic(ss))
        {
          Encoder_Motor_Step_Z = ss.toInt();
          WR_EEPROM(EP_Encoder_Motor_Step_Z, String(Encoder_Motor_Step_Z));
        }
      }

      else
      {
        if (isNumberic(cmd))
        {
          int dbt = cmd.toInt();

          Encoder_Motor_Step_X = dbt;
          Encoder_Motor_Step_Y = dbt;
          Encoder_Motor_Step_Z = dbt;
          WR_EEPROM(EP_Encoder_Motor_Step_X, cmd);
          WR_EEPROM(EP_Encoder_Motor_Step_Y, cmd);
          WR_EEPROM(EP_Encoder_Motor_Step_Z, cmd);

          MSGOutput("Set Motor ENC Step:" + cmd);
        }
      }
    }

    // Set Heater
    else if (Contains(cmd, "Heater::"))
    {
      cmd = ExtractCmd(cmd, "Heater::");

      int Heater_Idx = cmd.toInt();

      if (Heater_Idx == 0)
      {
        digitalWrite(Heater_Stop_Pin, true);
        Serial.println("Heater Stop");
        delay(300);
        digitalWrite(Heater_Stop_Pin, false);
      }
      else if (Heater_Idx == 1)
      {
        digitalWrite(Heater_Start_Pin, true);
        Serial.println("Heater Start");
        delay(300);
        digitalWrite(Heater_Start_Pin, false);
      }
    }

    // Get/Set PD_Ref_Array[15][0]=
    else if (Contains(cmd, "PD_Ref_Array"))
    {
      cmd = ExtractCmd(cmd, "PD_Ref_Array");

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
          if (idx_2 < 2)
          {
            double value = PD_Ref_Array[idx_1][idx_2];
            Serial.printf("idx1:%d, idx2:%d, value:%.2f\n", idx_1, idx_2, value);
          }
        }
        else
        {
          idx_start = cmd.indexOf('=', idx_end) + 1;
          if (isNumberic(cmd.substring(idx_start)))
          {
            double value = cmd.substring(idx_start).toDouble();
            Serial.printf("idx1:%d, idx2:%d, value:%.2f\n", idx_1, idx_2, value);

            PD_Ref_Array[idx_1][idx_2] = value;
            MSGOutput("Set PD_Ref_Array: " + WR_EEPROM(EP_PD_Ref_Array[idx_1][idx_2], String(value)));
          }
        }
      }
      else if (Contains(cmd, "?") && !Contains(cmd, "]"))
      {
        Serial.println("Get Ref Array Start");
        String eepromString;
        int iniEP = 700;
        for (size_t i = 0; i < sizeof(PD_Ref_Array) / sizeof(PD_Ref_Array[0]); i++)
        {
          // Serial.printf("%d:[%d, %d]:[%.0f, %.2f]\r", i, EP_PD_Ref_Array[i][0],  EP_PD_Ref_Array[i][1], PD_Ref_Array[i][0], PD_Ref_Array[i][1]);
          Serial.printf("PD_Ary::%d:%.0f:%.2f\n", i, PD_Ref_Array[i][0], PD_Ref_Array[i][1]);
        }
        MSGOutput("Get Ref Array End");
      }
    }

    // Set PD average points
    else if (Contains(cmd, "AVG"))
    {
      cmd = ExtractCmd(cmd, "AVG");

      if (isInteger(cmd))
      {
        Get_PD_Points = cmd.toInt();
        MSGOutput("Set IL_Avg_Points: " + WR_EEPROM(EP_Get_PD_Points, cmd));
      }
      else
        MSGOutput("Set IL_Avg_Points Failed: " + cmd);
    }

    // Set Station Type
    else if (Contains(cmd, "Station_Type:"))
    {
      cmd = ExtractCmd(cmd, "Station_Type:");

      if (isInteger(cmd))
      {
        Station_Type = cmd.toInt();
        MSGOutput("Set Station_Type: " + WR_EEPROM(EP_Station_Type, cmd));
      }
      else
        MSGOutput("Set Station_Type Failed: " + cmd);
    }

    // Get Station Type
    else if (Contains(cmd, "Station_Type?"))
    {
      MSGOutput("Station_Type : " + String(Station_Type));
      MSGOutput("0 : CTF");
      MSGOutput("1 : VOA - Heater");
      MSGOutput("2 : VOA - No Heater");
    }

    // Set Board ID
    else if (Contains(cmd, "ID#"))
    {
      cmd = ExtractCmd(cmd, "ID#");

      ID = cmd;
      MSGOutput("Set Board_ID: " + WR_EEPROM(EP_Board_ID, cmd));
    }

    // Get Board ID
    else if (cmd == "ID?")
    {
      MSGOutput(ReadInfoEEPROM(EP_Board_ID, 8));
    }

    // Set Station ID
    else if (Contains(cmd, "ID_Station#"))
    {
      cmd = ExtractCmd(cmd, "ID_Station#");

      // cmd.remove(0, 11);
      // cmd.trim();
      Station_ID = cmd;
      MSGOutput("Set Station_ID: " + WR_EEPROM(EP_Station_ID, cmd));
    }

    // Get Station ID
    else if (cmd == "ID_Station?")
    {
      MSGOutput(ReadInfoEEPROM(EP_Station_ID, 8));
    }

    // Set Server ID
    else if (Contains(cmd, "ID_Server#"))
    {
      cmd = ExtractCmd(cmd, "ID_Server#");

      // cmd.remove(0, 10);
      MSGOutput("Set Server ID: " + WR_EEPROM(88, 32, cmd));
    }

    // Get Server ID
    else if (cmd == "ID_Server?")
    {
      MSGOutput(ReadInfoEEPROM(88, 32));
    }

    // Get Firmware Version
    else if (cmd == "VER?")
    {
      MSGOutput("FW Verion: " + ReadInfoEEPROM(EP_FW_Version, 8));
    }

    // Set Firmware Version
    else if (Contains(cmd, "VER#"))
    {
      cmd = ExtractCmd(cmd, "VER#");

      MSGOutput("Set FW Verion: " + WR_EEPROM(EP_FW_Version, cmd));
    }

    // Re-start esp32
    else if (Contains(cmd, "ESP_RST"))
    {
      MSGOutput("ESP_Reset");
      ESP.restart();
    }

    // Clena EEPROM
    else if (Contains(cmd, "CLR_"))
    {
      cmd = ExtractCmd(cmd, "CLR_");

      if (isNumberic(cmd))
      {
        int epmP = cmd.toInt();
        CleanEEPROM(epmP, 8); // Clean EEPROM(int startPosition, int datalength)
        WR_EEPROM(cmd.toInt(), "");
        EEPROM.commit();
        MSGOutput("CleanEEPROM:" + String(epmP));
        cmd = "";
      }
    }

    // Get UI_Data cmd and return value to controller
    else if (Contains(cmd, "UI?"))
    {
      DataSent_Controller(cmd);

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
        if (i < sizeof(ThisAddress) - 1)
          addr += ":";
      }

#pragma endregion

      sendmsg_UI_Data.msg = "MAC:Core";
      addr.toCharArray(sendmsg_UI_Data.para, 30);
      MSGOutput("MAC Core:" + addr);

      esp_err_t result = esp_now_send(ThisAddress, (uint8_t *)&sendmsg_UI_Data, sizeof(sendmsg_UI_Data));
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
        if (i < sizeof(ServerAddress) - 1)
          addr += ":";
      }

#pragma endregion

      sendmsg_UI_Data.msg = "MAC:Server";
      addr.toCharArray(sendmsg_UI_Data.para, 30);
      MSGOutput("MAC Server:" + ThisAddr);

      esp_err_t result = esp_now_send(ControllerAddress, (uint8_t *)&sendmsg_UI_Data, sizeof(sendmsg_UI_Data));
      cmd = "";
      cmd_from_contr = "";
      cmd_value_from_contr = "";
    }

    // Set UI_Data from controller
    else if (Contains(cmd, "UI_Data_"))
    {
      String s = "UI_Data_";
      cmd.remove(0, s.length());
      cmd.trim();
      // Serial.println("UI_Data_:" + cmd);

      if (isNumberic(cmd_value_from_contr))
      {
        if (Contains(cmd, "Target_IL"))
          Target_IL = WR_EEPROM(72, cmd_value_from_contr).toDouble();
        else if (Contains(cmd, "Z_offset"))
          AQ_Scan_Compensation_Steps_Z_A = WR_EEPROM(160, cmd_value_from_contr).toInt();
        else if (Contains(cmd, "speed_x"))
          delayBetweenStep_X = WR_EEPROM(48, cmd_value_from_contr).toInt();
        else if (Contains(cmd, "speed_y"))
          delayBetweenStep_Y = WR_EEPROM(56, cmd_value_from_contr).toInt();
        else if (Contains(cmd, "speed_z"))
          delayBetweenStep_Z = WR_EEPROM(64, cmd_value_from_contr).toInt();
        else if (Contains(cmd, "Ref"))
        {
          if (true)
          {
            digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
            delay(5);

            averagePDInput = 0;
            for (int i = 0; i < 20; i++)
              // averagePDInput += ads.getResult_V(); // alternative: getResult_mV for Millivolt
              averagePDInput += ads.readADC_SingleEnded(0);

            averagePDInput = (averagePDInput / 20);

            ref_Dac = averagePDInput;
            ref_IL = ILConverter(averagePDInput);

            MSGOutput("EEPROM(" + String(0) + ") - " + WR_EEPROM(0, String(ref_Dac)));     // For update HMI ref value
            Serial.println("Ref_Dac: " + String(ref_Dac) + ", Ref_IL: " + String(ref_IL)); // Reading Data from EEPROM(start_position, data_length)

            digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode
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

    // Command No.
    else if (Contains(cmd, "cmd"))
    {
      cmd = ExtractCmd(cmd, "cmd");

      // cmd.remove(0, 3);
      cmd_No = cmd.toInt();
      delay(10);
    }
  }
  else if (ButtonSelected >= 0)
  {
    // Keyboard No. to Cmd Set No.
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
  isCheckStop = true;

  // Function Execution
  if (cmd_No != 0)
  {
    // Functions: Alignment
    if (cmd_No <= 100)
    {
      switch (cmd_No)
      {
        // Functions: Auto Align
      case 1: /* Auto Align */
        if (true)
        {
          DataSent_Controller("AA");

          bool initial_wifi_isConnected = isWiFiConnected;
          if (Station_Type != 3)
            isWiFiConnected = false;

          AQ_Scan_Compensation_Steps_Z_A = 0;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
          delay(3);

          AutoAlign();

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode
          MSGOutput("Auto_Align_End");
          MotorCC_A = true;

          StopValue = 0; // 0 dB

          AQ_Scan_Compensation_Steps_Z_A = ReadInfoEEPROM(160, 8).toInt();
          Serial.println("AQ_Scan_Compensation_Steps_Z_A: " + String(AQ_Scan_Compensation_Steps_Z_A));

          DataSent_Controller("Menu");

          delay(150);

          DataSent_Controller("Menu");

          delay(150);

          DataSent_Controller("Menu"); // Send Main UI msg to controller

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

        // Functions: Fine Scan
      case 2: /* Fine Scan */
        if (!btn_isTrigger)
        {
          DataSent_Controller("FS");

          if (Station_Type == 3 || Station_Type == 4)
            StopValue = Target_IL;
          else
            StopValue = 0; // 0 dB

          bool K_OK = true;
          bool initial_wifi_isConnected = isWiFiConnected;

          AQ_Scan_Compensation_Steps_Z_A = 0;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode

          //------------------------------------------------------------------------------------------------------------------------------Q Scan Z

          Fine_Scan(Z_Dir, false);

          Serial.println("Fine_Scan 3 End");

          // CheckStop();
          if (isStop)
            true;

          //------------------------------------------------------------------------------------------------------------------------------Q Scan Y

          Fine_Scan(Y_Dir, false);

          // CheckStop();
          if (isStop)
            true;

          //------------------------------------------------------------------------------------------------------------------------------Q Scan X

          Fine_Scan(X_Dir, false);

          if (isStop)
            true;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode

          MSGOutput("Auto_Align_End");

          delay(200);

          DataSent_Controller("Menu");

          AQ_Scan_Compensation_Steps_Z_A = ReadInfoEEPROM(160, 8).toInt();

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

        // Functions: Auto Curing
      case 3: /* Auto Curing */
        if (!btn_isTrigger)
        {
          btn_isTrigger = false;

          isILStable = false;
          bool isStopAlign = false;
          is_AutoCuring = true;

          ButtonSelected = -1;

          double IL_stable_count = 0;
          double Acceptable_Delta_IL = 0.3; // 0.3 for ctf
          unsigned long lastUpdateQT = 0;
          Q_Time = 0;

          time_curing_0 = millis();
          time_curing_1 = time_curing_0;
          time_curing_2 = time_curing_1;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
          delay(155);

          AutoCuring_Best_IL = Cal_PD_Input_IL(Get_PD_Points * 3);

          StopValue = AutoCuring_Best_IL;

          double temp_targetIL = Target_IL;
          Target_IL = AutoCuring_Best_IL; // Send AQ target IL to controller UI

          DataSent_Controller("AQ");

          if (StopValue > -0.9)
            StopValue = -0.9;

          Z_ScanSTP = AQ_Scan_Steps_Z_A; // 125 (AQ_Scan_Steps_Z_A)
          MSGOutput("Auto-Curing");
          CMDOutput("AQ");                             // Auto_Curing Start
          CMDOutput("QT" + String(AQ_Total_TimeSpan)); // Auto_Curing Start
          // MSGOutput("StopValue : " + String(StopValue));
          // MSGOutput("Target_IL : " + String((Target_IL - (Acceptable_Delta_IL))));

          // VOA Station - Heater
          if (Station_Type == 1)
          {
            Acceptable_Delta_IL = 0.05;

            AQ_Total_TimeSpan = 600;     // 10 mins
            AQ_StopAlign_TimeSpan = 240; // 4 mins

            StopValue = (Target_IL - (Acceptable_Delta_IL));
            MSGOutput("AutoCuring_Best_IL : " + String(AutoCuring_Best_IL));
            MSGOutput("StopValue : " + String(StopValue));
            MSGOutput("Target_IL : " + String((Target_IL - (Acceptable_Delta_IL))));
            MSGOutput("Scan Threshold : " + String((AutoCuring_Best_IL - Acceptable_Delta_IL)));

            digitalWrite(Heater_Start_Pin, true);
            MSGOutput("Heater Start");
            delay(600);
            digitalWrite(Heater_Start_Pin, false);

            Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, 30, 12 * MotorStepDelayRatio, 350);

            MSGOutput("Z offset 0.5 um");
          }
          else if (Station_Type == 2) // VOA Station - No Heater
          {
            Acceptable_Delta_IL = 0.05;

            AQ_Total_TimeSpan = 300;    // 5 mins
            AQ_StopAlign_TimeSpan = 60; // 1 min

            StopValue = (Target_IL - (Acceptable_Delta_IL));
            MSGOutput("AutoCuring_Best_IL : " + String(AutoCuring_Best_IL));
            MSGOutput("StopValue : " + String(StopValue));
            MSGOutput("Target_IL : " + String((Target_IL - (Acceptable_Delta_IL))));
            MSGOutput("Scan Threshold : " + String((AutoCuring_Best_IL - Acceptable_Delta_IL)));

            Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, 30, 12 * MotorStepDelayRatio, 350);

            MSGOutput("Z offset 0.5 um");
          }
          else
          {
            AQ_StopAlign_TimeSpan = 860; // 1 min

            MSGOutput("AutoCuring_Best_IL : " + String(AutoCuring_Best_IL));
            MSGOutput("Acceptable_Delta_IL : " + String(Acceptable_Delta_IL));
            MSGOutput("Target_IL : " + String((Target_IL)));
            // StopValue = (Target_IL - (Acceptable_Delta_IL));
            StopValue = Target_IL;
            MSGOutput("Scan Threshold : " + String(Target_IL - Acceptable_Delta_IL));
            MSGOutput("AQ_StopAlign_TimeSpan : " + String(AQ_StopAlign_TimeSpan));
          }

          while (true)
          {
            if (digitalRead(Tablet_PD_mode_Trigger_Pin))
            {
              digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
              delay(2);
            }

            delay(50);

            PD_Now = Cal_PD_Input_IL(Get_PD_Points);
            Q_Time = ((millis() - time_curing_0) / 1000);

            if (isStopAlign)
            {
              if (Q_Time - lastUpdateQT >= 3) // Reduce get data rate after stop auto-aligning
              {
                lastUpdateQT = Q_Time;
                MSGOutput("QT_IL:" + String(Q_Time) + " s, " + String(PD_Now));
              }
            }
            else
            {
              MSGOutput("QT_IL:" + String(Q_Time) + " s, " + String(PD_Now));
            }

            String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
            DataSent_Controller("AQ");

            delay(998); // get data time rate

            if (isStop)
              break;

            // Q State
            if (true)
            {
              // VOA Station
              if (Station_Type == 1 || Station_Type == 2)
              {
                Q_State = 1;
              }

              // CTF Station
              else
              {
                if (Q_Time <= 540) // 540
                {
                  Q_State = 1;
                }
                else if (Q_Time > 540 && Q_Time <= 600) // 540, 600
                {
                  Q_State = 2;
                }
                else if (Q_Time > 600 && Q_Time <= 700) // 600, 700
                {
                  Q_State = 3;
                }
                else if (Q_Time > 700) // 700
                {
                  Q_State = 4;
                }
              }
            }

            // MSGOutput("Test 3");

            // Q Stop Conditions
            if (true)
            {
              // VOA Station - Heater
              if (Station_Type == 1)
              {
                // Auto Align time threshold ,  4 mins
                if (Q_Time >= AQ_StopAlign_TimeSpan && !isStopAlign) // 240
                {
                  MSGOutput("Update : Stop Auto Align");
                  isStopAlign = true;
                }

                // Total curing time , 10 mins, 600s
                if (Q_Time >= AQ_Total_TimeSpan)
                {
                  MSGOutput("Curing Time's up - Stop Auto Curing");
                  isStop = true;
                  break;
                }
              }

              // VOA Station - No Heater
              else if (Station_Type == 2)
              {
                // Auto Align time threshold ,  1 mins
                if (Q_Time >= AQ_StopAlign_TimeSpan && !isStopAlign) // 60
                {
                  MSGOutput("Update : Stop Auto Align");
                  isStopAlign = true;
                }

                // Total curing time , 5 mins, 300s
                if (Q_Time >= AQ_Total_TimeSpan)
                {
                  MSGOutput("Curing Time's up - Stop Auto Curing");
                  isStop = true;
                  break;
                }
              }

              // CTF Station
              else
              {
                // IL Stable Time ,  70 secs,  curing time threshold , 12.5 mins
                if (time_curing_2 - time_curing_1 > 70000 && Q_Time >= AQ_StopAlign_TimeSpan && !isStopAlign) // 800
                {
                  MSGOutput("Update : IL Stable - Stop Auto Curing");
                  isStopAlign = true;
                }
                // Total curing time , 14 mins, 840s
                else if (Q_Time >= AQ_Total_TimeSpan - 1)
                {
                  MSGOutput("Over Limit Curing Time - Stop Auto Curing");
                  isStop = true;
                  break;
                }
              }
            }

            if (isStop)
              break;

            // Q scan conditions
            if (true)
            {
              if (Q_State == 1)
              {
                if (Z_ScanSTP != AQ_Scan_Steps_Z_A)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_A; // 125 - > 35 (AQ_Scan_Steps_Z_B)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }
              else if (Q_State == 2)
              {
                if (Z_ScanSTP != AQ_Scan_Steps_Z_B)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_B; // 125 - > 35 (AQ_Scan_Steps_Z_B)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }
              else if (Q_State == 3)
              {
                if (Z_ScanSTP != AQ_Scan_Steps_Z_C)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_C; // 70 (AQ_Scan_Steps_Z_C)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }
              else if (Q_State == 4)
              {
                if (Z_ScanSTP != AQ_Scan_Steps_Z_D)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_D; // 50 (AQ_Scan_Steps_Z_D)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }

              if (Q_Time > 420)
              {
                if (Acceptable_Delta_IL > 0.2)
                {
                  if (StopValue > -1.3)
                    Acceptable_Delta_IL = 0.15; // Target IL changed 0.2
                  else
                    Acceptable_Delta_IL = 0.12; // Target IL changed 0.12

                  MSGOutput("Update Scan Condition: " + String(Acceptable_Delta_IL));
                }
              }
            }

            PD_Now = Cal_PD_Input_IL(Get_PD_Points * 3); // Increase IL stability

            bool forcedAlign = false;
            if (cmd_value_from_contr == "8")
              forcedAlign = true;

            if (PD_Now >= (AutoCuring_Best_IL - (Acceptable_Delta_IL)) && cmd_value_from_contr != "8")
            {
              time_curing_2 = millis();
              continue;
            }
            else // Start Align
            {
              cmd_from_contr = "";
              cmd_value_from_contr = "";
              if (!isStopAlign || forcedAlign)
              {
                // Q Scan
                if (true && Q_Time <= 900 && !isStopAlign || forcedAlign)
                {
                  PD_Now = Cal_PD_Input_IL(Get_PD_Points * 3); // Increase IL stability

// Into Q Scan X
#pragma region Q Scan X
                  if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL) || forcedAlign)
                  {
                    DataSent_Controller("Scan");

                    Fine_Scan(X_Dir, false); //--------------------------------------------------------Q Scan X

                    PD_Now = Cal_PD_Input_IL(Get_PD_Points);

                    Q_Time = ((millis() - time_curing_0) / 1000);
                    MSGOutput("QT_IL:" + String(Q_Time) + " s, " + String(PD_Now));
                    String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                    DataSent_Controller("AQ");

                    if (PD_Now - Cal_PD_Input_IL(Get_PD_Points) > 1)
                    {
                      DataSent_Controller("Scan");

                      Fine_Scan(X_Dir, false); //------------------------------------------------------Q Scan X

                      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

                      Q_Time = ((millis() - time_curing_0) / 1000);
                      MSGOutput("QT_IL:" + String(Q_Time) + " s, " + String(PD_Now));
                      String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                      DataSent_Controller("AQ");
                    }

                    if (Q_State >= 4 && (maxIL_in_FineScan - minIL_in_FineScan) <= 0.25 && Q_Time >= 820 && !isStopAlign)
                    {
                      MSGOutput("Update : Delta IL < 0.25, break"); // break curing loop
                      isStopAlign = true;
                    }
                  }
#pragma endregion

                  if (isStop)
                    break;

                  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
                  delay(5);

                  PD_Now = Cal_PD_Input_IL(Get_PD_Points * 3); // Increase IL stability
                  MSGOutput("Q_State: " + String(Q_State));

// Into Q Scan Y
#pragma region Q Scan Y
                  if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL) || Q_State == 1 || forcedAlign)
                  {
                    DataSent_Controller("Scan");

                    Fine_Scan(Y_Dir, false); //--------------------------------------------------------Q Scan Y

                    PD_Now = Cal_PD_Input_IL(Get_PD_Points);

                    Q_Time = ((millis() - time_curing_0) / 1000);
                    MSGOutput("QT_IL:" + String(Q_Time) + " s, " + String(PD_Now));
                    String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                    DataSent_Controller("AQ");

                    if (PD_Now - Cal_PD_Input_IL(Get_PD_Points) > 1)
                    {
                      DataSent_Controller("Scan");

                      Fine_Scan(Y_Dir, false); //------------------------------------------------------Q Scan Y

                      Q_Time = ((millis() - time_curing_0) / 1000);
                      MSGOutput("QT_IL:" + String(Q_Time) + " s, " + String(PD_Now));
                      String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                      DataSent_Controller("AQ");
                    }

                    if (Q_State >= 4 && (maxIL_in_FineScan - minIL_in_FineScan) <= 0.25 && Q_Time >= 820 && !isStopAlign)
                    {
                      MSGOutput("Update : Delta IL < 0.25, break"); // break curing loop
                      isStopAlign = true;
                    }
                  }
#pragma endregion

                  if (isStop)
                    break;

                  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
                  delay(5);

                  PD_Before = Cal_PD_Input_IL(Get_PD_Points);

                  bool K_OK = true;

                  // Into Q Scan Z (CTF Station)
                  if (Station_Type == 0)
                  {
#pragma region Q Scan Z
                    if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL) || forcedAlign)
                    {
                      //-----------------------------------------------------------Q Scan Z2

                      DataSent_Controller("Scan");

                      digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
                      delay(5);

                      CMDOutput("AS");

                      // Fix Z scan steps when Auto Q to make sure find best IL
                      K_OK = AutoAlign_Scan_DirectionJudge_V3(Z_Dir, 6, Z_ScanSTP * MotorStepRatio, FS_Stable_Z, false, FS_DelaySteps_Z, Target_IL, FS_Avg_Z, FS_Trips_Z, "Z Fine-Scan,Trip_");
                      // K_OK = Scan_AllRange_TwoWay(2, FS_Count_Z, Z_ScanSTP, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Scan,Trip_");
                      CMDOutput("%:");

                      Q_Time = ((millis() - time_curing_0) / 1000);
                      MSGOutput("QT_IL:" + String(Q_Time) + " s, " + String(PD_Now));
                      String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                      DataSent_Controller("AQ");

                      if (!K_OK)
                      {
                        DataSent_Controller("Scan");

                        CMDOutput("AS");
                        AutoAlign_Scan_DirectionJudge_V3(Z_Dir, 6, Z_ScanSTP * MotorStepRatio, FS_Stable_Z, false, FS_DelaySteps_Z, Target_IL, FS_Avg_Z, FS_Trips_Z, "Z Fine-Scan,Trip_");
                        // Scan_AllRange_TwoWay(2, FS_Count_Z, Z_ScanSTP, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Re-Scan,Trip_");
                        CMDOutput("%:");

                        Q_Time = ((millis() - time_curing_0) / 1000);
                        MSGOutput("QT_IL:" + String(Q_Time) + " s, " + String(PD_Now));
                        String(PD_Now).toCharArray(sendmsg_UI_Data.para, 30);
                        DataSent_Controller("AQ");
                      }
                    }
#pragma endregion
                  }

                  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
                  delay(5);

                  if (isStop)
                    break;
                }

                PD_Now = Cal_PD_Input_IL(Get_PD_Points);

                // IL stable count (CTF Station only)
                if (abs(PD_Before - PD_Now) < 0.2 && Q_Time > 820 && Station_Type == 0)
                {
                  IL_stable_count++;

                  if (IL_stable_count > 4 && !isStopAlign && Q_Time > 820)
                  {
                    MSGOutput("Update : IL stable to break");
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
          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode

          // VOA Station - Heater
          if (Station_Type == 1)
          {
            digitalWrite(Heater_Stop_Pin, true);
            Serial.println("Heater Stop");
            delay(600);
            digitalWrite(Heater_Stop_Pin, false);
          }

          String eepromString = ReadInfoEEPROM(40, 8);                         // Reading z backlash from EEPROM
          MSGOutput("Reset Z backlash from EEPROM: " + ReadInfoEEPROM(40, 8)); //(start_position, data_length)
          Z_backlash = eepromString.toInt();

          Target_IL = temp_targetIL; // Re-cover Target IL value

          MSGOutput("Auto Q End");

          // Only CTF station has control panel
          if (Station_Type == 0)
          {
            DataSent_Controller("Menu");

            MSGOutput("LCD Re-Start");
          }

          Q_Time = 0;
        }
        cmd_No = 0;
        break;

      case 5: /* Fine Scan X */
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          if (Station_Type != 3)
            isWiFiConnected = false;

          Fine_Scan(X_Dir, false);

          MSGOutput("Auto Align End");

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 6: /* Fine Scan Y */
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          if (Station_Type != 3)
            isWiFiConnected = false;

          Fine_Scan(Y_Dir, false);

          MSGOutput("Auto Align End");

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 7: /* Fine Scan Z */
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          if (Station_Type != 3)
            isWiFiConnected = false;

          Fine_Scan(Z_Dir, false);

          MSGOutput("Auto Align End");

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 11: /* Get Board ID */
        MSGOutput("Board ID: " + ReadInfoEEPROM(EP_Board_ID, 8));
        cmd_No = 0;
        break;

      case 12: /* Get Station ID */
        MSGOutput("Station ID: " + ReadInfoEEPROM(EP_Station_ID, 8));
        cmd_No = 0;
        break;

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
          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
          delay(3);

          averagePDInput = 0;
          for (int i = 0; i < 20; i++)
            // averagePDInput += ads.getRawResult(); // alternative: getResult_mV for Millivolt
            averagePDInput += ads.readADC_SingleEnded(0);

          averagePDInput = (averagePDInput / 20);

          ref_Dac = averagePDInput;
          ref_IL = ILConverter(averagePDInput);

          CleanEEPROM(0, 8); // Clean EEPROM(int startPosition, int datalength)

          WriteInfoEEPROM(String(averagePDInput), 0); // Write Data to EEPROM (data, start_position)
          EEPROM.commit();

          Serial.println("Ref_Dac: " + ReadInfoEEPROM(0, 8) + ", Ref_IL: " + String(ref_IL)); // Reading Data from EEPROM(start_position, data_length)

          MSGOutput("EEPROM(" + String(0) + ") - " + ReadInfoEEPROM(0, 8)); // For update HMI ref value

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode
          delay(3);
        }

        cmd_No = 0;
        break;

      case 21: /* Get IL On */
        isGetPower = true;

        Serial.println("Cmd: Get IL On");
        digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode

        cmd_No = 0;
        break;

      case 22: /* Get Power Off */
        isGetPower = false;

        Serial.println("Cmd: Get Power Off");
        digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode

        cmd_No = 0;
        break;

      case 23: /* Get Power Mode: IL(dB) */
        GetPower_Mode = 1;
        Serial.println("Cmd: Get Power Mode: IL(dB)");
        cmd_No = 0;
        break;

      case 24: /* Get Power Mode: IL Dac */
        GetPower_Mode = 2;
        Serial.println("Cmd: Get Power Mode: IL Dac");
        cmd_No = 0;
        break;

      case 25: /* Get Power Mode: Row IL(dBm) */
        GetPower_Mode = 3;
        Serial.println("Cmd: Get Power Mode: Row IL(dBm)");
        cmd_No = 0;
        break;

      case 26: /* Get Power Mode: Row Dac */
        GetPower_Mode = 4;
        Serial.println("Cmd: Get Power Mode: Row Dac");
        cmd_No = 0;
        break;

      case 27: /* Get Row PD Dac */
        Serial.println(String(Cal_PD_Input_Row_Dac(Get_PD_Points)));
        cmd_No = 0;
        break;

      case 28: /* Get All eeprom value */
        for (int i = 0; i < 700; i = i + 8)
        {
          // if (i == 88)
          // {
          //   MSGOutput("EEPROM(" + String(i) + ") - " + ReadInfoEEPROM(i, 32)); // Server ID
          //   i = 120;
          // }
          // else if (i == 120)
          // {
          //   MSGOutput("EEPROM(" + String(i) + ") - " + ReadInfoEEPROM(i, 32)); // Server Password
          //   i = 152;
          // }
          // else
          MSGOutput("EPRM(" + String(i) + ") - " + ReadInfoEEPROM(i, 8)); // Reading EEPROM(int start_position, int data_length)
        }

      case 29: /* Get XYZ Position */
        DataOutput(false);
        cmd_No = 0;
        break;

        // case 30: /* Get XYZ Position */
        //   EmergencyStop();
        //   MSGOutput("Stop");
        //   cmd_No = 0;
        // break;
      }
    }

    // Functions: Motion
    if (cmd_No > 100)
    {
      // vTaskSuspend(Task_1); // 傳入 NULL 表示暫停Task
      // delay(50);
      // isMotorCont = true;

      switch (cmd_No)
      {
        // Function: Cont-------------------------------------------------------------------------
        // Z feed - cont
      case 101:
        while (true)
        {
          MotorCC_A = MotorCC_Z;
          Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, false, 100 * MotorStepRatio, delayBetweenStep_Z);
          MotorCC_Z = false;

          if (isStop)
          {
            isStop = false;
            break;
          }
        }

        DataOutput(false);

        cmd_No = 0;

        break;

      case 103:
        while (true)
        {
          // MotorCC_A = MotorCC_Z;
          Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, true, 100 * MotorStepRatio, delayBetweenStep_Z);
          // MotorCC_Z = true;

          if (isStop)
          {
            isStop = false;
            break;
          }
        }
        DataOutput(false);

        cmd_No = 0;

        break;

        // X feed - cont
      case 102:
        while (true)
        {
          MotorCC_A = MotorCC_X;
          Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, false, 100 * MotorStepRatio, delayBetweenStep_X);
          MotorCC_X = false;

          if (isStop)
          {
            isStop = false;
            break;
          }
        }
        DataOutput();

        cmd_No = 0;
        break;
        // X+ - cont
      case 105:
        while (true)
        {
          MotorCC_A = MotorCC_X;
          Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, true, 100 * MotorStepRatio, delayBetweenStep_X);
          MotorCC_X = true;

          if (isStop)
          {
            isStop = false;
            break;
          }
        }
        DataOutput(false);

        cmd_No = 0;
        break;

        // Y- feed - cont
      case 106:

        while (true)
        {
          MotorCC_A = MotorCC_Y;
          Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, false, 100 * MotorStepRatio, delayBetweenStep_Y);
          MotorCC_Y = false;

          if (isStop)
          {
            isStop = false;
            break;
          }
        }
        DataOutput(false);

        cmd_No = 0;
        break;

      // Y+ feed - cont
      case 104:

        while (true)
        {
          MotorCC_A = MotorCC_Y;
          Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, true, 100 * MotorStepRatio, delayBetweenStep_Y);
          MotorCC_Y = true;

          if (isStop)
          {
            isStop = false;
            break;
          }
        }
        DataOutput(false);

        cmd_No = 0;
        break;

        // Function: Jog-------------------------------------------------------------------------

      // X+ feed - jog
      case 107:
        MotorCC_A = MotorCC_X;
        Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, false, 400, delayBetweenStep_X);
        MotorCC_X = true;

        cmd_No = 0;
        break;

        // X- feed - jog
      case 108:
        MotorCC_A = MotorCC_X;
        Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, true, 400, delayBetweenStep_X);
        MotorCC_X = false;

        cmd_No = 0;
        break;

      // Y+ feed - jog
      case 109:
        MotorCC_A = MotorCC_Y;
        Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, false, 400, delayBetweenStep_Y);
        MotorCC_Y = true;
        cmd_No = 0;
        break;

      // Y- feed - jog
      case 110:
        MotorCC_A = MotorCC_Y;
        Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, true, 400, delayBetweenStep_Y);
        MotorCC_Y = false;
        cmd_No = 0;
        break;

        // Z+ feed - jog
      case 111:
        MotorCC_A = MotorCC_Z;
        Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, true, 400, delayBetweenStep_Z);
        MotorCC_Z = true;
        cmd_No = 0;
        break;

        // Z- feed - jog
      case 112:
        MotorCC_A = MotorCC_Z;
        Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, false, 400, delayBetweenStep_Z);
        MotorCC_Z = false;
        cmd_No = 0;
        break;

      // EmergencyStop
      case 129:
        isStop = true;
        Serial.println("EmergencyStop");
        digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode
        cmd_No = 0;
        break;

      // Go Home
      case 130:
        Move_Motor_abs_all(0, 0, 0);
        cmd_No = 0;
        break;

      // Line Scan
      case 198:

        int XYZ;
        int count;
        int motorStepC;
        // int stableDelay;
        bool Direction;
        // int delayBetweenStep;
        int StopValueC;
        int Get_PD_Points;
        int Trips;
        double SlopeC;
        double Tilt_XC;
        double Tilt_YC;
        double Tilt_ZC;

        XYZ = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toInt();
        Serial.println("XYZ:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // xyz
        Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);

        count = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toInt();
        Serial.println("count:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // count
        Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);

        motorStepC = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toInt();
        Serial.println("motorStepC:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // step
        Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);

        stableDelay = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toInt();
        Serial.println("stableDelay:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // stable delay
        Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);

        Direction = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')) == "1";
        Serial.println("Direction:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // direction
        Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);

        delayBetweenStep = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toInt();
        Serial.println("delayBetweenStep:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // delaySteps
        Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);

        StopValueC = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toInt();
        Serial.println("StopValue:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // stopValue
        Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);

        Get_PD_Points = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toInt();
        Serial.println("Get_PD_Points:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // average points
        Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);

        Trips = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toInt();
        Serial.println("Trips:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // trips
        Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);

        if (Txt_LineScanSetting != "")
        {
          SlopeC = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toDouble();
          Serial.println("Slope:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // Slope
          Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);
        }

        if (Txt_LineScanSetting != "")
        {
          Tilt_XC = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toDouble();
          Serial.println("Tilt_X:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // Tilt_X
          Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);
        }

        if (Txt_LineScanSetting != "")
        {
          Tilt_YC = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toDouble();
          Serial.println("Tilt_Y:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // Tilt_Y
          Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);
        }

        if (Txt_LineScanSetting != "")
        {
          Tilt_ZC = Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_')).toDouble();
          Serial.println("Tilt_Z:" + Txt_LineScanSetting.substring(0, Txt_LineScanSetting.indexOf('_'))); // Tilt_Z
          Txt_LineScanSetting.remove(0, Txt_LineScanSetting.indexOf('_') + 1);
        }

        isMotorManualCtr = false;

        msg = "Manual_Fine_Scan_Trip_";

        // return 0;

        CMDOutput("AS");

        if (Tilt_XC == 0 && Tilt_YC == 0 && Tilt_ZC == 0)
        {
          digitalWrite(X_DIR_Pin, Direction);
          delayMicroseconds(10);
          MotorCC_X = digitalRead(X_DIR_Pin);
          Scan_AllRange_TwoWay(XYZ, count, motorStepC * MotorStepRatio, stableDelay, MotorCC_X, delayBetweenStep, StopValueC, Get_PD_Points, Trips, " Fine-Scan, Trip_", SlopeC);
        }
        else
        {
          Line_Scan_3D(XYZ, count, motorStepC * MotorStepRatio, stableDelay,
                       Direction, delayBetweenStep, StopValueC, Get_PD_Points, Trips, msg,
                       SlopeC, Tilt_XC, Tilt_YC, Tilt_ZC);
        }

        CMDOutput("%:");

        isMotorManualCtr = true;

        // if (!)
        // {
        //   CMDOutput("AS");
        //   Scan_AllRange_TwoWay(XYZ, count, motorStepC, stableDelay,
        //                        Direction, delayBetweenStep, StopValueC, Get_PD_Points, Trips, msg);
        //   CMDOutput("%:");
        // }

        MSGOutput("Auto_Align_End");
        cmd_No = 0;

        return 0;

        break;

      // Circle Spiral
      case 199:

        int loops = 3;
        int motorStep = 50;
        int stableDelay = 0;
        int delay_btw_steps = 10;
        int StopPDValue = 2000;
        int Z_Layers = 1;
        int Z_Steps = 100;

        SpiralType spiType = rectangular;
        SpiralPlane spiPlane = YZ;
        int AngleSteps = 5;
        double ScanTilt_X = 0;
        double ScanTilt_Y = 0;
        double ScanTilt_Z = 0;

// Data analyze from txtcmd
#pragma region
        loops = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toInt();
        Serial.println("loops:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_'))); // loops
        Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);

        motorStep = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toInt();
        Serial.println("steps:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_'))); // steps
        Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);

        stableDelay = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toInt();
        Serial.println("stable delay:" + String(stableDelay)); // stable delay
        Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);

        delay_btw_steps = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toInt();
        Serial.println("delay_btw_puls:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_'))); // delay_btw_puls
        Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);

        StopPDValue = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toInt();
        Serial.println("stopValue:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_'))); // stopValue
        Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);

        Z_Layers = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toInt();
        Serial.println("Z_Layers:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_'))); // Z_Layers
        Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);

        Z_Steps = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toInt();
        Serial.println("Z_Steps:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_'))); // Z_Steps
        Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);

        if (Txt_SpiralSetting != "")
        {
          int stp = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toInt();
          Serial.println("spiral type:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_'))); // spiral type
          Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);

          if (stp == 0)
            spiType = rectangular;
          else if (stp == 1)
            spiType = smooth;
          else
            spiType = rectangular; // 預設方法
        }

        if (Txt_SpiralSetting != "")
        {
          String sprPlane = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_'));
          Serial.println("spiral plane:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_'))); // spiral type
          Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);

          if (sprPlane == "XY")
            spiPlane = XY;
          else if (sprPlane == "YZ")
            spiPlane = YZ;
          else if (sprPlane == "XZ")
            spiPlane = XZ;
          else
            spiPlane = XY; // 預設平面
        }

        if (Txt_SpiralSetting != "")
        {
          AngleSteps = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toInt();
          Serial.println("spiral angle gap:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')));
          Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);
        }

        if (Txt_SpiralSetting != "")
        {
          ScanTilt_X = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toDouble();
          Serial.println("spiral scan tilt x:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')));
          Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);
        }

        if (Txt_SpiralSetting != "")
        {
          ScanTilt_Y = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toDouble();
          Serial.println("spiral scan tilt y:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')));
          Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);
        }

        if (Txt_SpiralSetting != "")
        {
          ScanTilt_Z = Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')).toDouble();
          Serial.println("spiral scan tilt z:" + Txt_SpiralSetting.substring(0, Txt_SpiralSetting.indexOf('_')));
          Txt_SpiralSetting.remove(0, Txt_SpiralSetting.indexOf('_') + 1);
        }

        Txt_SpiralSetting = "";

        // return 0;

#pragma endregion

        // digitalWrite(Tablet_PD_mode_Trigger_Pin, false); // false is PD mode, true is Servo mode
        // delay(5);

        isMotorManualCtr = false;

#pragma region Rectangular Spiral

        if (spiType == rectangular)
        {
          M_Level = loops;

          CMDOutput("AS");
          Serial.println("Auto-Align Start");
          CMDOutput("^X");
          CMDOutput("R:" + String(M_Level * 2 + 1));
          CMDOutput("C:" + String(M_Level * 2 + 1));

          MinMotroStep = motorStep * MotorStepRatio; // 350
          delayBetweenStep = delay_btw_steps;

          if (Z_Layers > 1)
            sprial_JumpToBest = false;

          // isCheckStop = true;

          for (int ZL = 0; ZL < Z_Layers; ZL++)
          {
            if (ZL > 0)
            {
              Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, Z_Steps, 8, 150); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
            }

            AutoAlign_Spiral(M_Level, StopPDValue, stableDelay); // Input : (Sprial Level, Threshold, stable) Threshold:128
          }

          // isCheckStop = false;

          sprial_JumpToBest = true;

          CMDOutput("X^");
          // Serial.println("X^");

          isMotorManualCtr = true;

          return 0;
        }

#pragma endregion

#pragma region Smooth Spiral

        if (spiType == smooth)
        {
          CMDOutput("AS"); // 清除UI前一筆資訊
          MSGOutput("Spiral Start");

          Serial.println("Tilt x:" + String(ScanTilt_X));
          Serial.println("Tilt y:" + String(ScanTilt_Y));
          Serial.println("Tilt z:" + String(ScanTilt_Z));
          PD_Now = Cal_PD_Input_Row_Dac(1);
          // Serial.println("IL :" + String(PD_Now));
          Get_PD_Points = 1;
          Serial.println("Pos : " + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z) + "=" + String(PD_Now));

          CMDOutput("^X"); // 2D scan start msg for UI

          long R = 40;

          struct_Motor_Pos OriginalPos;
          OriginalPos.X = Pos_Now.X;
          OriginalPos.Y = Pos_Now.Y;
          OriginalPos.Z = Pos_Now.Z;

          struct_Motor_Pos BestPos;
          BestPos.X = Pos_Now.X;
          BestPos.Y = Pos_Now.Y;
          BestPos.Z = Pos_Now.Z;

          double BestIL = -80;

          // Initial Pos
          // Move_Motor(X_DIR_Pin, X_STP_Pin, true, R, 20, 100);

          // Target position Matrix
          BLA::Matrix<3, 1> M_A;

          // Rotation Matrix
          BLA::Matrix<3, 3> M_X;
          BLA::Matrix<3, 3> M_Y;
          BLA::Matrix<3, 3> M_Z;

          R = 0; // initial R
          double tx = ScanTilt_X * (PI / 180.0);
          double ty = ScanTilt_Y * (PI / 180.0);
          double tz = ScanTilt_Z * (PI / 180.0);

          M_X = {1, 0, 0, 0, cos(tx), -sin(tx), 0, sin(tx), cos(tx)}; // Rotation Matrix on X axis
          M_Y = {cos(ty), 0, sin(ty), 0, 1, 0, -sin(ty), 0, cos(ty)}; // Rotation Matrix on Y axis
          M_Z = {cos(tz), -sin(tz), 0, sin(tz), cos(tz), 0, 0, 0, 1};

          int ptsPerLoop = 360 / AngleSteps;
          long stpPerAngle = motorStep / ptsPerLoop;

          for (int i = 0; i <= (loops * 360); i = i + AngleSteps)
          {

            {
              long Targ_X = 0;
              long Targ_Y = 0;
              long Targ_Z = 0;

              // 分量計算
              if (spiPlane == XY)
              {
                Targ_X = R * (sin(i * (PI / 180.0)));
                Targ_Y = R * (cos(i * (PI / 180.0)));
              }
              else if (spiPlane == YZ)
              {
                Targ_Y = R * (cos(i * (PI / 180.0)));
                Targ_Z = R * (sin(i * (PI / 180.0)));
              }
              else if (spiPlane == XZ)
              {
                Targ_X = R * (cos(i * (PI / 180.0)));
                Targ_Z = R * (sin(i * (PI / 180.0)));
              }

              M_A = {Targ_X, Targ_Y, Targ_Z};

              // 旋轉
              BLA::Matrix<3, 1> M = M_X * M_Y * M_Z * M_A;

              Targ_X = M(0);
              Targ_Y = M(1);
              Targ_Z = M(2);

              // 加上初始點位移量
              Targ_X += OriginalPos.X;
              Targ_Y += OriginalPos.Y;
              Targ_Z += OriginalPos.Z;

              Move_Motor_abs_all(Targ_X, Targ_Y, Targ_Z, delay_btw_steps);

              if (stableDelay > 0)
                delay(stableDelay);

              PD_Now = Get_IL_DAC(1);

              if (PD_Now > BestIL)
              {
                BestIL = PD_Now;
                BestPos.X = Pos_Now.X;
                BestPos.Y = Pos_Now.Y;
                BestPos.Z = Pos_Now.Z;
              }

              if (PD_Now >= StopPDValue)
                break;

              if (i % 20 == 0)
              {
                CMDOutput("*[" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z) + "]=" + String(PD_Now));
                // MSGOutput("Ang:" + String(i) + ", Pos:" + String(Targ_X) + "," + String(Targ_Y) + "," + String(Targ_Z));
              }

              // break;

              R += stpPerAngle; // motorStep
            }

            if (isStop)
              break;
          }

          // Back to center position
          Serial.println("To best position");
          Move_Motor_abs_all(BestPos.X, BestPos.Y, BestPos.Z);

          PD_Now = Cal_PD_Input_Row_Dac(Get_PD_Points);
          Serial.println("Best : " + String(BestPos.X) + "," + String(BestPos.Y) + "," + String(BestPos.Z) + "=" + String(PD_Now));

          DataOutput(false);

          MSGOutput("Spiral End");

          isMotorManualCtr = true;

          // digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode

          return 0;
        }

#pragma endregion
        cmd_No = 0;

        break;
      }

      // vTaskResume(Task_1); // 回復 Task 1 的執行
      // isMotorCont = false;
    }
    cmd_from_contr = "";
    cmd_value_from_contr = "";
  }

  isCheckStop = false;

  return cmd_No;
}

//------------------------------------------------------------------------------------------------------------------------------------------

bool Motor_Feed_Mode_StopJudge()
{
  long deltaT = millis() - nowMillis_DataRec;

  if (abs(deltaT) > 230)
  {
    Serial.println("timeFeedStart:" + String(abs(deltaT)));
    return true;
  }

  if (cmd_from_contr != "BS")
  {
    Serial.printf("break:%s, %s\n", cmd_from_contr, cmd_value_from_contr);
    cmd_from_contr = "";

    return true;
  }

  return false;
}

//------------------------------------------------------------------------------------------------------------------------------

/// @brief 判斷是否收到緊急停止訊息
void CheckStop()
{
  if (isCheckStop && Serial.available())
  {
    String cmd = Serial.readString();

    // cmd_No = Function_Classification(cmd, ButtonSelected);
    // if (cmd_No == 30){
    //   isStop = true;

    // Serial.println("task:" + cmd);
    //   // EmergencyStop();
    // }
    if (Contains(cmd, "cmd30"))
    {
      isStop = true;

      Serial.println("EmergencyStop");
      // EmergencyStop();
    }
    else
    {
      cmd.replace("\r", "");
      cmd.replace("\n", "");
      cmd.trim();

      if (cmd == "0") // 馬達停止連續移動指令
      {
        isStop = true;
        Serial.println("Command 0");
      }
    }

    cmd = ""; // Reset command from serial port
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

long Get_Position(int xyz)
{
  switch (xyz)
  {
  case 0:
    return Pos_Now.X;
    break;

  case 1:
    return Pos_Now.Y;
    break;

  case 2:
    return Pos_Now.Z;
    break;
  }
}

struct_Motor_Pos Get_Position()
{
  struct_Motor_Pos smp;
  smp.X = Pos_Now.X;
  smp.Y = Pos_Now.Y;
  smp.Z = Pos_Now.Z;

  return smp;
}

long Get_1D_Position(struct_Motor_Pos Pos, int xyz)
{
  if (xyz == 0)
    return Pos.X;
  else if (xyz == 1)
    return Pos.Y;
  else if (xyz == 2)
    return Pos.Z;
}

bool Compare_Position(struct_Motor_Pos Pos1, struct_Motor_Pos Pos2)
{
  if (Pos1.X == Pos2.X && Pos1.Y == Pos2.Y && Pos1.Z == Pos2.Z)
    return true;
  else
    return false;
}

String Show_Position(struct_Motor_Pos Pos)
{
  return String(Pos.X) + "," + String(Pos.Y) + "," + String(Pos.Z);
}

bool isNumberic(String str)
{
  str.trim();
  unsigned int stringLength = str.length();

  if (stringLength == 0)
  {
    return false;
  }

  bool seenDecimal = false;
  bool seenMinus = false;

  for (unsigned int i = 0; i < stringLength; ++i)
  {
    if (isDigit(str.charAt(i)))
    {
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

bool isInteger(String str)
{
  if (str.length() == 0)
    return false;

  int startIdx = 0;
  if (str.charAt(0) == '-')
  {
    if (str.length() == 1)
      return false; // 單獨 "-" 不是整數
    startIdx = 1;   // 負數符號允許
  }

  for (int i = startIdx; i < str.length(); i++)
  {
    if (!isDigit(str.charAt(i)))
      return false;
  }

  return true;
}

/// @brief 判斷是否收到IL?訊息
void SendIL()
{
  if (isKeepGetIL && Serial.available())
  {
    String cmd = Serial.readString();

    // oldTime = millis();

    if (Contains(cmd, "IL?"))
    {
      switch (GetPower_Mode)
      {
      case 1:
        MSGOutput(String(Cal_PD_Input_IL(Get_PD_Points)));
        break;

      case 2:
        MSGOutput(String(Cal_PD_Input_Dac(Get_PD_Points)));
        break;

      case 3:
        MSGOutput(String(Cal_PD_Input_Row_IL(Get_PD_Points)));
        break;

      case 4:
        MSGOutput(String(Cal_PD_Input_Row_Dac(Get_PD_Points)));
        break;

      default:
        MSGOutput(String(Cal_PD_Input_IL(Get_PD_Points)));
        break;
      }
    }

    else if (Contains(cmd, "cmd30"))
    {
      isKeepGetIL = false;
      isMotorManualCtr = true;
      isCheckStop = true;
      isStop = true;
      Serial.println("EmergencyStop");
    }

    // currTime = millis();

    // Serial << "ts:"
    //        << (currTime - oldTime) << endl;
  }
}