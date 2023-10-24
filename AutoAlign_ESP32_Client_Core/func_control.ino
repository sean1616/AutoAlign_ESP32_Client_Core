#include "config.h"
#include "Arduino.h"

bool MotorCC_A = false;
bool MotorCC_X = false;
bool MotorCC_Y = false;
bool MotorCC_Z = false;

// Dac to dBm
double ILConverter(double pdDac)
{
  double IL = 0;

  if (pdDac >= PD_Ref_Array[0][0])
    return -3;
  else if (pdDac < PD_Ref_Array[14][0])
    return -50;

  for (size_t i = 1; i < 15; i++)
  {
    if (pdDac >= PD_Ref_Array[i][0])
    {
      IL = ((pdDac - PD_Ref_Array[i][0]) / (PD_Ref_Array[i - 1][0] - PD_Ref_Array[i][0]) * (PD_Ref_Array[i - 1][1] - PD_Ref_Array[i][1])) + PD_Ref_Array[i][1];
      break;
    }
  }

  return IL;
}

/// @brief Calculate PD input value, Return Dac
/// @param averageCount
/// @return
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

  // Function: (PD Value) - (reference) + 300
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  return (averagePDInput - ref_Dac);
}

/// @brief Calculate PD input value, Return IL
/// @param averageCount
/// @return
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

/// @brief Calculate PD input value, Return Row Dac
/// @param averageCount
/// @return
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
  // Function: (PD Value)
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  double IL = ILConverter(averagePDInput);

  return IL;
}

/// @brief Calculate PD input value, Return Row Dac
/// @param averageCount
/// @return
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

double Get_IL_DAC(int averageCount)
{
  switch (GetPower_Mode)
  {
  case 1:
    return Cal_PD_Input_IL(averageCount);
    break;

  case 2:
    return Cal_PD_Input_Dac(averageCount);
    break;

  case 3:
    return Cal_PD_Input_Row_IL(averageCount);
    break;

  case 4:
    return Cal_PD_Input_Row_Dac(averageCount);
    break;

  default:
    return Cal_PD_Input_IL(averageCount);
    break;
  }
}

void EmergencyStop()
{
  isStop = true;

  Serial.println("EmergencyStop");
  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); // false is PD mode, true is Servo mode
}

void step(byte stepperPin, long steps, int delayTime)
{
  steps = abs(steps);

  for (long i = 0; i < steps; i++)
  {
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(delayTime);

    if (isStop)
    {
      steps -= (i + 1);
      break;
    }
  }

  bool dirc = false;

  if (stepperPin == X_STP_Pin)
  {
    dirc = digitalRead(X_DIR_Pin);
  }
  else if (stepperPin == Y_STP_Pin)
  {
    dirc = digitalRead(Y_DIR_Pin);
  }
  else if (stepperPin == Z_STP_Pin)
  {
    dirc = digitalRead(Z_DIR_Pin);
  }

  // Position Record
  if (dirc)
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      Pos_Now.X += steps;
      MotorCC_X = true;
      X_Pos_Now = Pos_Now.X;
      break;
    case Y_STP_Pin:
      Pos_Now.Y += steps;
      MotorCC_Y = true;
      Y_Pos_Now = Pos_Now.Y;
      break;
    case Z_STP_Pin:
      Pos_Now.Z += steps;
      MotorCC_Z = true;
      Z_Pos_Now = Pos_Now.Z;
      break;
    }
  }
  else
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      Pos_Now.X -= steps;
      MotorCC_X = false;
      X_Pos_Now = Pos_Now.X;
      break;
    case Y_STP_Pin:
      Pos_Now.Y -= steps;
      MotorCC_Y = false;
      Y_Pos_Now = Pos_Now.Y;
      break;
    case Z_STP_Pin:
      Pos_Now.Z -= steps;
      MotorCC_Z = false;
      Z_Pos_Now = Pos_Now.Z;
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

    if (isStop)
    {
      steps -= (i + 1);
      break;
    }
  }

  bool dirc = false;

  if (stepperPin == X_STP_Pin)
  {
    dirc = digitalRead(X_DIR_Pin);
  }
  else if (stepperPin == Y_STP_Pin)
  {
    dirc = digitalRead(Y_DIR_Pin);
  }
  else if (stepperPin == Z_STP_Pin)
  {
    dirc = digitalRead(Z_DIR_Pin);
  }

  // Position Record
  if (dirc)
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      Pos_Now.X += steps;
      MotorCC_X = true;
      break;
    case Y_STP_Pin:
      Pos_Now.Y += steps;
      MotorCC_Y = true;
      break;
    case Z_STP_Pin:
      Pos_Now.Z += steps;
      MotorCC_Z = true;
      break;
    }
  }
  else
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      Pos_Now.X -= steps;
      MotorCC_X = false;
      break;
    case Y_STP_Pin:
      Pos_Now.Y -= steps;
      MotorCC_Y = false;
      break;
    case Z_STP_Pin:
      Pos_Now.Z -= steps;
      MotorCC_Z = false;
      break;
    }
  }
}

void Move_Motor(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay, bool isOutputPosition, int pinDelay)
{
  if (moveSteps > 0)
  {
    // MotorCC_A = dirt;
    digitalWrite(dir_pin, dirt);
    delay(pinDelay);

    step(stp_pin, moveSteps, delayStep);
    delay(stableDelay);

    if (isOutputPosition)
    {
      delay(3); // 一定要delay，可避免記憶體存取衝突
      DataOutput(false);
    }
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

void Move_Motor_abs(int xyz, long Target)
{
  long Pos_N = 0;
  switch (xyz)
  {
  case 0:
    MotorDir_Pin = X_DIR_Pin;
    MotorSTP_Pin = X_STP_Pin;
    Pos_N = Pos_Now.X;
    delayBetweenStep = delayBetweenStep_X;
    break;
  case 1:
    MotorDir_Pin = Y_DIR_Pin;
    MotorSTP_Pin = Y_STP_Pin;
    Pos_N = Pos_Now.Y;
    delayBetweenStep = delayBetweenStep_Y;
    break;
  case 2:
    MotorDir_Pin = Z_DIR_Pin;
    MotorSTP_Pin = Z_STP_Pin;
    Pos_N = Pos_Now.Z;
    delayBetweenStep = delayBetweenStep_Z;
    break;
  }

  MinMotroStep = Target - Pos_N;

  bool dir = false;

  if (MinMotroStep < 0)
    dir = false;
  else if (MinMotroStep > 0)
    dir = true;
  else
    return;

  MinMotroStep = abs(MinMotroStep);

  Move_Motor(MotorDir_Pin, MotorSTP_Pin, dir, MinMotroStep, delayBetweenStep, 0, false, 8);
}

void Move_Motor_abs_async(struct_Motor_Pos TargetPos, int DelayT)
{
  long Delta_X = TargetPos.X - Pos_Now.X;
  long Delta_Y = TargetPos.Y - Pos_Now.Y;
  long Delta_Z = TargetPos.Z - Pos_Now.Z;

  // 判斷方向
  if (Delta_X > 0)
    MotorCC_X = true;
  else if (Delta_X < 0)
    MotorCC_X = false;

  if (Delta_Y > 0)
    MotorCC_Y = true;
  else if (Delta_Y < 0)
    MotorCC_Y = false;

  if (Delta_Z > 0)
    MotorCC_Z = true;
  else if (Delta_Z < 0)
    MotorCC_Z = false;

  // 設定軸方向
  if (Delta_X != 0)
    digitalWrite(X_DIR_Pin, MotorCC_X);

  if (Delta_Y != 0)
    digitalWrite(Y_DIR_Pin, MotorCC_Y);

  if (Delta_Z != 0)
    digitalWrite(Z_DIR_Pin, MotorCC_Z);

  delay(3);

  // 計算分時時間
  byte MoveAxisCount = 0;

  bool is_X = false;
  bool is_Y = false;
  bool is_Z = false;

  if (Delta_X != 0)
  {
    MoveAxisCount++;
    is_X = true;
  }

  if (Delta_Y != 0)
  {
    MoveAxisCount++;
    is_Y = true;
  }

  if (Delta_Z != 0)
  {
    MoveAxisCount++;
    is_Z = true;
  }

  if (MoveAxisCount == 0)
    return;

  delayBetweenStep = DelayT / MoveAxisCount;

  // 分時多工

  Delta_X = abs(Delta_X);
  Delta_Y = abs(Delta_Y);
  Delta_Z = abs(Delta_Z);

  double max = (Delta_X > Delta_Y) ? ((Delta_X > Delta_Z) ? Delta_X : Delta_Z) : ((Delta_Y > Delta_Z) ? Delta_Y : Delta_Z);

  for (size_t i = 0; i < max; i++)
  {
    if (is_X)
    {
      digitalWrite(X_STP_Pin, HIGH);
      delayMicroseconds(delayBetweenStep);
      digitalWrite(X_STP_Pin, LOW);
      delayMicroseconds(delayBetweenStep);

      Delta_X--;
      if (Delta_X == 0)
        is_X = false;
    }

    if (is_Y)
    {
      digitalWrite(Y_STP_Pin, HIGH);
      delayMicroseconds(delayBetweenStep);
      digitalWrite(Y_STP_Pin, LOW);
      delayMicroseconds(delayBetweenStep);

      Delta_Y--;
      if (Delta_Y == 0)
        is_Y = false;
    }

    if (is_Z)
    {
      digitalWrite(Z_STP_Pin, HIGH);
      delayMicroseconds(delayBetweenStep);
      digitalWrite(Z_STP_Pin, LOW);
      delayMicroseconds(delayBetweenStep);

      Delta_Z--;
      if (Delta_Z == 0)
        is_Z = false;
    }
  }

  // // Position Record
  Pos_Now.X = TargetPos.X;
  Pos_Now.Y = TargetPos.Y;
  Pos_Now.Z = TargetPos.Z;
}

/// @brief 多軸馬達同動
/// @param TargetPos 目標絕對位置
/// @param DelayT 步進時間間隔
void Move_Motor_abs_sync(struct_Motor_Pos TargetPos, int DelayT)
{
  long Delta_X = TargetPos.X - Pos_Now.X;
  long Delta_Y = TargetPos.Y - Pos_Now.Y;
  long Delta_Z = TargetPos.Z - Pos_Now.Z;

  bool DIRX = false;
  bool DIRY = false;
  bool DIRZ = false;

  // 判斷方向
  if (Delta_X > 0)
    DIRX = true;
  else if (Delta_X < 0)
    DIRX = false;

  if (Delta_Y > 0)
    DIRY = true;
  else if (Delta_Y < 0)
    DIRY = false;

  if (Delta_Z > 0)
    DIRZ = true;
  else if (Delta_Z < 0)
    DIRZ = false;

  // 設定軸方向
  if (Delta_X != 0)
    digitalWrite(X_DIR_Pin, DIRX);

  if (Delta_Y != 0)
    digitalWrite(Y_DIR_Pin, DIRY);

  if (Delta_Z != 0)
    digitalWrite(Z_DIR_Pin, DIRZ);

  // delayMicroseconds(300);
  // delay(3);

  // 計算分時時間
  byte MoveAxisCount = 0;

  bool is_X = false;
  bool is_Y = false;
  bool is_Z = false;

  if (Delta_X != 0)
  {
    MoveAxisCount++;
    is_X = true;
  }

  if (Delta_Y != 0)
  {
    MoveAxisCount++;
    is_Y = true;
  }

  if (Delta_Z != 0)
  {
    MoveAxisCount++;
    is_Z = true;
  }

  if (MoveAxisCount == 0)
  {
    MSGOutput("Delta Distance All 0");
    return;
  }

  // 分時多工
  Delta_X = abs(Delta_X);
  Delta_Y = abs(Delta_Y);
  Delta_Z = abs(Delta_Z);

  // 計算最大的Delta 值
  long max = (Delta_X > Delta_Y) ? ((Delta_X > Delta_Z) ? Delta_X : Delta_Z) : ((Delta_Y > Delta_Z) ? Delta_Y : Delta_Z);

  int ratio_X = Delta_X == 0 ? 1 : (max / Delta_X);
  int ratio_Y = Delta_Y == 0 ? 1 : (max / Delta_Y);
  int ratio_Z = Delta_Z == 0 ? 1 : (max / Delta_Z);

  int ratio_max = (ratio_X > ratio_Y) ? ((ratio_X > ratio_Z) ? ratio_X : ratio_Z) : ((ratio_Y > ratio_Z) ? ratio_Y : ratio_Z);

  int Count_Step = 1;

  bool is_X_Temp = false;
  bool is_Y_Temp = false;
  bool is_Z_Temp = false;

  // int MinDelay = DelayT;

  for (size_t i = 0; i < max; i++)
  {
    is_X_Temp = Count_Step % ratio_X == 0 ? true : false;
    is_Y_Temp = Count_Step % ratio_Y == 0 ? true : false;
    is_Z_Temp = Count_Step % ratio_Z == 0 ? true : false;

    if (is_X && is_X_Temp)
      digitalWrite(X_STP_Pin, HIGH);
    if (is_Y && is_Y_Temp)
      digitalWrite(Y_STP_Pin, HIGH);
    if (is_Z && is_Z_Temp)
      digitalWrite(Z_STP_Pin, HIGH);

    delayMicroseconds(DelayT);

    if (is_X && is_X_Temp)
    {
      digitalWrite(X_STP_Pin, LOW);

      Delta_X--;
      if (Delta_X == 0)
        is_X = false;
    }

    if (is_Y && is_Y_Temp)
    {
      digitalWrite(Y_STP_Pin, LOW);

      Delta_Y--;
      if (Delta_Y == 0)
        is_Y = false;
    }

    if (is_Z && is_Z_Temp)
    {
      digitalWrite(Z_STP_Pin, LOW);

      Delta_Z--;
      if (Delta_Z == 0)
        is_Z = false;
    }

    delayMicroseconds(DelayT);

    Count_Step++;
    if (Count_Step > ratio_max)
      Count_Step = Count_Step % ratio_max;

    if (isStop)
      return;
  }

  // // Position Record
  Pos_Now.X = TargetPos.X;
  Pos_Now.Y = TargetPos.Y;
  Pos_Now.Z = TargetPos.Z;
}

void Move_Motor_abs_all(long x, long y, long z, int DelayT)
{
  struct_Motor_Pos TargetPos;
  TargetPos.X = x;
  TargetPos.Y = y;
  TargetPos.Z = z;
  Move_Motor_abs_sync(TargetPos, DelayT);

  // MSGOutput("Move All Abs End");
}

void Move_Motor_abs_all(int x, int y, int z, bool IsMsg, int DelayT)
{
  MSGOutput("Move All Abs Sync Start");

  struct_Motor_Pos TargetPos;
  TargetPos.X = x;
  TargetPos.Y = y;
  TargetPos.Z = z;
  Move_Motor_abs_sync(TargetPos, DelayT);

  if (IsMsg)
    MSGOutput("Move All Abs End");
}

void Move_Motor_Cont(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep)
{
  MotorSTP_Pin = dir_pin;

  if (MotorDir_Pin != dir_pin || MotorCC_A != dirt)
  {
    MotorCC_A = dirt;
    MotorDir_Pin = dir_pin;
    digitalWrite(MotorDir_Pin, MotorCC_A); // 步進馬達方向控制, false為負方向
    delay(3);
  }

  step(stp_pin, moveSteps, delayStep);
}

//------------------------------------------------------------------------------------------------------------------------------------------
