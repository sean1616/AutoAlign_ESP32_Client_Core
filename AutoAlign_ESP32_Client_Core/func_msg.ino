#include "config.h"
#include "ep_adrs.h"
#include <esp_now.h>

void DataSent_Server(String MSG)
{
  MSG.toCharArray(sendmsg_server.msg, 30);
  esp_err_t result = esp_now_send(ServerAddress, (uint8_t *)&sendmsg_server, sizeof(sendmsg_server));
}

void CMDOutput(String cmd)
{
  String msg = "CMD::" + cmd;
  Serial.println(msg);

  if (isWiFiConnected)
    DataSent_Server(msg);
}

void CMDOutput(String cmd, bool isSentServer)
{
  String msg = "CMD::" + cmd;
  Serial.println(msg);

  if (isSentServer)
    DataSent_Server(msg);
}

void MSGOutput(String msg)
{
  Serial.println(msg);

  if (isWiFiConnected)
  {
    DataSent_Server(msg);
  }
}

// Output now XYZ position and IL
void DataOutput()
{
  if (isGetPower)
  {
    double IL = Cal_PD_Input_IL(Get_PD_Points);
    Serial.println("Position:" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z) + "," + String(IL));
  }
  else
  {
    Serial.println("Position:" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z) + ",0");
  }
}

// Output now XYZ position and IL
void DataOutput(double IL)
{
  Serial.println("Position:" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z) + "," + String(IL));
}

// Output now XYZ position (IL))
void DataOutput(bool isIL)
{
  if (isIL && isGetPower)
  {
    double IL = Cal_PD_Input_IL(Get_PD_Points);
    MSGOutput("Position:" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z) + "," + String(IL));
  }
  else
  {
    MSGOutput("Position:" + String(Pos_Now.X) + "," + String(Pos_Now.Y) + "," + String(Pos_Now.Z));
  }
}

// Output now one axis position and IL
void DataOutput(int xyz, double pdValue)
{
  switch (xyz)
  {
  case 0:
    CMDOutput(">:" + String(Pos_Now.X) + "," + String(pdValue));
    break;

  case 1:
    CMDOutput(">:" + String(Pos_Now.Y) + "," + String(pdValue));
    break;

  case 2:
    CMDOutput(">:" + String(Pos_Now.Z) + "," + String(pdValue));
    break;
  }
}
