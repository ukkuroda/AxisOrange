#include <M5Unified.h>
#include <BluetoothSerial.h>
#include "imu/ImuReader.h"
#include "imu/AverageCalc.h"
#include "input/ButtonCheck.h"
#include "input/ButtonData.h"
#include "session/SessionData.h"
#include "prefs/Settings.h"
#include "device_name/DeviceName.h"
#include "SPIFFS.h"

#define TASK_DEFAULT_CORE_ID 1
#define TASK_STACK_DEPTH 4096UL
#define TASK_NAME_IMU "IMUTask"
#define TASK_NAME_WRITE_SESSION "WriteSessionTask"
#define TASK_NAME_READ_SESSION "ReadSessionTask"
#define TASK_NAME_BUTTON "ButtonTask"
#define TASK_SLEEP_IMU 5            // = 1000[ms] / 200[Hz]
#define TASK_SLEEP_WRITE_SESSION 40 // = 1000[ms] / 25[Hz]
#define TASK_SLEEP_READ_SESSION 100 // = 1000[ms] / 10[Hz]
#define TASK_SLEEP_BUTTON 1         // = 1000[ms] / 1000[Hz]
#define MUTEX_DEFAULT_WAIT 1000UL

static void ImuLoop(void *arg);
static void WriteSessionLoop(void *arg);
static void ReadSessionLoop(void *arg);
static void ButtonLoop(void *arg);

imu::ImuReader *imuReader;
BluetoothSerial btSpp;
input::ButtonCheck button;

imu::ImuData imuData;
input::ButtonData btnData;
bool hasButtonUpdate = false;
static SemaphoreHandle_t imuDataMutex = NULL;
static SemaphoreHandle_t btnDataMutex = NULL;

uint8_t readBuffer[session::data_length::max] = {0};
bool gyroOffsetInstalled = true;
imu::AverageCalcXYZ gyroAve;
prefs::Settings settingPref;
device_name::DeviceName deviceName("AxisOrange");

// axisorangeを修正、センサーとしては機能するようになった。ところが！！！
// 音が出るようにlibraryを出し入れしていたら、エラーが出るようになり、元に戻せなくなった。
// ソースコード保存のためメールしている。
int pitch = 0, roll = 0, yaw = 0;
void QuaternionToEuler(float q0, float q1, float q2, float q3)

// void QuaternionToEuler(float q3, float q2, float q0, float q1)
{ //, float& pitch, float& roll, float& yaw) {
  float pitchf, rollf, yawf;
  pitchf = asin(-2 * q1 * q3 + 2 * q0 * q2);                                    // pitch
  rollf = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1);     // roll
  yawf = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3); // yaw

  pitchf *= RAD_TO_DEG;
  yawf *= RAD_TO_DEG;
  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  //     8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  // yawf -= 8.5;
  //yawf *= 1.2;
  rollf *= RAD_TO_DEG;
  pitch = int(pitchf);
  roll = int(rollf);
  yaw = int(yawf);
}

// int pitchA[1000];
int mode = 0; // 0:running 1:pitchlimit 2:rolllimit 3:yawlimit
int gainValue = 1;
int pitchLimit = 30;
int rollLimit = 30;
int yawLimit = 30;
int pitchA[4];
int rollA[4];
int yawA[4];
int millA[4];
int getDirection(int a, int b, int c, int d)
{
  if (a > 150)
  { // 150以上回転して-になることはないとして、30以上回転して-になったとき
    if (b < 0)
      b += 360; // yaw += 1.2 360->430
    if (c < 0)
      c += 360;
    if (d < 0)
      c += 360;
  }
  else if (a < -150)
  {
    if (b > 0)
      b -= 360;
    if (c > 0)
      c -= 360;
    if (d > 0)
      c -= 360;
  } // 以上は yawのためのチェック
  if ((a < b) && (b < c) && (c < d))
    return 1;
  if ((a > b) && (b > c) && (c > d))
    return -1;
  return 0;
}
// int pitchArray[1000];
// int milliArray[1000];
int okPitchnum = 0;
int okPitch[1000];
int okLastPitch[1000];
int okPitchMilli[1000];
int lastPitch = 0;
int lastPitchmilli = 0;
int pitchDirection = 0;
int okRollnum = 0;
int okRoll[1000];
int okLastRoll[1000];
int okRollMilli[1000];
int lastRoll = 0;
int lastRollmilli = 0;
int rollDirection = 0;
int okYawnum = 0;
int okYaw[1000];
int okLastYaw[1000];
int okYawMilli[1000];
int lastYaw = 0;
int lastYawmilli = 0;
int yawDirection = 0;
int cnt = 0;
int checkOK(int degree0, int degree1, int limit, int millis)
{ // 120ms以下,10度以下なら0
  int d = degree0 - degree1;
  if (d > 200) // yaw 160  -160 = 320 だが実は-40
    d -= 360;
  else if (d < -200) // yaw -160 160 = -320だが実は40
    d += 360;
  if (millis < 200)
    return 0;
  if (d > limit || d < -limit)
  {
    if (millis < 500)
      return 5; // 30度以上１秒以内
    if (millis < 1000)
      return 5;
  }
  else if ((d > 10) || (d < -10))
  {
    if (millis < 500)
      return 3;
    if (millis < 1000)
      return 2;
  }
  return 0;
}
void pushPitchRollYawMill(int p, int r, int y, int m)
{
  for (int i = 0; i < 3; i++)
  {
    pitchA[i] = pitchA[i + 1];
    rollA[i] = rollA[i + 1];
    yawA[i] = yawA[i + 1];
    millA[i] = millA[i + 1];
  }
  pitchA[3] = p;
  rollA[3] = r;
  yawA[3] = y;
  millA[3] = m;
}
boolean soundFlag = false;

int debugFlag = 0;
int debugData[150];
void UpdateScreenDebug()
{
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextSize(1);
  for (int i = 0; i < 150; i++)
  {
    if (i % 10 == 0 && i != 0)
      M5.Lcd.println();
    M5.Lcd.printf("%03d ", debugData[i]);
  }
}
void dispOKs(){
  M5.Lcd.setCursor(0, 0);
//  M5.Lcd.setTextSize(1);
   M5.Lcd.fillScreen(BLACK); 
  M5.Lcd.print("P)");
  for(int i=0;i<okPitchnum;i++){
    M5.Lcd.printf("%03d:%03d:%03d ", okLastPitch[i],okPitch[i],okPitchMilli[i]/100%1000);
  }

  M5.Lcd.print("R)");
  for(int i=0;i<okRollnum;i++){
    M5.Lcd.printf("%03d:%03d:%03d ", okLastRoll[i],okRoll[i],okRollMilli[i]/100%1000);
  }
   M5.Lcd.print("Y)");
  for(int i=0;i<okYawnum;i++){
    M5.Lcd.printf("%03d:%03d:%03d ", okLastYaw[i], okYaw[i],okYawMilli[i]/100%1000);
  }
  
}
void UpdateScreen()
{
  // if(debugFlag==1)return;
  int mill = millis();
  int tempdirection;
  pushPitchRollYawMill(pitch, roll, yaw, mill);

  // pitch
  tempdirection = getDirection(pitchA[0], pitchA[1], pitchA[2], pitchA[3]);
  if (tempdirection == -1 && pitchDirection == 1)
  {
    pitchDirection = -1;
    if (checkOK(lastPitch, pitchA[0], pitchLimit, millA[0] - lastPitchmilli) == 5)
    {
      okLastPitch[okPitchnum] = lastPitch;
      okPitch[okPitchnum] = pitchA[0];
      okPitchMilli[okPitchnum] = millA[0];
      okPitchnum++;
      soundFlag = true;
    }
    lastPitch = pitchA[0];
    lastPitchmilli = millA[0];
  }
  else if (tempdirection == 1 && pitchDirection == -1)
  {
    pitchDirection = 1;
    if (checkOK(lastPitch, pitchA[0], pitchLimit, millA[0] - lastPitchmilli) == 5)
    {
       okLastPitch[okPitchnum] = lastPitch;
      okPitch[okPitchnum] = pitchA[0];
      okPitchMilli[okPitchnum] = millA[0];
      okPitchnum++;
      soundFlag = true;
    }
    lastPitch = pitchA[0];
    lastPitchmilli = millA[0];
  }
  if (tempdirection != 0)
  {
    pitchDirection = tempdirection;
  }
  // roll
  tempdirection = getDirection(rollA[0], rollA[1], rollA[2], rollA[3]);
  if (tempdirection == -1 && rollDirection == 1)
  {
    rollDirection = -1;
    if (checkOK(lastRoll, rollA[0], rollLimit, millA[0] - lastRollmilli) == 5)
    {
       okLastRoll[okRollnum] = lastRoll;
      okRoll[okRollnum] = rollA[0];
      okRollMilli[okRollnum] = millA[0];
      okRollnum++;
      soundFlag = true;
    }
    lastRoll = rollA[0];
    lastRollmilli = millA[0];
  }
  else if (tempdirection == 1 && rollDirection == -1)
  {
    rollDirection = 1;
    if (checkOK(lastRoll, rollA[0], rollLimit, millA[0] - lastRollmilli) == 5)
    {
       okLastRoll[okRollnum] = lastRoll;
      okRoll[okRollnum] = rollA[0];
      okRollMilli[okRollnum] = millA[0];
      okRollnum++;
      soundFlag = true;
    }
    lastRoll = rollA[0];
    lastRollmilli = millA[0];
  }
  if (tempdirection != 0)
  {
    rollDirection = tempdirection;
  }
  // yaw
  tempdirection = getDirection(yawA[0], yawA[1], yawA[2], yawA[3]);
 // if (pitchA[0] < 60 && pitchA[0] > -60 && rollA[0] < 60 && rollA[0] > -60)
  //{
    if (tempdirection == -1 && yawDirection == 1)
    {
      yawDirection = -1;
      if (checkOK(lastYaw, yawA[0], yawLimit, millA[0] - lastYawmilli) == 5)
      {
         okLastYaw[okYawnum] = lastYaw;
        okYaw[okYawnum] = yawA[0];
        okYawMilli[okYawnum] = millA[0];
        okYawnum++;
        soundFlag = true;
      }
      lastYaw = yawA[0];
      lastYawmilli = millA[0];
    }
    else if (tempdirection == 1 && yawDirection == -1)
    {
      yawDirection = 1;
      if (checkOK(lastYaw, yawA[0], yawLimit, millA[0] - lastYawmilli) == 5)
      {
          okLastYaw[okYawnum] = lastYaw;
        okYaw[okYawnum] = yawA[0];
        okYawMilli[okYawnum] = millA[0];
        okYawnum++;
        soundFlag = true;
      }
      lastYaw = yawA[0];
      lastYawmilli = millA[0];
    }
    if (tempdirection != 0)
    {
      yawDirection = tempdirection;
    }
  //}
  //else
  //{
   // yawDirection = tempdirection;
  //}
  // static int cnt = 0;
  // if (cnt++ % 10 != 0)
  //   return;
  if(mode==4){
    dispOKs();
    return;
  }
  if (mode < 1)
  {
    M5.Lcd.setCursor(0, 30);
    M5.Lcd.printf("pitch:%03d:%03d:%04d ", okPitchnum, pitchLimit, pitch);
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.printf("roll :%03d:%03d:%04d ", okRollnum, rollLimit, roll);
    M5.Lcd.setCursor(0, 70);
    M5.Lcd.printf("yaw  :%03d:%03d:%04d ", okYawnum, yawLimit, yaw);
  }
  M5.Lcd.setTextColor(DARKGREY, BLACK);
  if (mode == 0)
  {
    M5.Lcd.setCursor(0, 100);
    M5.Lcd.printf("M5 but:Reset count");
    M5.Lcd.setCursor(0, 118);
    M5.Lcd.printf("M5 >5sec:Reset ALL ");
  }
  else
  {
    M5.Lcd.setCursor(0, 118);
    M5.Lcd.printf("M5 >2sec:chan. gain");
    M5.Lcd.setCursor(0, 100);
    M5.Lcd.printf("M5 but:inc. limit  ");
  }
  M5.Lcd.setTextColor(DARKGREEN, BLACK);
  if (mode == 1)
  {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(120, 30);
    M5.Lcd.printf("%03d", pitchLimit);
  }
  else if (mode == 2)
  {
    M5.Lcd.setCursor(120, 30);
    M5.Lcd.printf("%03d", pitchLimit);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(120, 50);
    M5.Lcd.printf("%03d", rollLimit);
  }
  else if (mode == 3)
  {
    M5.Lcd.setCursor(120, 50);
    M5.Lcd.printf("%03d", rollLimit);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(120, 70);
    M5.Lcd.printf("%03d", yawLimit);
  }
  if (mode > 0)
    M5.Lcd.setTextColor(DARKGREEN, BLACK);
}

void UpdateLcd()
{
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(2);
  if (gyroOffsetInstalled)
  {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(DARKGREY, BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Top but:change mode");
  }
  else
  {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Setting GyroOffset ");
  }
}

void setup()
{
  M5.begin();
  Serial.begin(115200);
  // SPIFFS.format();//軌道に非常に時間がかかる

  float gyroOffset[3] = {0.0F};
  settingPref.begin();
  // settingPref.clear(); // to reinstall gyro offset by only m5stickc remove commentout
  settingPref.readGyroOffset(gyroOffset);
  settingPref.finish();
  // lcd
  pinMode(GPIO_NUM_10, OUTPUT);
  digitalWrite(GPIO_NUM_10, HIGH);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);
  //UpdateLcd();
  M5.Lcd.setTextColor(DARKGREY, BLACK);
      M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Top but:change mode");
  M5.Lcd.setCursor(0, 100);
  M5.Lcd.printf("M5 but:Reset count");
  M5.Lcd.setCursor(0, 118);
  M5.Lcd.printf("M5 >5sec:Reset ALL ");
  M5.Lcd.setTextColor(DARKGREEN, BLACK);
  digitalWrite(GPIO_NUM_10, HIGH); // 2回実行すると消える？
  // M5.update();
  // M5.Speaker.setVolume(4);//音量を設定すると無音になる？
  // M5.Speaker.setVolume(5);
  // M5.update();
  //   imu
  imuReader = new imu::ImuReader(M5.Imu);
  imuReader->initialize();
  if (gyroOffsetInstalled)
  {
    imuReader->writeGyroOffset(gyroOffset[0], gyroOffset[1], gyroOffset[2]);
  }
  // bluetooth serial
  btSpp.begin(deviceName.getName(M5.getBoard()));
  // task
  imuDataMutex = xSemaphoreCreateMutex();
  btnDataMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(ImuLoop, TASK_NAME_IMU, TASK_STACK_DEPTH,
                          NULL, 2, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(WriteSessionLoop, TASK_NAME_WRITE_SESSION, TASK_STACK_DEPTH,
                          NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(ReadSessionLoop, TASK_NAME_READ_SESSION, TASK_STACK_DEPTH,
                          NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(ButtonLoop, TASK_NAME_BUTTON, TASK_STACK_DEPTH,
                          NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
/*

      imuReader->update();
      imuReader->read(imuData);
      if (!gyroOffsetInstalled)
      {
        if (!gyroAve.push(imuData.gyro[0], imuData.gyro[1], imuData.gyro[2]))
        {
 //         float x = 0.54F;//gyroAve.averageX();
  //        float y = -8.6F;//gyroAve.averageY();
  //        float z = -12.7F;//gyroAve.averageZ();
       float x = gyroAve.averageX();
          float y = gyroAve.averageY();
          float z = //gyroAve.averageZ();
          // set offset
          imuReader->writeGyroOffset(x, y, z);//(0.54F,-8.6F,-12.7F);

 // M5.Lcd.setCursor(0, 0);
 //   M5.Lcd.printf("%.2f:%0.2f:%.2f ", x,y,z);//okYawnum, yawLimit, yaw);


          // save offset
          float offset[] = {x, y, z};
          settingPref.begin();
          settingPref.writeGyroOffset(offset);
          settingPref.finish();
          gyroOffsetInstalled = true;
          gyroAve.reset();
          UpdateLcd();
        }

*/

}

void loop()
{ /* Do Nothing */
  static int cnt;
  if (soundFlag)
  {
    // static int gain=0;
    soundFlag = false;
    // M5.update();
    // M5.Speaker.setVolume(3);//音量を設定すると無音になる？
    // M5.Speaker.setVolume(5);
    // M5.update();
    // M5.Speaker.getVolume()

    M5.Speaker.tone(3500, 50);
    // M5.Lcd.setCursor(0, 100);
    // M5.Lcd.printf("Hz:%d",1000+gain*10);
    delay(100);
    digitalWrite(GPIO_NUM_10, LOW);
    delay(100);
    digitalWrite(GPIO_NUM_10, HIGH);

    // soundFlag=false;
    // soundFlag = false;
  }
}

static void ImuLoop(void *arg)
{
  while (1)
  {
    uint32_t entryTime = millis();
    if (xSemaphoreTake(imuDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE)
    {
      imuReader->update();
      imuReader->read(imuData);
      if (!gyroOffsetInstalled)
      {
        if (!gyroAve.push(imuData.gyro[0], imuData.gyro[1], imuData.gyro[2]))
        {
          float x = gyroAve.averageX();
          float y = gyroAve.averageY();
          float z = gyroAve.averageZ();
          // set offset
          imuReader->writeGyroOffset(x, y, z);//0.54,-8.6,-12.7

          // save offset
          float offset[] = {x, y, z};
          settingPref.begin();
          settingPref.writeGyroOffset(offset);
          settingPref.finish();
          gyroOffsetInstalled = true;
          gyroAve.reset();
          UpdateLcd();
        }
      }
    }
    xSemaphoreGive(imuDataMutex);
    // idle
    int32_t sleep = TASK_SLEEP_IMU - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
    static int count = 0;
    if (count++ % 8 == 0) //
      UpdateScreen();     // 40ms
  }
}

static void WriteSessionLoop(void *arg)
{
  static session::SessionData imuSessionData(session::DataDefineImu);
  static session::SessionData btnSessionData(session::DataDefineButton);
  while (1)
  {
    uint32_t entryTime = millis();
    // imu
    if (gyroOffsetInstalled)
    {
      if (xSemaphoreTake(imuDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE)
      {
        imuSessionData.write((uint8_t *)&imuData, imu::ImuDataLen);
        btSpp.write((uint8_t *)&imuSessionData, imuSessionData.length());
        QuaternionToEuler(imuData.quat[0], imuData.quat[1], imuData.quat[2], imuData.quat[3]); // 加えた20240410
      }
      xSemaphoreGive(imuDataMutex);
    }
    // button
    if (xSemaphoreTake(btnDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE)
    {
      if (hasButtonUpdate)
      {
        btnSessionData.write((uint8_t *)&btnData, input::ButtonDataLen);
        btSpp.write((uint8_t *)&btnSessionData, btnSessionData.length());
        hasButtonUpdate = false;
      }
      xSemaphoreGive(btnDataMutex);
    }
    // idle
    int32_t sleep = TASK_SLEEP_WRITE_SESSION - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void ReadSessionLoop(void *arg)
{
 
  while (1)
  {
    uint32_t entryTime = millis();
    // read
    size_t len = btSpp.readBytes(readBuffer, session::data_length::header);
    if (len == (size_t)session::data_length::header)
    {
      uint16_t dataId = (uint16_t)((readBuffer[1] << 8) | readBuffer[0]);
     /* if (dataId == session::data_type::installGyroOffset && gyroOffsetInstalled)
      {
        gyroOffsetInstalled = false;
        imuReader->writeGyroOffset(0.0F, 0.0F, 0.0F);
        UpdateLcd();
      }*/
    }
    // idle
    int32_t sleep = TASK_SLEEP_READ_SESSION - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void ButtonLoop(void *arg)
{
  uint8_t btnFlag = 0;

  while (1)
  {
    uint32_t entryTime = millis();
    M5.update();
    if (button.containsUpdate(M5, btnFlag))
    {
      for (int i = 0; i < INPUT_BTN_NUM; i++)
      {
        if (xSemaphoreTake(btnDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE)
        {
          btnData.timestamp = millis();
          btnData.btnBits = btnFlag;
          hasButtonUpdate = true;

          static int lastTime = 0;
          static int lastBtn = 0;
          M5.Lcd.setCursor(0, 80);

          if (btnFlag == 1 && i == 0)
          { // M5button pressed
            lastTime = btnData.timestamp;
            lastBtn = btnFlag;
          }
          else if (btnFlag == 2 && i == 0)
          { // side button pressed
            lastTime = btnData.timestamp;
            lastBtn = btnFlag;
          }
          else if (btnFlag == 0 && i == 0)
          { // release
            if (btnData.timestamp - lastTime < 1000)
            {
              if (lastBtn == 1) // M5button released
              {
                if (mode == 1)
                  pitchLimit += gainValue;
                else if (mode == 2)
                  rollLimit += gainValue;
                else if (mode == 3)
                  yawLimit += gainValue;
                else if (mode == 0)
                {
                  okPitchnum = 0;
                  okRollnum = 0;
                  okYawnum = 0;
                }
                if (pitchLimit > 120)
                  pitchLimit = 20;
                if (rollLimit > 120)
                  rollLimit = 20;
                if (yawLimit > 120)
                  yawLimit = 20;
              }
              else
              { // side button released
                mode++;
                if (mode == 4)
                  mode = 0;
              }
            }
            else if (btnData.timestamp - lastTime < 5000)
            {
              if (lastBtn == 1) // M5 button longpressed and released
              {
                if (gainValue == 1)
                  gainValue = 10;
                else
                  gainValue = 1;
              }
              else // side button longpressed and released
              {
              }
            }
            else // 5000以上長押し後のreleased
            {
              if (lastBtn == 1 && mode == 0)
              {
                yawLimit = 30;
                rollLimit = 30;
                pitchLimit = 30;
                mode = 0;
                okPitchnum = 0;
                okRollnum = 0;
                okYawnum = 0;
              }
            }
          }
        }
        xSemaphoreGive(btnDataMutex);
      }
    }
    // idle
    int32_t sleep = TASK_SLEEP_BUTTON - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}
