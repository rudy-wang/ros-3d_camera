#include "ros_arduino_core_config.h"

int lin_vel1;  //左輪
int lin_vel2;  //右輪
uint32_t motor_t, pub_t;
String enc = "";
String enc_buffer = "L0R0;";
long Lenc_0, Renc_0;
long Lenc_1, Renc_1;
int Lencfeedback;
int Rencfeedback;
short LIMIT_X_MAX_VELOCITY = 18000;      //max cmd:8293~=0.5m/s
short LIMIT_X_MIN_VELOCITY = 1640;       //min value

float enc2cmdRate = 8; // 15.6 based on 20Hz

byte  incomingByte[10] =
{
  0x00, 0x00 , 0x00 , 0x00 , 0x00
};

void setup() {
  wdt_disable();
  wdt_enable(WDTO_2S);
  Serial2.begin(9600);
  Serial2.setTimeout(3);
  Wire.begin();
  Wire.setTimeout(10);
  motor_t = 0;
  pub_t = 0;
  Lenc_0 = 0;
  Renc_0 = 0;
  Lenc_1 = 0;
  Renc_1 = 0;
  lin_vel1 = 0;
  lin_vel2 = 0;
  Lencfeedback = 0;
  Rencfeedback = 0;
  byteconvert(0, 21);
  delay(2);
  byteconvert(0, 20);
  delay(2);
}

void loop()
{
  uint32_t t = millis();

  if ((t - motor_t) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    i2cenc();
    delay(1);
    if(Serial2.availableForWrite()>enc_buffer.length())
    {
      delay(1);
      Serial2.print(enc_buffer);
      delay(1);
    }
    motorRun();
    motor_t = t;
  }
  wdt_reset();
  delay(1);
}

/*******************************************************************************
  Motor command
*******************************************************************************/
void motorRun(void)
{
  String cstring = "";
  int lstart, lend, rstart;
  int new_vel1, new_vel2;
  if (Serial2.available())
  {
    while (Serial2.available())
    {
      cstring = Serial2.readString();
    }
    Serial2.flush();
    for (int i = 0; i < cstring.length(); i++)
    {
      if ( !((cstring[i] >= '0' && cstring[i] <= '9') || cstring[i] == '-' || cstring[i] == 'L' || cstring[i] == 'R' || cstring[i] == ';'))
      {
        return;
      }
      if (cstring[i] == '-')
      {
        if (cstring[i - 1] != 'L' && cstring[i - 1] != 'R')
        {
          return;
        }
      }
      if (i == cstring.length() - 1)
      {
        if (cstring[i] != ';')
        {
          return;
        }
      }
      if (cstring[i] == 'L')
      {
        lstart = i + 1;
        continue;
      }
      else if (cstring[i] == 'R')
      {
        rstart = i + 1;
        lend = i;
        continue;
      }
    }
    lin_vel1 = cstring.substring(lstart, lend).toInt();
    lin_vel2 = cstring.substring(rstart, cstring.length()).toInt();

    new_vel1 = lin_vel1;// * 1.5 - 0.5 * Lencfeedback;
    new_vel2 = lin_vel2;// * 1.5 - 0.5 * Rencfeedback;

    if (new_vel1 > LIMIT_X_MAX_VELOCITY)       new_vel1 =  LIMIT_X_MAX_VELOCITY;
    else if (new_vel1 < LIMIT_X_MIN_VELOCITY && new_vel1 > 0 && lin_vel1 > LIMIT_X_MIN_VELOCITY) new_vel1 = LIMIT_X_MIN_VELOCITY;

    else if (new_vel1 < -LIMIT_X_MAX_VELOCITY) new_vel1 = -LIMIT_X_MAX_VELOCITY;
    else if (new_vel1 > -LIMIT_X_MIN_VELOCITY && new_vel1 < 0 && lin_vel1 < -LIMIT_X_MIN_VELOCITY) new_vel1 = -LIMIT_X_MIN_VELOCITY;

    if (new_vel2 > LIMIT_X_MAX_VELOCITY)       new_vel2 =  LIMIT_X_MAX_VELOCITY;
    else if (new_vel2 < LIMIT_X_MIN_VELOCITY && new_vel2 > 0 && lin_vel2 > LIMIT_X_MIN_VELOCITY) new_vel2 = LIMIT_X_MIN_VELOCITY;

    else if (new_vel2 < -LIMIT_X_MAX_VELOCITY) new_vel2 = -LIMIT_X_MAX_VELOCITY;
    else if (new_vel2 > -LIMIT_X_MIN_VELOCITY && new_vel2 < 0 && lin_vel2 < -LIMIT_X_MIN_VELOCITY) new_vel2 = -LIMIT_X_MIN_VELOCITY;

    byteconvert( new_vel1, 21);
    delay(2);
    byteconvert( new_vel2, 20);
    cstring = "";
  }
}

/*******************************************************************************
  Read encoder value through i2c
*******************************************************************************/
void i2cenc(void)
{
  enc = "";
  enc += 'L';
  Lenc_1 = 0;
  Lenc_1 = i2cenc2(21);
  delay(1);
  enc += String(Lenc_1,DEC).c_str();
  enc += 'R';
  Renc_1 = 0;
  Renc_1 = i2cenc2(20);
  delay(1);
  enc += String(Renc_1,DEC).c_str();
  enc += ';';
  enc_buffer = "";
  for( int i = 0; i < enc.length(); i++ )
  {
    enc_buffer += enc[i];
  }
  Lencfeedback = enc2cmdRate * (Lenc_0 - Lenc_1); // Lenc_1 decrease while moving forward
  Rencfeedback = enc2cmdRate * (Renc_0 - Renc_1); // Renc_1 decrease while moving forward
  Renc_0 = Renc_1;
  Lenc_0 = Lenc_1;
}

/*******************************************************************************
   i2c 車速指令
*******************************************************************************/
void byteconvert(int motorOutPut, int transmission) {
  Wire.beginTransmission(transmission);
  Wire.write(B00000000);
  Wire.endTransmission();
  if (transmission == 21) {
    Wire.beginTransmission(transmission);
    Wire.write(B00000000);
    Wire.write(highByte(motorOutPut));
    Wire.write(lowByte(motorOutPut));
    Wire.endTransmission();
    delay(1);
  } else if (transmission == 20) {
    Wire.beginTransmission(transmission);
    Wire.write(B00000000);
    Wire.write(highByte(-motorOutPut));
    Wire.write(lowByte(-motorOutPut));
    Wire.endTransmission();
    delay(1);
  }
}

/*******************************************************************************
   encoder checksum
*******************************************************************************/
long i2cenc2(byte SA) {
  //  Wire.beginTransmission(SA);
  byte  i = 0;
  Wire.requestFrom(SA, 6, 1); // request 6 bytes from slave
  while (Wire.available()) {           // slave may send less than requested
    incomingByte [i] = Wire.read();      // receive a byte
    i++;
    delay(1);
  }
  return (check(SA));
}
long check(byte SA) {
  byte i;
  long encv;
  if ((incomingByte[4] == 0xAA) &&  (incomingByte[5] == 0XBB)) {
    for (i = 0; i < 4; i++) {
      encv = encv << 8;
      encv = incomingByte[i] | encv;
    }
    return (encv);
  }
  else {
    Wire.requestFrom(SA, 1, 1);
    while (Wire.available()) {           // slave may send less than requested
      Wire.read();
      delay(1);
      i2cenc2(SA);
    }
  }
}
