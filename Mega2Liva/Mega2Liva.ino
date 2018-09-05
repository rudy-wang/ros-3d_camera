#include "ros_arduino_core_config.h"

#define LEFT 0
#define RIGHT 1
#define LIFT_UP_PIN 3
#define LIFT_DOWN_PIN 4
#define LED_PIN_LEFT 8
#define LED_PIN_RIGHT 9

#define BREAK_LEFT 6
#define BREAK_RIGHT 7


/*******************************************************************************
  ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
  Publisher
*******************************************************************************/
std_msgs::Int32MultiArray sensor_value;                           // [0,1] for encoder, [2] for IMU theta
ros::Publisher sensor_value_pub("sensor_value", &sensor_value);
std_msgs::UInt8 lift_status;                                      // 0 = unexpected result or error, 1 = lift up, 2 = lift down, 3 = halt, 4 = reserved for UAGV onload status
ros::Publisher lift_status_pub("liftStatus", &lift_status);

/*******************************************************************************
  Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<std_msgs::UInt8> lift_cmd_sub("liftCmd", liftCmdCallback);

/*******************************************************************************
  SoftwareTimer for UAGV
*******************************************************************************/
static uint32_t cmd_t, pub_t;

/*******************************************************************************
  Declaration for motor & BT command
*******************************************************************************/
String btcmds;
String motorCmd;

float mDesiredDirection, mDesiredVelocity; //從BT來的command：角度-1~1, 速度 -1~1
float mMaxVelocity = 0.5; //最高速 = 0.5m/s
float mCurrentVelocity, mCurrentDirection; //目前的速度和角度
float safeVelocity = 1;

float goal_linear_velocity  = 0.0;
float goal_angular_velocity = 0.0;

short VELOCITY_CONSTANT_VAULE = 30000;    //cmd:16585~=1(m/s)
short LIMIT_X_MAX_VELOCITY = 18000;      //max cmd:8293~=0.5m/s
short LIMIT_X_MIN_VELOCITY = 1640;       //min value

int lin_vel1;  //左輪
int lin_vel2;  //右輪

/*******************************************************************************
  Declaration for LED
*******************************************************************************/
int brightness = 0;
int fadeAmount = 5;

/*******************************************************************************
  Declaration for control flags & other parameters
*******************************************************************************/
byte resetCtrl = 0; // 0 for disconnected, 1 for connected
bool i2cinitial = true;

void setup() {
  // Initializing parameters
  wdt_disable();
  wdt_enable(WDTO_2S);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(57600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  Serial3.setTimeout(3);
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  pinMode(LIFT_UP_PIN, OUTPUT);
  pinMode(LIFT_DOWN_PIN, OUTPUT);
  digitalWrite(LIFT_UP_PIN, LOW);
  digitalWrite(LIFT_DOWN_PIN, LOW);

  lin_vel1 = 0;
  lin_vel2 = 0;
  cmd_t = 0;
  pub_t = 0;
  sensor_value.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension));
  sensor_value.layout.dim[0].label = "sensors";
  sensor_value.layout.dim[0].size = 3;
  sensor_value.layout.dim[0].stride = 3;
  sensor_value.layout.data_offset = 0;
  sensor_value.data_length = 3;
  sensor_value.data = (long int *)malloc (sizeof(long int) * 3);
  sensor_value.data[0] = 0;
  sensor_value.data[1] = 0;
  sensor_value.data[2] = 0;

  // Initializing IMU sensor
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("ready");
    delay(5);
  } else {
    //while (1);
  }
  // Initializing ROS connection
  nh.initNode();
  nh.advertise(sensor_value_pub);
  nh.subscribe(cmd_vel_sub);
  nh.advertise(lift_status_pub);
  nh.subscribe(lift_cmd_sub);
  delay(100);
}

void loop() {
  if (nh.connected())
  {
    if (resetCtrl == 0)
    {
      digitalWrite(53, LOW);
      delay(1);
      digitalWrite(53, HIGH);
      Serial3.end();
      delay(1000);
      Serial3.begin(9600);
      delay(1);
      i2cinitial = true;
      resetCtrl = 1;
    }
    uint32_t t = millis();

    BTCommand();
    updateEncoderValue();

    if ((t - cmd_t) >= (1000 / CMD_VEL_PUBLISH_PERIOD))
    {
      sendCmd();
      cmd_t = t;
    }
    if ((t - pub_t) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
    {
      sensor_value_pub.publish(&sensor_value);
      pub_t = t;
    }
  }
  if (!nh.connected())
  {
    lin_vel1 = 2500;
    lin_vel2 = 2500;
    sendCmd();
    digitalWrite(53, LOW);
    delay(1);
    digitalWrite(53, HIGH);
    Serial3.end();
    delay(1000);
    Serial3.begin(9600);
    delay(1);
    resetCtrl = 0;
  }
  nh.spinOnce();
  wdt_reset();
  delay(1);
}

/*******************************************************************************
  Send command to mega2560 responsible for motors
*******************************************************************************/
void sendCmd(void)
{
  motorCmd = "";
  motorCmd += 'L';
  motorCmd += lin_vel1;
  motorCmd += 'R';
  motorCmd += lin_vel2;
  motorCmd += ';';
  Serial3.print(motorCmd);
  delay(2);
}

/*******************************************************************************
  Get quaternion from IMU
*******************************************************************************/
void getQuaternionFromIMU() {
  // reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

/*******************************************************************************
   BT receive command from mobile
*******************************************************************************/
void BTCommand() {
  if (Serial2.available()) {
    while (Serial2.available()) {
      btcmds += (char) Serial2.read();
      delay(3);
    }
    Serial2.println(btcmds);
    commandCheck(btcmds);
    btcmds = "";
  }
}

/*******************************************************************************
   BT command check
*******************************************************************************/
void commandCheck(String cmd) {
  int len = cmd.length();
  mDesiredDirection = 0;
  mDesiredVelocity = 0;

  delay(10);
  mDesiredDirection = 0;
  mDesiredVelocity = 0;
  if (cmd[0] == '@' && cmd[1] == 'V' && cmd[7] == ';') {
    digitalWrite(BREAK_LEFT, LOW);
    digitalWrite(BREAK_RIGHT, LOW);

    mDesiredVelocity = (((cmd[2] - 48) * 10 + (cmd[3] - 48)) - 20) * 0.1;
    mDesiredDirection = (((cmd[5] - 48) * 10 + (cmd[6] - 48)) - 20) * 0.1;

    transitVelDir(mDesiredVelocity, mDesiredDirection);
  }/* else if (cmd[0] == '@' && cmd[1] == 'L' && cmd[3] == ';') {
    if (cmd[2] == 'S') {
      liftup(0);
    } else if (cmd[2] == 'U') {
      liftup(1);
    } else if (cmd[2] == 'D') {
      liftup(2);
    }
  } */else if (cmd[0] == '@' && cmd[1] == 'S' && cmd[4] == ';') {
    if (cmd[2] == 'L') {
      if (cmd[3] == 'O') {
        digitalWrite(BREAK_LEFT, HIGH);
      } else if (cmd[3] == 'F') {
        digitalWrite(BREAK_LEFT, LOW);
      }

    } else if (cmd[2] == 'R') {
      if (cmd[3] == 'O') {
        digitalWrite(BREAK_RIGHT, HIGH);
      } else if (cmd[3] == 'F') {
        digitalWrite(BREAK_RIGHT, LOW);
      }
    }
  }
}

/*******************************************************************************
  Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_linear_velocity  = cmd_vel_msg.linear.x;
  goal_angular_velocity = cmd_vel_msg.angular.z;
  controlMotorSpeed(goal_linear_velocity, goal_angular_velocity);
}

/*******************************************************************************
   transit BT command to velocity & direction
*******************************************************************************/
void transitVelDir(float vel, float dir) {
  float velocity = vel;
  if (dir == 0) { //直走
    if (velocity > safeVelocity) {
      velocity = safeVelocity;
    } else if (velocity < -safeVelocity) {
      velocity = -safeVelocity;
    }
    goal_linear_velocity = velocity;
    goal_angular_velocity = 0;

  } else if (dir == -1 || dir == 1) { //90度左右轉
    goal_linear_velocity = 0;
    goal_angular_velocity = -1 * dir * velocity; //-3
    //    goal_angular_velocity = -0.1 * dir * 50;
    //    goal_angular_velocity = -0.1 * dir * 10;
  } else { //任意角度-1~1
    float x = sin(dir * PI);
    float y = (cos(dir * PI) + 1);
    float r = ((x * x) + (y * y)) / (2 * x);
    float abs_r = (r > 0) ? r : -r;
    velocity /= (1 + (1.0 / abs_r));
    if (velocity > safeVelocity) {
      //print warning
      velocity = safeVelocity;
    } else if (velocity < -safeVelocity) {
      //print warning
      delay(2);
      velocity = -safeVelocity;
    }
    goal_linear_velocity = velocity;
    goal_angular_velocity = -1 / r * goal_linear_velocity;
  }
  controlMotorSpeed(goal_linear_velocity, goal_angular_velocity);
}

/*******************************************************************************
   Control Motor Speed
*******************************************************************************/
void controlMotorSpeed(float goalLinearVel, float goalAngularVel) {
  float wheel_speed_cmd[2];

  wheel_speed_cmd[LEFT]  = goalLinearVel - (goalAngularVel * WHEEL_SEPARATION / 2);
  wheel_speed_cmd[RIGHT] = goalLinearVel + (goalAngularVel * WHEEL_SEPARATION / 2);

  lin_vel1 = wheel_speed_cmd[LEFT] * VELOCITY_CONSTANT_VAULE;
  delay(2);

  if (lin_vel1 > LIMIT_X_MAX_VELOCITY)       lin_vel1 =  LIMIT_X_MAX_VELOCITY;
  else if (lin_vel1 < LIMIT_X_MIN_VELOCITY && lin_vel1 > 0 ) lin_vel1 = LIMIT_X_MIN_VELOCITY;

  else if (lin_vel1 < -LIMIT_X_MAX_VELOCITY) lin_vel1 = -LIMIT_X_MAX_VELOCITY;
  else if (lin_vel1 > -LIMIT_X_MIN_VELOCITY && lin_vel1 < 0) lin_vel1 = -LIMIT_X_MIN_VELOCITY;

  lin_vel2 = wheel_speed_cmd[RIGHT] * VELOCITY_CONSTANT_VAULE;
  delay(2);

  if (lin_vel2 > LIMIT_X_MAX_VELOCITY)       lin_vel2 =  LIMIT_X_MAX_VELOCITY;
  else if (lin_vel2 < LIMIT_X_MIN_VELOCITY && lin_vel2 > 0 ) lin_vel2 = LIMIT_X_MIN_VELOCITY;

  else if (lin_vel2 < -LIMIT_X_MAX_VELOCITY) lin_vel2 = -LIMIT_X_MAX_VELOCITY;
  else if (lin_vel2 > -LIMIT_X_MIN_VELOCITY && lin_vel2 < 0) lin_vel2 = -LIMIT_X_MIN_VELOCITY;
}

/*******************************************************************************
   Lift
*******************************************************************************/
void liftCmdCallback(const std_msgs::UInt8& lift_cmd_msg) {
  delay(10);
  digitalWrite(LIFT_UP_PIN, LOW);
  digitalWrite(LIFT_DOWN_PIN, LOW);
  delay(10);
  switch (lift_cmd_msg.data) {
    case 1:         //up
      digitalWrite(LIFT_UP_PIN, HIGH);
      digitalWrite(LIFT_DOWN_PIN, LOW);
      lift_status.data = (unsigned int)1;
      break;
    case 2:         //down
      digitalWrite(LIFT_UP_PIN, LOW);
      digitalWrite(LIFT_DOWN_PIN, HIGH);
      lift_status.data = (unsigned int)2;
      break;
    case 3:         //stop
      digitalWrite(LIFT_UP_PIN, LOW);
      digitalWrite(LIFT_DOWN_PIN, LOW);
      lift_status.data = (unsigned int)3;
      break;
    default:        //stop
      digitalWrite(LIFT_UP_PIN, LOW);
      digitalWrite(LIFT_DOWN_PIN, LOW);
      lift_status.data = (unsigned int)0;
      break;
  }
  lift_status_pub.publish(&lift_status);
}

/*******************************************************************************
  Publish encoder value (encoder i2c value, timestamp)
*******************************************************************************/
void updateEncoderValue(void)
{
  float* orientation;
  String cstring = "";
  int theta;
  int lstart = 0, lend = 0, rstart = 0;
  byte incomingByte[10] =
  {
    0x00, 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00
  };
  if (Serial3.available())
  {
    while (Serial3.available())
    {
      cstring = Serial3.readString();
      delay(2);
    }
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
    if (lstart <= 0 || lend <= 0 || rstart <= 0)
    {
      return;
    }
    if (i2cinitial == false)
    {
      if (abs(cstring.substring(lstart, lend).toInt() - sensor_value.data[0]) > 5000 || abs(cstring.substring(rstart, cstring.length() - 1).toInt() - sensor_value.data[1]) > 5000)
      {
        return;
      }
    }
    sensor_value.data[0] = cstring.substring(lstart, lend).toInt();
    sensor_value.data[1] = cstring.substring(rstart, cstring.length() - 1).toInt();
    i2cinitial = false;
  }
  //  getQuaternionFromIMU();
  //  theta = round(-ypr[0] * 100000);
  sensor_value.data[2] = 0;//theta;
}
