#include <Arduino.h>
#include <Wire.h> // 新增

const static uint8_t pwm_min = 2;
const static uint8_t motorpwmPin[4] = { 10, 9, 6, 11};
const static uint8_t motordirectionPin[4] = { 12, 8, 7, 13};

// 定义速度参数
#define STRAIGHT_SPEED 80    // 直行速度
#define TURN_SPEED 70        // 转向速度

void Motor_Init(void);
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot,bool drift);
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);

volatile int receivedCommand = -1; // 用于存储接收到的命令
bool isMoving = false; // 用于跟踪移动状态

void receiveData(int numBytes) {
  if (Wire.available()) {
    receivedCommand = Wire.read(); // 读取命令
  }
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(500);
  Motor_Init();

  // I2C从机初始化，地址8
  Wire.begin(8);
  Wire.onReceive(receiveData);

  // 可选：上电时静止
  Velocity_Controller(0,0,0,0);
}

void loop() {
  // 根据接收到的命令控制
  if (receivedCommand == 1) {
    Serial.println("I2C: left, 原地左转");
    Velocity_Controller(0, 0, TURN_SPEED, 0); // 原地左转
    isMoving = false; // 停止后重置移动状态
  } else if (receivedCommand == 2) {
    Serial.println("I2C: right, 原地右转");
    Velocity_Controller(0, 0, -TURN_SPEED, 0); // 原地右转
    isMoving = false; // 停止后重置移动状态
  } else if (receivedCommand == 3) {
    Serial.println("I2C: stop, 停止");
    Velocity_Controller(0, 0, 0, 0); // 停止
    isMoving = false; // 更新移动状态
  } else if (receivedCommand == 4) {
    if (!isMoving) {
      Serial.println("I2C: start moving, 开始移动");
      isMoving = true; // 更新移动状态
    }
    if (isMoving) {
      Velocity_Controller(0, STRAIGHT_SPEED, 0, 0); // 直行
    }
  } else {
    // 未收到命令时保持当前状态
    if (!isMoving) {
      Velocity_Controller(0, 0, 0, 0);
    }
  }
  delay(100); // 防止过快
}

 /* 电机初始化函数 */
void Motor_Init(void){
  for(uint8_t i = 0; i < 4; i++){
    pinMode(motordirectionPin[i], OUTPUT);
  }
  Velocity_Controller( 0, 0, 0, 0);
}

/**
 * @brief 速度控制函数
 * @param angle   用于控制小车的运动方向，小车以车头为0度方向，逆时针为正方向。
 *                取值为0~359
 * @param velocity   用于控制小车速度，取值为0~100。
 * @param rot     用于控制小车的自转速度，取值为-100~100，若大于0小车有一个逆
 *                 时针的自转速度，若小于0则有一个顺时针的自转速度。
 * @param drift   用于决定小车是否开启漂移功能，取值为0或1，若为0则开启，反之关闭。
 * @retval None
 */
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot,bool drift) {
  int8_t velocity_0, velocity_1, velocity_2, velocity_3;
  float speed = 1;
  angle += 90;
  float rad = angle * PI / 180;
  if (rot == 0) speed = 1;///< 速度因子
  else speed = 0.5; 
  velocity /= sqrt(2);
  if (drift) {
    velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed;
    velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed;
    velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed * 2;
    velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed * 2;
  } else {
    velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed + rot * speed;
    velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed - rot * speed;
    velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed;
    velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed;
  }
  Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
}
 
/**
 * @brief PWM与轮子转向设置函数
 * @param Motor_x   作为PWM与电机转向的控制数值。根据麦克纳姆轮的运动学分析求得。
 * @retval None
 */
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3) {
  int8_t pwm_set[4];
  int8_t motors[4] = { Motor_0, Motor_1, Motor_2, Motor_3};
  bool direction[4] = { 1, 0, 0, 1};///< 前进 左1 右0
  for(uint8_t i; i < 4; ++i) {
    if(motors[i] < 0) direction[i] = !direction[i];
    else direction[i] = direction[i];

    if(motors[i] == 0) pwm_set[i] = 0;
    else pwm_set[i] = map(abs(motors[i]), 0, 100, pwm_min, 255);

    digitalWrite(motordirectionPin[i], direction[i]); 
    analogWrite(motorpwmPin[i], pwm_set[i]); 
  }
}


