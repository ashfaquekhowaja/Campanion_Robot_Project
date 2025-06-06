#include <Wire.h>
#include <Seeed_Arduino_SSCMA.h>

SSCMA AI;

// Define the slave I2C address
#define SLAVE_ADDR 8
#define TIMEOUT_INTERVAL 3000  // 3秒超时时间（毫秒）
#define DETECTION_INTERVAL 50  // 检测间隔时间（毫秒）

unsigned long lastDetectionTime = 0;  // 上次检测到目标的时间戳
unsigned long lastSendTime = 0;       // 上次发送命令的时间戳
bool targetDetected = false;  // 用于跟踪是否检测到目标

// Position determination function
String getPosition(int x) {
  if (x < 150) {
    return "left";
  } else if (x > 345) {
    return "right";
  } else {
    return "center";
  }
}

// 计算距离的函数
String calculateDistance(int w, int h) {
  int area = w * h;
  if (area < 230000) {
    return "far";
  } else if (area > 230000) {
    return "near";
  }
}

void setup() {
  Wire.begin();  // Start I2C communication as master
  Serial.begin(9600);  // Start Serial for debugging
  delay(1000);

  AI.begin();  // Initialize Grove Vision AI
  Serial.println("=== Position Detection System Started ===");
}

void loop() {
  if (!AI.invoke()) {
    unsigned long currentTime = millis();
    
    // 检查是否超时
    if (currentTime - lastDetectionTime > TIMEOUT_INTERVAL && targetDetected) {
      Serial.println("No target detected (timeout).");
      sendCommandToSlave(0);  // Send center command if no target detected
      targetDetected = false;  // 重置检测状态
    }
    
    // Check if any boxes are detected
    if (AI.boxes().size() > 0) {
      lastDetectionTime = currentTime;  // 更新最后检测时间
      targetDetected = true;  // 设置检测状态为true
      
      for (int i = 0; i < AI.boxes().size(); i++) {
        String position = getPosition(AI.boxes()[i].x);  // Get position (left, right, or center)
        String distance = calculateDistance(AI.boxes()[i].w, AI.boxes()[i].h);  // 计算距离
        
        // 只在需要时打印调试信息
        if (currentTime - lastSendTime > 500) {  // 每500ms打印一次调试信息
          Serial.println("-------------------");
          Serial.print("Target #");
          Serial.println(i + 1);
          Serial.print("Position: ");
          Serial.println(position);
          Serial.print("X Coordinate: ");
          Serial.println(AI.boxes()[i].x);
          Serial.print("Width: ");
          Serial.println(AI.boxes()[i].w);
          Serial.print("Height: ");
          Serial.println(AI.boxes()[i].h);
          Serial.print("Distance: ");
          Serial.println(distance);
          Serial.print("Area: ");
          Serial.println(AI.boxes()[i].w * AI.boxes()[i].h);
          Serial.print("Confidence: ");
          Serial.println(AI.boxes()[i].score);
          Serial.println("-------------------");
          lastSendTime = currentTime;
        }

        // 根据位置发送命令
        if (position == "left") {
          sendCommandToSlave(1);  // Send 1 for left
          Serial.println("Sending left command to slave.");
        } else if (position == "right") {
          sendCommandToSlave(2);  // Send 2 for right
          Serial.println("Sending right command to slave.");
        } else if (position == "center") {
          // 当在中心位置时，根据距离发送不同的命令
          if (distance == "near") {
            sendCommandToSlave(3);  // 近
            Serial.println("Sending near command (3) to slave.");
          } else if (distance == "far") {
            sendCommandToSlave(4);  // 远
            Serial.println("Sending far command (4) to slave.");
          }
        }
      }
    } else {
      // 如果没有检测到目标，检查是否超时
      if (currentTime - lastDetectionTime > TIMEOUT_INTERVAL && targetDetected) {
        Serial.println("No target detected.");
        sendCommandToSlave(0);  // Send center command if no target detected
        targetDetected = false;  // 重置检测状态
      }
    }

    delay(DETECTION_INTERVAL);  // 使用较短的检测间隔
  }
}

// Function to send direction to slave (1 = left, 2 = right, 0 = center)
void sendCommandToSlave(int command) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(command);  // Send the numerical command (1, 2, or 0)
  Wire.endTransmission();
}