#include <Arduino.h>
#include <Wire.h>
#include "pin_config.h"
#include "config_thamso.h"
#include "HMC5883L_Simple.h"

unsigned long previousTime = 0; // Thời gian của vòng lặp trước

// put function declarations here:
void adc_read(int state);
void selectFunction(int function);
void led_status(float time);
void countLeftPulse();
void countRightPulse();
void read_sensor();
void steering_control();
void stopMotors();
void turn90_counterclockwise();
void turn90_clockwise();
void turn180_counterclockwise();
void turn180_clockwise();
void motor_controller(int dir, int target_left_pwm, int target_right_pwm);
void flood_fill();
int checkCell();
int findClosestIndex(int target, int array[], int size);
int position_controller();

bool isButtonPressed();

// Hàm ngắt để đếm xung động cơ trái
void countLeftPulse() {
  leftPulseCount++;
}

// Hàm ngắt để đếm xung động cơ phải
void countRightPulse() {
  rightPulseCount++;
}

HMC5883L_Simple compass;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  compass.SetDeclination(23, 35, 'E');
  compass.SetSamplingMode(COMPASS_SINGLE);
  compass.SetScale(COMPASS_SCALE_810);
  compass.SetOrientation(COMPASS_HORIZONTAL_Y_NORTH);
  // Đặt FUNCTION_SELECT là INPUT vì bạn đang đọc giá trị từ chân này
  pinMode(FUNCTION_SELECT, INPUT);

  pinMode(SENSOR_0, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(EMITTER_FRONT, OUTPUT);
  pinMode(EMITTER_SIDE, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);

  // Cài đặt các chân điều khiển động cơ là output
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  // Cài đặt các chân đọc encoder là input
  pinMode(ENCODER_LEFT_B, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);
  // Kích hoạt ngắt ngoài khi có sự thay đổi trên các chân encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_CLK), countLeftPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_CLK), countRightPulse, CHANGE);
  //đọc các giá trị từ EEPROM
}

void loop() {
  if (isButtonPressed() == true) {
    adc_read(1023);
  }
  else if (Serial.available() > 0) {
    int state = Serial.parseInt();
    adc_read(state);
    state = 0;
  }
}

void adc_read(int state) {
  // Đọc giá trị từ FUNCTION_SELECT

  // Nếu giá trị là 1023 (tối đa), chờ đến khi giá trị giảm xuống
  if (state == 1023) {
    unsigned long startTime = millis(); // Lưu thời gian bắt đầu

    // Chờ cho đến khi giá trị FUNCTION_SELECT < 1023 hoặc vượt quá thời gian giới hạn
    while (analogRead(FUNCTION_SELECT) >= 1023) {
      delay(50); // Tránh làm CPU quá tải với vòng lặp liên tục
      if (millis() - startTime > 5000) { // Thoát sau 5 giây
        return; // Thoát hàm nếu không đạt điều kiện
      }
    }

    // Đọc giá trị và xử lý
    int adc_in = analogRead(FUNCTION_SELECT);
    int index = findClosestIndex(adc_in, adc_thesholds, sizeof(adc_thesholds) / sizeof(int));
    Serial.println(index);
    selectFunction(index);
  }
  else {
    Serial.println(state);
    selectFunction(state);
  }

}

int findClosestIndex(int target, int array[], int size) {
  int closestIndex = 0;
  int minDifference = abs(target - array[0]);

  for (int i = 1; i < size; i++) {
    int difference = abs(target - array[i]);
    if (difference < minDifference) {
      minDifference = difference;
      closestIndex = i;
    }
  }

  return closestIndex;
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

void turn90_counterclockwise(){
  // Bánh trái lùi
  digitalWrite(MOTOR_LEFT_DIR, left_backward);
  digitalWrite(MOTOR_RIGHT_DIR, right_forward);
  analogWrite(MOTOR_LEFT_PWM, 150);
  analogWrite(MOTOR_RIGHT_PWM, 150);
  // Quay trong thời gian cố định
  delay(TURN_TIME_90_DEGREES/2.1236543212345689);
  stopMotors();
  currentDirection = (currentDirection << 1) | (currentDirection >> 3);
  currentDirection &= 0b1111;
}

void turn90_clockwise(){
  digitalWrite(MOTOR_LEFT_DIR, left_forward);
  digitalWrite(MOTOR_RIGHT_DIR, right_backward);
  analogWrite(MOTOR_LEFT_PWM, 150);
  analogWrite(MOTOR_RIGHT_PWM, 150);
  // Quay trong thời gian cố định
  delay(TURN_TIME_90_DEGREES/2.1236543212345689);
  stopMotors();
  currentDirection = (currentDirection >> 1) | (currentDirection << 3);
  currentDirection &= 0b1111; // Giới hạn chỉ 4 bit
}

void turn180_clockwise(){
  digitalWrite(MOTOR_LEFT_DIR, left_forward);
  digitalWrite(MOTOR_RIGHT_DIR, right_backward);
  analogWrite(MOTOR_LEFT_PWM, 75);
  analogWrite(MOTOR_RIGHT_PWM, 75);
  // Quay trong thời gian cố định
  delay(TURN_TIME_90_DEGREES*2.1236543212345689);
  stopMotors();
  currentDirection = (currentDirection >> 2) | (currentDirection << 2);
  currentDirection &= 0b1111; // Giới hạn chỉ 4 bit
}

void turn180_counterclockwise(){
  digitalWrite(MOTOR_LEFT_DIR, left_backward);
  digitalWrite(MOTOR_RIGHT_DIR, right_forward);
  analogWrite(MOTOR_LEFT_PWM, 75);
  analogWrite(MOTOR_RIGHT_PWM, 75);
  // Quay trong thời gian cố định
  delay(TURN_TIME_90_DEGREES*2.1236543212345689);
  stopMotors();
  currentDirection = (currentDirection << 2) | (currentDirection >> 2);
  currentDirection &= 0b1111; // Giới hạn chỉ 4 bit
}

void selectFunction(int function) {
  if (function == 15) {
    while (isButtonPressed() == false)
    {
      steering_control();
      delay(10);
    }
  }
  if (function == 13) {
    while (isButtonPressed() == false)
    {
      turn90_counterclockwise();
      delay(2000);
    }
  }
  if (function == 11) {
    while (isButtonPressed() == false){
      read_sensor();
      delay(10);
    }
  }
}

void led_status(float time) {

  // Tính khoảng thời gian nhấp nháy (ms)
  unsigned long interval = (unsigned long)(time * 100);

  // Lấy thời gian hiện tại
  unsigned long currentMillis = millis();

  // Kiểm tra nếu đã đủ thời gian
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Cập nhật thời gian trước
    ledState = !ledState;           // Đảo trạng thái LED
    digitalWrite(LED_PIN, ledState); // Cập nhật trạng thái LED
  }
}

bool isButtonPressed() {
  return analogRead(FUNCTION_SELECT) == 1023;
}

void steering_control() {
  read_sensor();
  int steering_error;
  if(side_left > side_right){
    steering_error = side_left - side_left_threshold;
  } else {
    steering_error = -1*(side_right - side_right_threshold);
  }

  
  int steering_pd = steering_Kp * steering_error + steering_Kd * (steering_error - last_steeing_error);
  last_steeing_error = steering_error;
  steering_pd = constrain(steering_pd, -88, 88);
  int base_speed = position_controller();
  //Serial.print("front left: " + String(front_left) + " | front right: " + String(front_right));
  // Tính khoảng cách của cảm biến đến ngưỡng

  current_left_pwm = constrain(base_speed - steering_pd, 0, 255); //base_speed - steering_pd;
  current_right_pwm = constrain(base_speed + steering_pd, 0, 255); //base_speed + steering_pd;
  motor_controller(0, current_left_pwm, current_right_pwm);
  Serial.println(" | Steering error: " + String(steering_error) + " | steering_pd: " + String(steering_pd) + " | base_speed: " + String(base_speed) + " | current_left_pwm: " + String(current_left_pwm) + " | current_right_pwm: " + String(current_right_pwm));
}

void motor_stop(){
  current_left_pwm = 0;
  current_right_pwm = 0;
  analogWrite(MOTOR_LEFT_PWM, current_left_pwm);
  analogWrite(MOTOR_RIGHT_PWM, current_right_pwm);
}

void motor_controller(int dir, int target_left_pwm, int target_right_pwm) {
  static unsigned long last_update = 0;  // Thời gian cập nhật cuối cùng
  int avr_pulse = (leftPulseCount + rightPulseCount) / 2;
  // Kiểm tra nếu đã đến lúc cập nhật PWM
  if (millis() - last_update >= delay_between_steps) {
    last_update = millis(); // Cập nhật thời gian
    // Điều chỉnh dần PWM
    if ( current_left_pwm < target_left_pwm) {
      current_left_pwm += acceleration_step;
      if ( current_left_pwm > target_left_pwm) {
        current_left_pwm = target_left_pwm; // Đảm bảo không vượt quá mục tiêu
      }
    } else if ( current_left_pwm > target_left_pwm) {
      current_left_pwm -= acceleration_step;
      if ( current_left_pwm < target_left_pwm) {
        current_left_pwm = target_left_pwm; // Đảm bảo không vượt quá mục tiêu
      }
    }
    if ( current_right_pwm < target_right_pwm) {
      current_right_pwm += acceleration_step;
      if ( current_right_pwm > target_right_pwm) {
        current_right_pwm = target_right_pwm; // Đảm bảo không vượt quá mục tiêu
      }
    } else if ( current_right_pwm > target_right_pwm) {
      current_right_pwm -= acceleration_step;
      if ( current_right_pwm < target_right_pwm) {
        current_right_pwm = target_right_pwm; // Đảm bảo không vượt quá mục tiêu
      }
    }
  }
  // Điều khiển động cơ
  if (dir == 1) {
    // Lùi
    digitalWrite(MOTOR_LEFT_DIR, left_backward);
    digitalWrite(MOTOR_RIGHT_DIR, right_backward);
  } else if (dir == 2) {
    // Xoay trái
    digitalWrite(MOTOR_LEFT_DIR, left_backward);
    digitalWrite(MOTOR_RIGHT_DIR, right_forward);
  }
    else if (dir == 3) {
    // Xoay phải
    digitalWrite(MOTOR_LEFT_DIR, left_forward);
    digitalWrite(MOTOR_RIGHT_DIR, right_backward);
  }
   else {
    // Tiến
    digitalWrite(MOTOR_LEFT_DIR, left_forward);
    digitalWrite(MOTOR_RIGHT_DIR, right_forward);
  }

  // Gửi tín hiệu PWM
  analogWrite(MOTOR_LEFT_PWM, current_left_pwm);
  analogWrite(MOTOR_RIGHT_PWM, current_right_pwm);

  // In thông tin PWM
  //Serial.print("Current_left_PWM: "); Serial.print(current_left_pwm); Serial.print(" Current_right_PWM: "); Serial.println(current_right_pwm);
  cell_count =(int)((avr_pulse / FULL_CELL) + 0.5);
  //Serial.println("Số cell đi được: " + String(cell_count));
}

void read_sensor() {
  //activate emitter
  led_status(0.5);
  digitalWrite(EMITTER_FRONT, HIGH);
  front_left =  map(analogRead(SENSOR_0), 14, 510, 0, 1023);
  front_right = map(analogRead(SENSOR_3), 2, 110, 0, 1023);
  digitalWrite(EMITTER_FRONT, LOW);
  digitalWrite(EMITTER_SIDE, HIGH);
  //read sensor data
  side_left = analogRead(SENSOR_1);
  side_right = analogRead(SENSOR_2);
  digitalWrite(EMITTER_SIDE, LOW);
  //deactivate emitter
  // digitalWrite(EMITTER_FRONT, LOW);
  // digitalWrite(EMITTER_SIDE, LOW);
}

int position_controller() {
  int avr_front_threshold = (front_left_threshold + front_right_threshold) / 2;
  int error;
  if ((front_left > avr_front_threshold && side_left > side_left_threshold) || (front_right > avr_front_threshold && side_right > side_right_threshold)) {
    // deflection
    error = avr_front_threshold - min(front_left, front_right);
  } else if(side_left < side_left_threshold && side_right < side_right_threshold) {
    // lost both walls
    delay(150);
    return 0;
  }
   else {
    error = avr_front_threshold - ((front_left + front_right) / 2);
  }
  float position_pd = error * position_Kp + (error - last_position_error) * position_Kd;
  last_position_error = error;
  position_pd = constrain(position_pd, 0, 150);
  Serial.print("front left: " + String(front_left) + " | front right: " + String(front_right) + " |error: " + String(error));
  return position_pd;
}

int checkCell(){
  if(front_left > front_left_threshold && front_right < front_right_threshold){
    //onlysee left wall, turn right 1
    // turn90_clockwise();
    return 1;
  }else if(front_left < front_left_threshold && front_right > front_right_threshold){
    //only see right wall, turn left 2
    // turn90_counterclockwise();
    return 2;
  } else if(front_left > front_left_threshold && front_right > front_right_threshold && side_left > side_left_threshold && side_right > side_right_threshold && side_left > side_right){
    //see both walls, close to left wall, turn 180 right 3
    // turn180_clockwise();
    return 3;
  } else if(front_left > front_left_threshold && front_right > front_right_threshold && side_left > side_left_threshold && side_right > side_right_threshold && side_left < side_right){
    //see both walls, close to right wall, turn 180 left 4
    // turn180_counterclockwise();
    return 4;
  } else{
    //not see both walls 5
    return 5;
  }

}

void priority_direction(){ //tự chế
  
}

void flood_fill(){

}