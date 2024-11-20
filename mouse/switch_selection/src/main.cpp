#include <Arduino.h>
#include <EEPROM.h>
#include "pin_config.h"
#include "config_thamso.h"
#include "config_diachi.h"

void adc_read(int state);
void selectFunction(int function);
void led_status(float time);
void countLeftPulse();
void countRightPulse();
void motor_move(int pulse_count, int direction, int pwm);
void motor_turn(int pulse_count, int angle);
void positon_control();
void read_sensor();
void motor_controller(int dir, int target_left_pwm, int target_right_pwm);
void steering_controller();
void merge_controller(int dir, float position_pd, float steeing_pd);

int findClosestIndex(int target, int array[], int size);

bool isButtonPressed();

// Hàm ngắt để đếm xung động cơ trái
void countLeftPulse() {
  leftPulseCount++;
}

// Hàm ngắt để đếm xung động cơ phải
void countRightPulse() {
  rightPulseCount++;
}

void setup() {
  Serial.begin(115200);
  
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
  else if(Serial.available() > 0) {
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
  else{
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

void selectFunction(int function) {
  if (function == 15) {
  } else if (function == 13) {
    read_sensor();
    Serial.println(front_left);
    Serial.println(front_right);
    Serial.println(side_left);
    Serial.println(side_right);
  } else if (function == 14) {
    motor_move(FULL_CELL*4, 1, 150);
    delay(500);
    motor_turn(FULL_CELL/1.7, 90);
  } else if (function == 12) {
   while (isButtonPressed() == false)
   {
    read_sensor();
    // positon_control();
    steering_controller();
    delay(100);
   }
  } else if (function == 11) {
   while (isButtonPressed() == false)
   {
    read_sensor();
    positon_control();
    steering_controller();
    // Serial.print("position_pd: "); Serial.print(position_pd); Serial.print(" steering_pd: "); Serial.println(steering_pd);
    //Gọi motor_controller dựa trên hướng position_pd
  if (position_pd > 50) {
    merge_controller(1, abs(position_pd), steering_pd); // Chạy lui
  } else if (0 < position_pd && position_pd < 50){
    merge_controller(1, (abs(position_pd)) / 1.5, steering_pd); // Chạy tiến
  }
   else if ( -170 < position_pd && position_pd <  0) {
    merge_controller(0, abs(position_pd) / 2, steering_pd); // Chạy tiến
  } else{
    merge_controller(0, abs(position_pd) / 2.2, steering_pd);  
  }
   }
   delay(10);
  } else if (function == 10) {
    while (isButtonPressed() == false)
    {
    read_sensor();
    steering_controller();
    // Serial.print(" steering_pd: "); Serial.println(steering_pd);
    merge_controller(0, 200, steering_pd);
    //Gọi motor_controller dựa trên hướng position_pd
    delay(50);
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

//basic motion (for calibration)
void motor_move(int pulse_count, int direction, int pwm){
  int current_pwm = 0;  // Bắt đầu từ 0 PWM

  // Tăng dần PWM từ 0 lên pwm
  
  if (direction == 1){
  while (leftPulseCount < pulse_count && rightPulseCount < pulse_count)
    {
    if (current_pwm < pwm) {
    current_pwm += acceleration_step;
    }
    if (current_pwm > pwm) {
      current_pwm = pwm;  // Đảm bảo không vượt quá pwm
    }
    // Điều khiển động cơ trái tiến
    digitalWrite(MOTOR_LEFT_DIR, left_forward);  // Thiết lập hướng
    analogWrite(MOTOR_LEFT_PWM, current_pwm);    // Tốc độ PWM (giá trị từ 0 đến 255)

    // Điều khiển động cơ phải tiến
    digitalWrite(MOTOR_RIGHT_DIR, right_forward); // Thiết lập hướng
    analogWrite(MOTOR_RIGHT_PWM, current_pwm);   // Tốc độ PWM (giá trị từ 0 đến 255)
    delay(delay_between_steps);
    }
  }else{
    while (leftPulseCount < pulse_count && rightPulseCount < pulse_count){
    if (current_pwm < pwm) {
    current_pwm += acceleration_step;
    }
    if (current_pwm > pwm) {
      current_pwm = pwm;  // Đảm bảo không vượt quá pwm
    }
    // Điều khiển động cơ trái lui
    digitalWrite(MOTOR_LEFT_DIR, left_backward);  // Thiết lập hướng
    analogWrite(MOTOR_LEFT_PWM, current_pwm);    // Tốc độ PWM (giá trị từ 0 đến 255)

    // Điều khiển động cơ phải lui
    digitalWrite(MOTOR_RIGHT_DIR, right_backward); // Thiết lập hướng
    analogWrite(MOTOR_RIGHT_PWM, current_pwm);   // Tốc độ PWM (giá trị từ 0 đến 255)
    delay(delay_between_steps);
    }
    
  }
  
  // Dừng động cơ
  analogWrite(MOTOR_LEFT_PWM, 0);  // Dừng động cơ trái
  analogWrite(MOTOR_RIGHT_PWM, 0); // Dừng động cơ phải

    // Reset lại số xung để tính toán tốc độ trong lần tiếp theo
    leftPulseCount = 0;
    rightPulseCount = 0;
    return;
}

void motor_turn(int pulse_count, int angle){
 if(angle == 90){
  while (leftPulseCount < pulse_count && rightPulseCount < pulse_count){
    // Điều khiển động cơ trái tiến
    digitalWrite(MOTOR_LEFT_DIR, left_forward);  // Thiết lập hướng
    analogWrite(MOTOR_LEFT_PWM, 75);    // Tốc độ PWM (giá trị từ 0 đến 255)

    // Điều khiển động cơ phải lui
    digitalWrite(MOTOR_RIGHT_DIR, right_backward); // Thiết lập hướng
    analogWrite(MOTOR_RIGHT_PWM, 75);   // Tốc độ PWM (giá trị từ 0 đến 255)
    }
  }else if(angle == -90){
  while (leftPulseCount < pulse_count && rightPulseCount < pulse_count){
    // Điều khiển động cơ trái lui
    digitalWrite(MOTOR_LEFT_DIR, left_backward);  // Thiết lập hướng
    analogWrite(MOTOR_LEFT_PWM, 100);    // Tốc độ PWM (giá trị từ 0 đến 255)

    // Điều khiển động cơ phải tiến
    digitalWrite(MOTOR_RIGHT_DIR, right_forward); // Thiết lập hướng
    analogWrite(MOTOR_RIGHT_PWM, 100);   // Tốc độ PWM (giá trị từ 0 đến 255)
  }
  }
  else if(angle == 180){
    while (leftPulseCount < pulse_count && rightPulseCount < pulse_count){
    // Điều khiển động cơ trái lui
    digitalWrite(MOTOR_LEFT_DIR, left_backward);  // Thiết lập hướng
    analogWrite(MOTOR_LEFT_PWM, 150);    // Tốc độ PWM (giá trị từ 0 đến 255)

    // Điều khiển động cơ phải tiến
    digitalWrite(MOTOR_RIGHT_DIR, right_forward); // Thiết lập hướng
    analogWrite(MOTOR_RIGHT_PWM, 150);   // Tốc độ PWM (giá trị từ 0 đến 255)
    }
  }
  //reset motor
  analogWrite(MOTOR_LEFT_PWM, 0);  // Dừng động cơ trái
  analogWrite(MOTOR_RIGHT_PWM, 0); // Dừng động cơ phải
 }


//controller (in use)
void positon_control() {
  front_threshold_diff = front_right_threshold - front_left_threshold;
  //read_sensor();
  front_value_diff = front_right - front_left;

  int avr_threshold = (front_left_threshold + front_right_threshold) / 2;
  int avr_value = (front_left + front_right) / 2;

  // Tính toán lỗi và điều chỉnh PD
  int error = avr_value - avr_threshold;
  position_pd = error * position_Kp + (error - last_position_error) * position_Kd;
  last_position_error = error;

  // Giới hạn PD trong khoảng 0-255 để đảm bảo PWM hợp lệ
  position_pd = constrain(position_pd, -255, 255);

  // Xuất giá trị lỗi và điều chỉnh
  //Serial.print("Error: "); Serial.print(error);
  //Serial.print(" | position_pd: "); Serial.println(position_pd);
}

void motor_controller(int dir, int target_left_pwm, int target_right_pwm) {
  static int current_left_pwm = 0;             // PWM hiện tại
  static int current_right_pwm = 0;            // PWM hiện tại
  static unsigned long last_update = 0;  // Thời gian cập nhật cuối cùng
  int avr_pulse = (leftPulseCount + rightPulseCount)/2;
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
  } else {
    // Tiến
    digitalWrite(MOTOR_LEFT_DIR, left_forward);
    digitalWrite(MOTOR_RIGHT_DIR, right_forward);
  }

  // Gửi tín hiệu PWM
  analogWrite(MOTOR_LEFT_PWM, current_left_pwm);
  analogWrite(MOTOR_RIGHT_PWM, current_right_pwm);

  // In thông tin PWM
  //Serial.print("Current_left_PWM: "); Serial.print(current_left_pwm); Serial.print(" Current_right_PWM: "); Serial.println(current_right_pwm);
  Serial.println("Số cell đi được: " + String((int)((avr_pulse / FULL_CELL) + 0.5)));
  }

void steering_controller(){
  //read_sensor();
  int last_crawling  = 0; //0 left, 1 right
  int error;
  if (side_left > side_left_threshold && side_right > side_right_threshold) {
    //Serial.println("See both sides");
    error = side_left - side_right;
    // Xuất giá trị lỗi và điều chỉnh
    // Serial.print("Error: "); Serial.print(error);
    // Serial.print(" | steering_pd: "); Serial.println(steering_pd);
  } else if (side_left > side_left_threshold/2) {
    //Serial.println("See left side");
    error = side_left_threshold - side_left;
    // Xuất giá trị lỗi và điều chỉnh
    // Serial.print("left Error: "); Serial.println(error);
    last_crawling = 0;
    // Serial.print(" | steering_pd: "); Serial.println(steering_pd);
  } else if (side_right > side_right_threshold/2) {
    //Serial.println("See right side");
    error = -1*(side_right_threshold - side_right);
    last_crawling = 1;
    // Xuất giá trị lỗi và điều chỉnh
    // Serial.print("right Error: "); Serial.print(error);
    // Serial.print(" | steering_pd: "); Serial.println(steering_pd);
  } else if(side_left < side_left_threshold/2 && side_right < side_right_threshold/2){
    if (last_crawling == 0){
      error = -1 * last_steeing_error;
    } else
    {
      error = -1 * last_steeing_error;
    }

  }
    steering_pd = error * steering_Kp + (error - last_steeing_error) * steering_Kd;
    last_steeing_error = error;
    steering_pd = constrain(steering_pd, -255, 255);
    // Xuất giá trị lỗi và điều chỉnh
    // Serial.print("Error: "); Serial.println(error);
    // Lệch phải error dương, lệch trái error âm 
    // Serial.print(" | steering_pd: "); Serial.println(steering_pd);
}

void merge_controller(int dir ,float position_pd, float steeing_pd){
  float left_motor_output;
  float right_motor_output;
  if (dir == 1){
    
    motor_controller(dir, left_motor_output, right_motor_output); 

  } else if (dir == 0){
    if (steering_pd < 0) {
     left_motor_output = constrain(position_pd  + steeing_pd,0,255); //left motor output
     right_motor_output = constrain(position_pd - steeing_pd ,0,255); //right motor output 
    }
    else{
     left_motor_output = constrain(position_pd + steeing_pd,0,255); //left motor output
     right_motor_output = constrain(position_pd - steeing_pd ,0,255); //right motor output
    }
    motor_controller(dir, left_motor_output, right_motor_output); 
  }
  // Serial.print("left pwm output: "); Serial.print(left_motor_output); Serial.print(" | right pwm output: "); Serial.print(right_motor_output);
}

void read_sensor(){
  //activate emitter
  led_status(0.5);
  digitalWrite(EMITTER_FRONT, HIGH);
  digitalWrite(EMITTER_SIDE, HIGH);
  //read sensor data
  front_left = analogRead(SENSOR_0);
  front_right = analogRead(SENSOR_3);
  side_left = analogRead(SENSOR_1);
  side_right = analogRead(SENSOR_2);
  //deactivate emitter
  // digitalWrite(EMITTER_FRONT, LOW);
  // digitalWrite(EMITTER_SIDE, LOW);
}