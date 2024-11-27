#include <Arduino.h>
#include "pin_config.h"
#include "parameter_declare.h"
// put function declarations here:

void read_sensor();
void led_status(float time);
float wall_follower(bool wall);
void adc_read(int state);
void selectFunction(int function);
int findClosestIndex(int target, int array[], int size);
void countLeftPulse();
void countRightPulse();
void update();
bool isButtonPressed();
void floodfill();
void motor_control(int left_speed, int right_speed);
void setup() {
  Serial.begin(115200);
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
  // put your setup code here, to run once:
  // Kích hoạt ngắt ngoài khi có sự thay đổi trên các chân encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_CLK), countLeftPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_CLK), countRightPulse, CHANGE);
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

bool isButtonPressed() {
  return analogRead(FUNCTION_SELECT) == 1023;
}

void read_sensor() {
  //activate emitter
  led_status(0.5);
  digitalWrite(EMITTER_FRONT, HIGH);
  front_left =  map(analogRead(SENSOR_0), 14, 660, 0, 1023);
  front_right = map(analogRead(SENSOR_3), 2, 135, 0, 1023);
  digitalWrite(EMITTER_FRONT, LOW);
  digitalWrite(EMITTER_SIDE, HIGH);
  //read sensor data
  side_left =  map(analogRead(SENSOR_1) , 2, 110, 0, 1023);
  side_right = map(analogRead(SENSOR_2), 2, 680, 0, 1023);
  digitalWrite(EMITTER_SIDE, LOW);
  //deactivate emitter
  // digitalWrite(EMITTER_FRONT, LOW);
  // digitalWrite(EMITTER_SIDE, LOW);
  //Serial.println("front left: " + String(front_left) + " | front right: " + String(front_right) + "side left: " + String(side_left) + " | side right: " + String(side_right));
}

// Hàm ngắt để đếm xung động cơ trái
void countLeftPulse() {
  leftPulseCount++;
}

// Hàm ngắt để đếm xung động cơ phải
void countRightPulse() {
  rightPulseCount++;
}

void led_status(float time) {

  // Tính khoảng thời gian nhấp nháy (ms)
  unsigned long interval = (unsigned long)(time * 100);

  // Lấy thời gian hiện tại
  unsigned long currentMillis = millis();
  unsigned long previousMillis;
  bool ledState;
  // Kiểm tra nếu đã đủ thời gian
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Cập nhật thời gian trước
    ledState = !ledState;           // Đảo trạng thái LED
    digitalWrite(LED_PIN, ledState); // Cập nhật trạng thái LED
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

void selectFunction(int function) {
  while (1)
  {
    if (function == 15){
      update();
      delay(10);
    }
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
 // cần thông qua tính toán floodfill để biết xem hướng đi nào cần ưu tiêu tiếp theo
void floodfill(){
  int dx = end_x - start_x; // tính toán delta x
  int dy = end_y - start_y; // tính toán delta y
  // nếu delta x lớn hơn 0, xu hướng rẽ phải theo góc tọa độ
  // nếu delta x bé hơn 0 , xu hướng rẽ trái theo góc tọa độ
  // nếu delta y lớn hơn 0, xu hướng đi lên theo góc tọa độ
  // nếu delta y bé hơn 0 , xu hướng đi xuống theo góc tọa độ
} 
void update() {
  int steer;
  //đọc giá trị cảm biến
  read_sensor();
  // if(side_left > wall_left_recognition){
  //   //thấy tường trái, kích hoạt bám tường trái, trả về hướng rẽ
  //   see_wall_left = true; //biến kiểm tra bám tường, nếu đang bám mà mất, tắt pid, chuẩn bị rẽ hay gì đó, nói chung kích hàm khác
  //   steer = wall_follower(true);
  // }
  // if(side_left < wall_left_recognition && see_wall_left){
  //   //nếu mất tường trái, chạy thêm xíu nữa rồi bật lại pid cho nó quẹo
  //   see_wall_left = false;
  //   steer = wall_follower(true);
  //   motor_control(base_speed + steer/2, base_speed - steer/2); // đảo hướng
  //   delay(300);
  //   motor_control(0, 0);
  //   while (side_left < wall_left_recognition){ // quay bắt tường trái
  //     read_sensor();
  //     motor_control(50, -50);
  //   }
  //   motor_control(0, 0);
  //   delay(10);
  //   motor_control(50, 50);
  //   delay(100);
  //   motor_control(0, 0);
  //   steer = wall_follower(true);
  // }

  if(side_right > wall_right_recognition){
    //thấy tường phải, kích hoạt bám tường phải, trả về hướng rẽ
    see_wall_right = true; //biến kiểm tra bám tường, nếu đang bám mà mất, tắt pid, chuẩn bị rẽ hay gì đó, nói chung kích hàm khác
    steer = wall_follower(false);
  }
  if(side_right < wall_right_recognition && see_wall_right){
    //nếu mất tường phải, chạy thêm xíu nữa rồi bật lại pid cho nó quẹo
    see_wall_right = false;
    steer = wall_follower(false);
    motor_control(base_speed + steer/2, base_speed - steer/2); // đảo hướng
    delay(300);
    motor_control(0, 0);
    while (side_right < wall_right_recognition){ // quay bắt tường phải
      read_sensor();
      motor_control(-50, 50);
    }
    motor_control(0, 0);
    delay(10);
    motor_control(50, 50);
    delay(100);
    motor_control(0, 0);
    steer = wall_follower(false);
  }

  //kết hợp controller
  motor_control(base_speed - steer, base_speed + steer);
  Serial.println("wall_left: " + String(side_left) + " wall_right: " + String(side_right) + " pd_ouput: " + String(steer));
}

float wall_follower(bool wall){
  //nếu wall true, bám trái
  //nếu wall false, bám phải
  if(wall){
    wall_left_error = side_left - wall_left_setpoint;
    steering_pd = steering_KP * wall_left_error + steering_KD * (wall_left_error - last_wall_error);
    last_wall_error = wall_left_error;
    steering_pd = constrain(steering_pd, -30, 30);
    if(steering_pd > -5 && steering_pd < 5){
      steering_pd = 0;
    }
    return steering_pd;
  } else{
    wall_right_error = side_right - wall_right_setpoint;
    steering_pd = steering_KP * wall_right_error + steering_KD * (wall_right_error - last_wall_error);
    last_wall_error = wall_right_error;
    steering_pd = constrain(steering_pd, -30, 30);
    if(steering_pd > -5 && steering_pd < 5){
      steering_pd = 0;
    }
    return -1*steering_pd;
  }
}

void motor_control(int left_speed, int right_speed){
  if (left_speed > 0){
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  }
  else{
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  }
  if (right_speed > 0){
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  }
  else{
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  }
  analogWrite(MOTOR_LEFT_PWM, abs(left_speed));
  analogWrite(MOTOR_RIGHT_PWM, abs(right_speed));
}