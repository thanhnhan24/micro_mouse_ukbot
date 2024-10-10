#include <Arduino.h>

// Khai báo các chân kết nối cho driver TB6612FNG
const int LEFT_PWM = 9;   // Chân PWM cho động cơ trái
const int LEFT_DIR = 8;   // Chân điều khiển hướng cho động cơ trái
const int RIGHT_PWM = 10; // Chân PWM cho động cơ phải
const int RIGHT_DIR = 7;  // Chân điều khiển hướng cho động cơ phải

// Khai báo các chân kết nối cho encoder
const int LEFT_CLK_A = 2;  // Chân đọc xung từ encoder động cơ trái (ngắt ngoài)
const int RIGHT_CLK_A = 3; // Chân đọc xung từ encoder động cơ phải (ngắt ngoài)

volatile long leftPulseCount = 0;  // Biến lưu số xung của động cơ trái
volatile long rightPulseCount = 0; // Biến lưu số xung của động cơ phải
// Hàm ngắt để đếm xung động cơ trái
void countLeftPulse() {
  leftPulseCount++;
}

// Hàm ngắt để đếm xung động cơ phải
void countRightPulse() {
  rightPulseCount++;
}

void setup() {
  // Cài đặt các chân điều khiển động cơ là output
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  // Cài đặt các chân đọc encoder là input
  pinMode(LEFT_CLK_A, INPUT);
  pinMode(RIGHT_CLK_A, INPUT);

  // Kích hoạt ngắt ngoài khi có sự thay đổi trên các chân encoder
  attachInterrupt(digitalPinToInterrupt(LEFT_CLK_A), countLeftPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_CLK_A), countRightPulse, CHANGE);

  // Khởi động Serial để giám sát số xung
  Serial.begin(9600);
}

void loop() {
  // Reset số xung
  leftPulseCount = 0;
  rightPulseCount = 0;

  // Điều khiển động cơ trái tiến
  digitalWrite(LEFT_DIR, HIGH);  // Thiết lập hướng
  analogWrite(LEFT_PWM, 150);    // Tốc độ PWM (giá trị từ 0 đến 255)

  // Điều khiển động cơ phải tiến
  digitalWrite(RIGHT_DIR, HIGH); // Thiết lập hướng
  analogWrite(RIGHT_PWM, 150);   // Tốc độ PWM (giá trị từ 0 đến 255)

  // Chạy trong 2 giây và đếm xung
  delay(2000);

  // Dừng động cơ
  analogWrite(LEFT_PWM, 0);  // Dừng động cơ trái
  analogWrite(RIGHT_PWM, 0); // Dừng động cơ phải

  // Xuất số xung của hai động cơ ra Serial Monitor
  Serial.print("Số xung động cơ trái: ");
  Serial.println(leftPulseCount);
  Serial.print("Số xung động cơ phải: ");
  Serial.println(rightPulseCount);

  // Nghỉ 2 giây trước khi lặp lại
  delay(2000);
}

