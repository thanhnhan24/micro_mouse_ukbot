#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;

#define PCA9548A_ADDRESS 0x70 // Địa chỉ I2C của PCA9548A

void selectChannel(uint8_t channel) {
  if (channel > 7) return; // Chỉ có 8 kênh từ 0 đến 7
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1 << channel); // Chọn kênh tương ứng
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("PCA9548A test!");
  // Khởi tạo và thay đổi địa chỉ của từng cảm biến
  selectChannel(0);
  if(!sensor1.init()){
    Serial.println("Failed to detect and initialize sensor1");
  }
  else{
    Serial.println("Sensor1 initialized");
  }

  selectChannel(1);
  if(!sensor2.init()){
    Serial.println("Failed to detect and initialize sensor2");
  }
  else{
    Serial.println("Sensor2 initialized");
  }

  selectChannel(2);
  if(!sensor3.init()){
    Serial.println("Failed to detect and initialize sensor3");
  }
  else{
    Serial.println("Sensor3 initialized");
  }

  selectChannel(3);
  if(!sensor4.init()){
    Serial.println("Failed to detect and initialize sensor4");
  }
  else{
    Serial.println("Sensor4 initialized");
  }
}
void loop() {

}
