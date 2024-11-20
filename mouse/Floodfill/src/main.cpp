#include <Arduino.h>
#include "maze.h"


void checkAvailablemove(int y, int x);
void addElementToArray(int x, int y, byte dir);


void setup() {
  Serial.begin(115200);
  checkAvailablemove(start_cols, start_rows);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void checkAvailablemove(int y, int x) {
  byte cell = maze[x][y];
  bool frontAvailable;
  if (cell & (1 << dir_format[0])){Serial.println("left available");}
  if (cell & (1 << dir_format[2])){Serial.println("right available");}
  if (cell & (1 << dir_format[3])){Serial.println("back available");}
  if (cell & (1 << dir_format[1])){Serial.println("front available");}
  //kiểm tra xem là dfs hay bfs, chuyển tới cell tiếp theo và lưu các cell còn lại vào stack
  if (flood_type == false){ //dfs
    //move front 
    x = dir_move[1][0] + x;
    y = dir_move[1][1] + y;
    Serial.print(x); Serial.print(" "); Serial.println(y);
  //addElementToArray()
  }
  // Serial.println((cell & dir_format[0]));  
  // Serial.println((cell & dir_format[2]));
  // Serial.println((cell & dir_format[3]));
  // Serial.println((cell & dir_format[1]));
}

void addElementToArray(int x, int y, byte dir) {
  if (size >= 50) { // Kiểm tra nếu mảng đầy
    Serial.println("Mảng đã đầy, không thể thêm phần tử mới.");
    return;
  }
}