#include <Arduino.h>
#define d_f 11 //Hai mat phat truoc
#define d_r 12 //Hai mat phat cheo
#define right_front 14
#define right_diagonal 15
#define left_front 16
#define left_diagonal 17
#define function_select 20
#define battery_capacity 21


void setup() {
  Serial.begin(115200);
  pinMode(d_f, OUTPUT);
  pinMode(d_r, OUTPUT);
  pinMode(right_front, INPUT);
  pinMode(right_diagonal, INPUT);
  pinMode(left_front, INPUT);
  pinMode(left_diagonal, INPUT);
  pinMode(function_select, INPUT);
  pinMode(battery_capacity, INPUT);
}

void loop() {
  digitalWrite(d_f, HIGH);
  digitalWrite(d_r, LOW);
  Serial.print(analogRead(right_front));
  Serial.print('\t');
  Serial.print(analogRead(right_diagonal));
  Serial.print('\t');
  Serial.print(analogRead(left_front));
  Serial.print('\t');
  Serial.print(analogRead(left_diagonal));
  Serial.print('\t');
  Serial.print(analogRead(function_select));
  Serial.print('\t');
  Serial.println(map(analogRead(battery_capacity),444,1023,0,11.7));
  delay(1);
}
