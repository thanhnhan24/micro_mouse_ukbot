const int adc_thesholds[] = {660, 647, 630, 614, 590, 570, 545, 522, 461, 429, 385, 343, 271, 212, 128, 44, 0};
const int adc_offets = 2; //sai so cho swtich

unsigned long previousMillis = 0; // Lưu thời gian lần nhấp nháy trước
bool ledState = false;            // Trạng thái của LED


// Các tham số tốc độ
volatile long leftPulseCount = 0;  // Biến lưu số xung của động cơ trái
volatile long rightPulseCount = 0; // Biến lưu số xung của động cơ phải

int encoder_revolutions = 840;
int distance_per_revolution = 60 * PI * encoder_revolutions;
float encoder_bias = 0.98;
const int acceleration_step = 2;  // Bước tăng PWM mỗi lần
const int delay_between_steps = 20;  // Độ trễ giữa các lần tăng PWM (ms)
  
//Các tham số cơ bản
int function_value = 0;
float time_interval = 100; // Khoaảng thời gian delay (ms)
float FULL_CELL = encoder_revolutions+encoder_revolutions*(1-encoder_bias);

//Tham số PID
float position_Kd = 0.02;
float position_Kp = 0.7;
float steeing_Kd = 0.02;
float steeing_Kp = 0.7;
int last_position_error;
int last_steeing_error;
const int stant_still_pwm = 12; 
float position_pd;
float steeing_pd;
//tham so cam bien
int front_left = 0;
int front_right = 0;
int side_left = 0;
int side_right = 0;
int front_value_diff = 0;
int front_left_threshold = 231;
int front_right_threshold = 282;
int side_left_threshold = 101;
int side_right_threshold = 113;
int front_threshold_diff = 0;
int left_value_diff = 0;
int right_value_diff = 0;
//hướng duy chuyển
#define left_forward 0
#define left_backward 1
#define right_forward 1
#define right_backward 0