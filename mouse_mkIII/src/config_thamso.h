const int adc_thesholds[] = {660, 647, 630, 614, 590, 570, 545, 522, 461, 429, 385, 343, 271, 212, 128, 44, 0};
const int adc_offets = 2; //sai so cho swtich

unsigned long previousMillis = 0; // Lưu thời gian lần nhấp nháy trước
bool ledState = false;            // Trạng thái của LED


// Các tham số tốc độ
volatile long leftPulseCount = 0;  // Biến lưu số xung của động cơ trái
volatile long rightPulseCount = 0; // Biến lưu số xung của động cơ phải

int encoder_revolutions = 1440;
int distance_per_revolution = 30 * PI * encoder_revolutions;
float encoder_bias = 0.98;
const int acceleration_step = 5;  // Bước tăng PWM mỗi lần
const int delay_between_steps = 20;  // Độ trễ giữa các lần tăng PWM (ms)
const int lower_threshold_pwm = 30; // Giá trị pwm mà dưới ngưỡng này mô hình ngưng chuyển động
const int lower_duration = 300; //khoảng thời gian để make sure rằng mô hình ngưng chuyển động (ms)
int current_left_pwm = 0;             // PWM hiện tại
int current_right_pwm = 0;            // PWM hiện tại
bool isUnderthreshold = false; // Kiểm tra nếu mô hình ngưng chuyển động
unsigned long startTime;
int cell_count = 0; 
int last_cell_count = 0;
//Các tham số cơ bản
int function_value = 0;
float time_interval = 100; // Khoaảng thời gian delay (ms)
float FULL_CELL = encoder_revolutions+encoder_revolutions*(1-encoder_bias);
const unsigned long TURN_TIME_90_DEGREES = 375;  // Đơn vị: ms (thử nghiệm để xác định giá trị chính xác)
//Tham số PID
float position_Kd = 0.01;
float position_Kp = 3.2;

float steering_Kd = 0.00623;
float steering_Kp = 0.112135;

float rotation_Kd = 0.02421232;
float rotation_Kp = 1.12354563;
int last_position_error;
int last_steeing_error;
const int stant_still_pwm = 12; 
//tham so cam bien
int front_left = 0;
int front_right = 0;
int side_left = 0;
int side_right = 0;
int front_value_diff = 0;
int front_left_threshold = 365;
int front_right_threshold = 342;
int side_left_threshold = 105;
int side_right_threshold = 105;
int front_threshold_diff = 0;
int left_value_diff = 0;
int right_value_diff = 0;
//hướng duy chuyển
#define left_forward 0
#define left_backward 1
#define right_forward 1
#define right_backward 0

//tham số flood fill
int start_cell_x = 0;
int start_cell_y = 0;
int end_cell_x = 4;
int end_cell_y = 8;
#define NORTH 0b0010
#define EAST  0b0001
#define SOUTH 0b1000
#define WEST  0b0100
uint8_t currentDirection = NORTH; // Hướng khởi tạo ban đầu