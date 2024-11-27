//tham số swtich 
const int adc_thesholds[] = {660, 647, 630, 614, 590, 570, 545, 522, 461, 429, 385, 343, 271, 212, 128, 44, 0};
const int adc_offets = 2; //sai so cho swtich

// tham số IR
int front_left;
int front_right;
int side_left;
int side_right;

//tham số encoder
int rightPulseCount;
int leftPulseCount;

//tham số tốc độ
int base_speed = 90;

//tham số tường
int wall_left_recognition = 50;
int wall_right_recognition = 50;
int wall_front_recognition = 50;
bool see_wall_left = false;
bool see_wall_right = false;

//tham số PID bám tường
int wall_left_setpoint = 250;
int wall_right_setpoint = 250;
int wall_left_error = 0;
int wall_right_error = 0;
int last_wall_error = 0;

float steering_KP = 0.42;
float steering_KD = 0.001;
float steering_pd;

// Tham số mê cung
#define chieurong 5
#define chieucao 10
int start_x = 0; // vi tri bat dau x
int start_y = 9; // vi tri bat dau y
int end_x = 4; // vi tri ket thuc x
int end_y = 0; // vi tri ket thuc y
uint8_t maze[chieucao][chieurong] = {
    {4, 3, 2, 1, 0},
    {4, 3, 2, 1, 1},
    {4, 3, 2, 2, 2},
    {4, 3, 3, 3, 3},
    {4, 4, 4, 4, 4},
    {5, 5, 5, 5, 5},
    {6, 6, 6, 6, 6},
    {7, 7, 7, 7, 7},
    {8, 8, 8, 8, 8},
    {9, 9, 9, 9, 9}
};
