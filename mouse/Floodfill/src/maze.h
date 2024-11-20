#define ROWS 10
#define COLS 5

uint8_t maze[ROWS][COLS] = {
    {0b1100,0b0100,0b0100,0b0100,0b0110},
    {0b1000,0b0000,0b0000,0b0000,0b0010},
    {0b1000,0b0000,0b0000,0b0000,0b0010},
    {0b1000,0b0000,0b0000,0b0000,0b0010},
    {0b1000,0b0000,0b0000,0b0000,0b0010},
    {0b1000,0b0000,0b0000,0b0000,0b0010},
    {0b1000,0b0000,0b0000,0b0000,0b0010},
    {0b1000,0b0000,0b0000,0b0000,0b0010},
    {0b1000,0b0000,0b0000,0b0000,0b0010},
    {0b1001,0b0001,0b0001,0b0001,0b0011}
};
//cấu hình đường đi
struct Position {
  int x;
  int y;
  byte dir;
};

// Mảng tĩnh lưu tối đa 50 phần tử
Position arr[50];
int size = 0; // Số lượng phần tử hiện tại trong mảng
// cấu hình floodfill
bool flood_type = false; //false: DFS, true: BFS
// cấu hình hướng duy chuyển ban đầu
//left-front-right-back
uint8_t dir_format[4] = {0b0010, 0b0001, 0b1000, 0b0100};
int8_t dir_move[4][2] = {
  {0, -1},  // Left: Dịch sang trái trên lưới tọa độ (giảm y)
  {1, 0},   // Front: Dịch lên trên lưới tọa độ (tăng x)
  {0, 1},   // Right: Dịch sang phải trên lưới tọa độ (tăng y)
  {-1, 0}   // Back: Dịch xuống dưới lưới tọa độ (giảm x)
};


//tọa độ xuất phảt
const int start_cols = 4;
const int start_rows = 0;
//tọa độ điểm đến
const int end_cols = 0;
const int end_rows = 9;