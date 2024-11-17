import pygame
from collections import deque
# Cấu hình mê cung từ hình ảnh
maze_data = [
    [0b1100, 0b0101, 0b0101, 0b0110, 0b1110],
    [0b1010, 0b1101, 0b0110, 0b1010, 0b1010],
    [0b1001, 0b0110, 0b0001, 0b0000, 0b0010],
    [0b1100, 0b0001, 0b0100, 0b0011, 0b0010],
    [0b1011, 0b1100, 0b0001, 0b0101, 0b0010],
    [0b1100, 0b0001, 0b0100, 0b0100, 0b0011],
    [0b1001, 0b0110, 0b1000, 0b0100, 0b0111],
    [0b1101, 0b0010, 0b1011, 0b1010, 0b1110],
    [0b1100, 0b0000, 0b0100, 0b0000, 0b0011],
    [0b1011, 0b1101, 0b0101, 0b0001, 0b0111]
]

# Kích thước mê cung
ROWS, COLS = 10, 5
CELL_SIZE = 50
SCREEN_WIDTH = COLS * CELL_SIZE
SCREEN_HEIGHT = ROWS * CELL_SIZE

# Khởi tạo Pygame
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Maze")

# Màu sắc
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
ORANGE = (255, 165, 0)

def draw_cell(x, y, walls):
    """Vẽ một ô với các bức tường."""
    top_left = (x, y)
    top_right = (x + CELL_SIZE, y)
    bottom_left = (x, y + CELL_SIZE)
    bottom_right = (x + CELL_SIZE, y + CELL_SIZE)

    # Vẽ các bức tường
    if walls & 0b0100:  # Tường trên
        pygame.draw.line(screen, ORANGE, top_left, top_right, 2)
    if walls & 0b0010:  # Tường phải
        pygame.draw.line(screen, ORANGE, top_right, bottom_right, 2)
    if walls & 0b0001:  # Tường dưới
        pygame.draw.line(screen, ORANGE, bottom_left, bottom_right, 2)
    if walls & 0b1000:  # Tường trái
        pygame.draw.line(screen, ORANGE, top_left, bottom_left, 2)

def draw_maze(maze):
    """Vẽ toàn bộ mê cung."""
    for row in range(ROWS):
        for col in range(COLS):
            x = col * CELL_SIZE
            y = row * CELL_SIZE
            walls = maze[row][col]
            draw_cell(x, y, walls)

start_cell = (0, 9) #(x, y)
end_cell = (4, 0)
# Directions (up, down, left, right)
directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # (x, y)
stack = deque()

def find_positive_move(maze,cordinate):
    x, y = cordinate
    #check positive move
    if ((maze[y][x] & 0b0001) == 0):
        stack.append(directions[2]) # đi lên
        pass
    if ((maze[y][x] & 0b1000) == 0):
        stack.append(directions[0]) # đi phải
        pass
    if ((maze[y][x] & 0b0100) == 0):
        stack.append(directions[3]) # đi xuong
        pass
    if ((maze[y][x] & 0b0010) == 0):
        stack.append(directions[1]) # đi trái
        pass
    result = tuple(a + b for a, b in zip(cordinate, tuple(stack.pop())))
    print(result)
    
find_positive_move(maze_data,start_cell)
# Vòng lặp chính
# running = True
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

#     screen.fill(WHITE)
#     draw_maze(maze_data)
#     pygame.display.flip()

# pygame.quit()



