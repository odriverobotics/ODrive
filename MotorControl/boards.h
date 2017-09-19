#ifndef BOARDS_H
#define BOARDS_H

// Start board list
#define BOARD_ODRIVE_V3_3   3
#define BOARD_ODRIVE_V3_2   2
#define BOARD_ODRIVE_V3_1   1

#define MB(board) (MOTHERBOARD==BOARD_##board)

#endif