#include <Arduino.h>

#define MAZE_SIZE 16

// Bitmasks for walls
#define WALL_NORTH 1
#define WALL_EAST  2
#define WALL_SOUTH 4
#define WALL_WEST  8

// Maze arrays
uint8_t walls[MAZE_SIZE][MAZE_SIZE];
int distances[MAZE_SIZE][MAZE_SIZE];

// Simple struct for queue coordinates
struct Cell {
    int x, y;
};

// Static circular queue for fast floodfill updates
Cell queue[MAZE_SIZE * MAZE_SIZE];
int head = 0;
int tail = 0;

void push(int x, int y) {
    queue[tail].x = x;
    queue[tail].y = y;
    tail = (tail + 1) % (MAZE_SIZE * MAZE_SIZE);
}

Cell pop() {
    Cell c = queue[head];
    head = (head + 1) % (MAZE_SIZE * MAZE_SIZE);
    return c;
}

bool isQueueEmpty() {
    return head == tail;
}

// 1. Initialize the maze with Manhattan distances to the center
void initMaze() {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            walls[x][y] = 0; // No walls known initially (except borders, handled later)
            
            // Standard target is the center 4 cells for a 16x16 maze
            // x: 7, 8 | y: 7, 8
            int dx = max(0, max(7 - x, x - 8));
            int dy = max(0, max(7 - y, y - 8));
            distances[x][y] = dx + dy;
        }
    }
}

// 2. Check the minimum distance of reachable neighbors
int getMinNeighborDistance(int x, int y) {
    int min_dist = 999;
    if (!(walls[x][y] & WALL_NORTH) && y < MAZE_SIZE - 1) min_dist = min(min_dist, distances[x][y + 1]);
    if (!(walls[x][y] & WALL_EAST)  && x < MAZE_SIZE - 1) min_dist = min(min_dist, distances[x + 1][y]);
    if (!(walls[x][y] & WALL_SOUTH) && y > 0)             min_dist = min(min_dist, distances[x][y - 1]);
    if (!(walls[x][y] & WALL_WEST)  && x > 0)             min_dist = min(min_dist, distances[x - 1][y]);
    return min_dist;
}

// 3. The Core Floodfill Update logic
void updateDistances() {
    while (!isQueueEmpty()) {
        Cell current = pop();
        int cx = current.x;
        int cy = current.y;

        // The center cells should always be 0
        if ((cx == 7 || cx == 8) && (cy == 7 || cy == 8)) continue;

        int min_neighbor = getMinNeighborDistance(cx, cy);

        // If the current distance is incorrect based on walls, update it and push neighbors
        if (distances[cx][cy] != min_neighbor + 1) {
            distances[cx][cy] = min_neighbor + 1;

            // Push all accessible neighbors to the queue to be re-evaluated
            if (!(walls[cx][cy] & WALL_NORTH) && cy < MAZE_SIZE - 1) push(cx, cy + 1);
            if (!(walls[cx][cy] & WALL_EAST)  && cx < MAZE_SIZE - 1) push(cx + 1, cy);
            if (!(walls[cx][cy] & WALL_SOUTH) && cy > 0)             push(cx, cy - 1);
            if (!(walls[cx][cy] & WALL_WEST)  && cx > 0)             push(cx - 1, cy);
        }
    }
}

// 4. Function to call when your sensors detect a new wall
void addWall(int x, int y, uint8_t wallType) {
    if (!(walls[x][y] & wallType)) { // If wall isn't already recorded
        walls[x][y] |= wallType;     // Add wall to current cell
        
        // Add corresponding wall to the neighboring cell
        if (wallType == WALL_NORTH && y < MAZE_SIZE - 1) walls[x][y + 1] |= WALL_SOUTH;
        if (wallType == WALL_EAST  && x < MAZE_SIZE - 1) walls[x + 1][y] |= WALL_WEST;
        if (wallType == WALL_SOUTH && y > 0)             walls[x][y - 1] |= WALL_NORTH;
        if (wallType == WALL_WEST  && x > 0)             walls[x - 1][y] |= WALL_EAST;

        // Push current and affected neighbor to queue to trigger floodfill cascade
        push(x, y);
        updateDistances();
    }
}