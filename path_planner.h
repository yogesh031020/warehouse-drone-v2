#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "warehouse_map.h"

// ============================================
//  A* PATH PLANNER
// ============================================
//
//  WHAT IS A* ?
//  ────────────
//  A* (A-star) is a pathfinding algorithm that finds
//  the SHORTEST path from point A to point B on a grid.
//
//  How it works (simple version):
//  1. Start at point A
//  2. Look at all neighbor cells (up, down, left, right)
//  3. For each neighbor, calculate a SCORE:
//     score = (steps taken so far) + (estimated distance to goal)
//  4. Pick the neighbor with the LOWEST score
//  5. Repeat until you reach point B
//
//  The "estimated distance to goal" is called the HEURISTIC.
//  We use Manhattan distance: |x1-x2| + |y1-y2|
//
//  Example path from Home(1,1) to Delivery(8,6):
//
//     ██ ██ ██ ██ ██ ██ ██ ██ ██ ██
//     ██ 🏠→ → .  .  .  .  .  . ██
//     ██  . ↓ ██ ██  . ██ ██  . ██
//     ██  . → → → → → → ↓  . ██
//     ██  .  . ██ ██  . ██ ██ ↓ ██
//     ██  .  .  .  .  .  .  . ↓ ██
//     ██ 📦  .  .  .  .  .  . 📮 ██
//     ██ ██ ██ ██ ██ ██ ██ ██ ██ ██
//

#define MAX_PATH_LEN 80    // Max waypoints in a path
#define MAX_OPEN_LIST 80   // Max nodes to explore

struct Node {
    int x, y;              // Grid position
    float g;               // Cost from start to here
    float h;               // Estimated cost to goal (heuristic)
    float f;               // Total score: f = g + h
    int parentX, parentY;  // Where we came from (for backtracking)
};

class PathPlanner {
private:
    // Open list: nodes we haven't explored yet
    Node openList[MAX_OPEN_LIST];
    int openCount;

    // Closed list: nodes we already explored
    bool closedList[MAP_ROWS][MAP_COLS];

    // Parent tracking for path reconstruction
    int parentX[MAP_ROWS][MAP_COLS];
    int parentY[MAP_ROWS][MAP_COLS];

    // The final path
    int pathX[MAX_PATH_LEN];
    int pathY[MAX_PATH_LEN];
    int pathLength;
    int currentStep;

    // Manhattan distance heuristic
    float heuristic(int x1, int y1, int x2, int y2) {
        return (float)(abs(x1 - x2) + abs(y1 - y2));
    }

    // Check if a cell is valid (inside map and not a wall)
    bool isWalkable(int x, int y) {
        if (x < 0 || x >= MAP_COLS || y < 0 || y >= MAP_ROWS) return false;
        return warehouseMap[y][x] != 1;  // Not a wall
    }

    // Find node with lowest f score in open list
    int findLowestF() {
        int bestIdx = 0;
        for (int i = 1; i < openCount; i++) {
            if (openList[i].f < openList[bestIdx].f) {
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    // Check if position is already in open list
    int findInOpen(int x, int y) {
        for (int i = 0; i < openCount; i++) {
            if (openList[i].x == x && openList[i].y == y) return i;
        }
        return -1;
    }

    // Remove node from open list by index
    void removeFromOpen(int idx) {
        openList[idx] = openList[--openCount];
    }

public:
    // Find shortest path from start to goal
    // Returns true if path found
    bool findPath(int startX, int startY, int goalX, int goalY) {
        Serial.printf("[PATH] Finding path: (%d,%d) -> (%d,%d)\n",
                       startX, startY, goalX, goalY);

        // Reset everything
        pathLength = 0;
        currentStep = 0;
        openCount = 0;
        memset(closedList, false, sizeof(closedList));
        memset(parentX, -1, sizeof(parentX));
        memset(parentY, -1, sizeof(parentY));

        // Add start node to open list
        float h = heuristic(startX, startY, goalX, goalY);
        openList[0] = { startX, startY, 0.0f, h, h, -1, -1 };
        openCount = 1;

        // 4 directions: right, left, down, up
        int dx[] = { 1, -1, 0, 0 };
        int dy[] = { 0, 0, 1, -1 };

        while (openCount > 0) {
            // Pick node with lowest f score
            int bestIdx = findLowestF();
            Node current = openList[bestIdx];
            removeFromOpen(bestIdx);

            // Already explored? Skip
            if (closedList[current.y][current.x]) continue;

            // Mark as explored
            closedList[current.y][current.x] = true;
            parentX[current.y][current.x] = current.parentX;
            parentY[current.y][current.x] = current.parentY;

            // GOAL REACHED!
            if (current.x == goalX && current.y == goalY) {
                // Reconstruct path by backtracking through parents
                int px = goalX, py = goalY;
                pathLength = 0;

                while (px != -1 && py != -1 && pathLength < MAX_PATH_LEN) {
                    pathX[pathLength] = px;
                    pathY[pathLength] = py;
                    pathLength++;
                    int tmpX = parentX[py][px];
                    int tmpY = parentY[py][px];
                    px = tmpX;
                    py = tmpY;
                }

                // Reverse path (it's currently goal→start, we want start→goal)
                for (int i = 0; i < pathLength / 2; i++) {
                    int tmpX = pathX[i]; pathX[i] = pathX[pathLength-1-i]; pathX[pathLength-1-i] = tmpX;
                    int tmpY = pathY[i]; pathY[i] = pathY[pathLength-1-i]; pathY[pathLength-1-i] = tmpY;
                }

                Serial.printf("[PATH] Found! Length: %d steps\n", pathLength);
                printPath();
                return true;
            }

            // Explore 4 neighbors (up, down, left, right)
            for (int d = 0; d < 4; d++) {
                int nx = current.x + dx[d];
                int ny = current.y + dy[d];

                if (!isWalkable(nx, ny)) continue;
                if (closedList[ny][nx]) continue;

                float newG = current.g + 1.0f;
                float newH = heuristic(nx, ny, goalX, goalY);
                float newF = newG + newH;

                // Check if already in open list with better cost
                int existIdx = findInOpen(nx, ny);
                if (existIdx >= 0) {
                    if (newG < openList[existIdx].g) {
                        openList[existIdx].g = newG;
                        openList[existIdx].f = newF;
                        openList[existIdx].parentX = current.x;
                        openList[existIdx].parentY = current.y;
                    }
                } else if (openCount < MAX_OPEN_LIST) {
                    openList[openCount++] = { nx, ny, newG, newH, newF,
                                              current.x, current.y };
                }
            }
        }

        Serial.println("[PATH] No path found!");
        return false;
    }

    // Print the path to Serial Monitor
    void printPath() {
        Serial.println("[PATH] Route:");
        for (int i = 0; i < pathLength; i++) {
            const char* label = "";
            uint8_t cell = warehouseMap[pathY[i]][pathX[i]];
            if (cell == 4) label = " [HOME]";
            else if (cell == 2) label = " [PICKUP]";
            else if (cell == 3) label = " [DELIVERY]";

            Serial.printf("  Step %2d: (%d, %d)%s\n", i, pathX[i], pathY[i], label);
        }
    }

    // Print the map with path marked
    void printMapWithPath() {
        Serial.println("\n[MAP] Warehouse with path:");
        for (int y = 0; y < MAP_ROWS; y++) {
            Serial.print("  ");
            for (int x = 0; x < MAP_COLS; x++) {
                // Check if this cell is on the path
                bool onPath = false;
                for (int p = 0; p < pathLength; p++) {
                    if (pathX[p] == x && pathY[p] == y) {
                        onPath = true;
                        break;
                    }
                }

                if (onPath) {
                    Serial.print(" * ");  // Path marker
                } else {
                    switch (warehouseMap[y][x]) {
                        case 1:  Serial.print("███"); break;  // Wall
                        case 2:  Serial.print(" P "); break;  // Pickup
                        case 3:  Serial.print(" D "); break;  // Delivery
                        case 4:  Serial.print(" H "); break;  // Home
                        default: Serial.print(" . "); break;  // Free
                    }
                }
            }
            Serial.println();
        }
        Serial.println();
    }

    // Get next waypoint
    Position getNextWaypoint() {
        if (currentStep < pathLength) {
            return { pathX[currentStep], pathY[currentStep] };
        }
        return { -1, -1 };
    }

    // Get current position on path
    Position getCurrentPosition() {
        if (currentStep > 0 && currentStep <= pathLength) {
            return { pathX[currentStep - 1], pathY[currentStep - 1] };
        }
        return { pathX[0], pathY[0] };
    }

    void advanceStep()     { currentStep++; }
    bool isPathComplete()  { return currentStep >= pathLength; }
    int  getPathLength()   { return pathLength; }
    int  getCurrentStep()  { return currentStep; }
};

#endif
