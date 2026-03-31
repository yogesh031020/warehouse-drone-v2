#ifndef WAREHOUSE_MAP_H
#define WAREHOUSE_MAP_H

// ============================================
//  WAREHOUSE MAP - Grid Layout
// ============================================
//
//  HOW THIS WORKS:
//  ───────────────
//  Your warehouse is divided into a grid of cells.
//  Each cell represents 50cm × 50cm in real life.
//
//  Example: A 5m × 4m room = 10 columns × 8 rows
//
//  Cell values:
//    0 = Free space (drone can fly here)
//    1 = Wall / shelf / obstacle (blocked!)
//    2 = Pickup station
//    3 = Delivery station
//    4 = Home pad (takeoff & landing)
//
//  HOW TO CUSTOMIZE FOR YOUR SPACE:
//  ─────────────────────────────────
//  1. Measure your room (length × width in meters)
//  2. Divide by 0.5 to get grid size
//     Example: 5m × 4m = 10 columns × 8 rows
//  3. Draw your room on graph paper
//  4. Mark walls as 1, open space as 0
//  5. Place stations (2, 3, 4) where you want them
//

#define MAP_COLS 10    // Number of columns (room width / 0.5m)
#define MAP_ROWS 8     // Number of rows (room length / 0.5m)
#define CELL_SIZE_CM 50 // Each cell = 50cm in real world

// The warehouse grid map
// Viewed from ABOVE (bird's eye view):
//
//    Col:  0  1  2  3  4  5  6  7  8  9
//         ┌──┬──┬──┬──┬──┬──┬──┬──┬──┬──┐
// Row 0:  │██│██│██│██│██│██│██│██│██│██│  ← North wall
//         ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤
// Row 1:  │██│🏠│  │  │  │  │  │  │  │██│  ← Home at (1,1)
//         ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤
// Row 2:  │██│  │  │██│██│  │██│██│  │██│  ← Shelves
//         ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤
// Row 3:  │██│  │  │  │  │  │  │  │  │██│  ← Open aisle
//         ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤
// Row 4:  │██│  │  │██│██│  │██│██│  │██│  ← Shelves
//         ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤
// Row 5:  │██│  │  │  │  │  │  │  │  │██│  ← Open aisle
//         ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤
// Row 6:  │██│📦│  │  │  │  │  │  │📮│██│  ← Pickup(1,6) Delivery(8,6)
//         ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤
// Row 7:  │██│██│██│██│██│██│██│██│██│██│  ← South wall
//         └──┴──┴──┴──┴──┴──┴──┴──┴──┴──┘

const uint8_t warehouseMap[MAP_ROWS][MAP_COLS] = {
//   Col:  0  1  2  3  4  5  6  7  8  9
        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },   // Row 0: North wall
        { 1, 4, 0, 0, 0, 0, 0, 0, 0, 1 },   // Row 1: Home(1,1)
        { 1, 0, 0, 1, 1, 0, 1, 1, 0, 1 },   // Row 2: Shelves
        { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },   // Row 3: Aisle
        { 1, 0, 0, 1, 1, 0, 1, 1, 0, 1 },   // Row 4: Shelves
        { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },   // Row 5: Aisle
        { 1, 2, 0, 0, 0, 0, 0, 0, 3, 1 },   // Row 6: Pickup & Delivery
        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },   // Row 7: South wall
};

// === Key Positions (column, row) ===
struct Position {
    int x;  // column
    int y;  // row
};

const Position HOME_POS     = { 1, 1 };  // Takeoff/landing pad
const Position PICKUP_POS   = { 1, 6 };  // Package pickup station
const Position DELIVERY_POS = { 8, 6 };  // Package delivery station

#endif
