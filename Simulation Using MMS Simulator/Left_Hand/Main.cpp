#include "API.h"
#include <iostream>
#include <string>

enum class Direction {
    North,
    East,
    South,
    West
};

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

int main(int argc, char* argv[]) {
    log("Running...");
    API::setColor(0, 0, 'G');  // Set the starting cell color to green
    API::setText(0, 0, "S");   // Mark the starting position with 'S'

    int x = 0, y = 0;  // Starting coordinates of the mouse
    Direction dir = Direction::North;  // Initial direction of the mouse

    while (true) {
        int next_x = x, next_y = y;
        switch (dir) {
            case Direction::North:
                next_y += 1;
                break;
            case Direction::East:
                next_x += 1;
                break;
            case Direction::South:
                next_y -= 1;
                break;
            case Direction::West:
                next_x -= 1;
                break;
        }

        if (!API::wallLeft()) {
            API::turnLeft();
            dir = static_cast<Direction>((static_cast<int>(dir) + 3) % 4); // Rotate direction counter-clockwise
            // Recalculate next position after turn
            next_x = x;
            next_y = y;
            switch (dir) {
                case Direction::North:
                    next_y += 1;
                    break;
                case Direction::East:
                    next_x += 1;
                    break;
                case Direction::South:
                    next_y -= 1;
                    break;
                case Direction::West:
                    next_x -= 1;
                    break;
            }
        }

        if (!API::wallFront()) {
            // Set the color of the next cell to green before moving
            API::setColor(next_x, next_y, 'G');
            API::moveForward();
            // Update the current position after moving
            x = next_x;
            y = next_y;
        } else {
            // If there's a wall in front, keep turning right until no wall is in front
            while (API::wallFront()) {
                API::turnRight();
                dir = static_cast<Direction>((static_cast<int>(dir) + 1) % 4); // Rotate direction clockwise
            }
        }
    }
}