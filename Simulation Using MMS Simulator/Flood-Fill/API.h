#ifndef API_H
#define API_H

#include <string>
#include <utility>

namespace API {

    int mazeWidth();
    int mazeHeight();

    void fixWalls(int x, int y, int dir);

    std::pair<int, int> getDir(int dir, int dest);

    bool wallFront();
    bool wallRight();
    bool wallLeft();

    void moveForward(int distance = 1);
    void turnRight();
    void turnLeft();

    void setWall(int x, int y, char direction);
    void clearWall(int x, int y, char direction);

    void setColor(int x, int y, char color);
    void clearColor(int x, int y);
    void clearAllColor();

    void setText(int x, int y, const std::string& text);
    void clearText(int x, int y);
    void clearAllText();

    bool wasReset();
    void ackReset();

} // namespace API

#endif // API_H