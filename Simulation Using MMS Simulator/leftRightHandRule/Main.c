#include <stdio.h>
#include "API.h"

// Declare the logMessage function
void logMessage(char *text);

void logMessage(char *text)
{
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

int main(int argc, char *argv[])
{
    logMessage("Running... Switching modes automatically between Left-Hand Rule and Right-Hand Rule.");

    int mode = 1;               // Start with Left-Hand Rule (1 for Left-Hand Rule, 2 for Right-Hand Rule)
    int steps = 0;              // Track the number of steps taken
    const int switchAfter = 20; // Number of steps before switching modes

    int x = 0, y = 0;           // Starting position
    int dir = 0;                // Direction (0: North, 1: East, 2: South, 3: West)

    while (1)
    {
        // Color the current cell based on the mode
        if (mode == 1) // Left-Hand Rule
        {
            API_setColor(x, y, 'G'); // Green for Left-Hand Rule
        }
        else if (mode == 2) // Right-Hand Rule
        {
            API_setColor(x, y, 'B'); // Blue for Right-Hand Rule
        }

        // Execute the current algorithm
        if (mode == 1) // Left-Hand Rule
        {
            if (!API_wallLeft())
            {
                API_turnLeft();
                dir = (dir + 3) % 4; // Turn left (counter-clockwise)
            }
            while (API_wallFront())
            {
                API_turnRight();
                dir = (dir + 1) % 4; // Turn right (clockwise)
            }
            API_moveForward();
            logMessage("Using Left-Hand Rule...");
        }
        else if (mode == 2) // Right-Hand Rule
        {
            if (!API_wallRight())
            {
                API_turnRight();
                dir = (dir + 1) % 4; // Turn right (clockwise)
            }
            while (API_wallFront())
            {
                API_turnLeft();
                dir = (dir + 3) % 4; // Turn left (counter-clockwise)
            }
            API_moveForward();
            logMessage("Using Right-Hand Rule...");
        }

        // Update the robot's position based on the direction
        switch (dir)
        {
        case 0: // North
            y += 1;
            break;
        case 1: // East
            x += 1;
            break;
        case 2: // South
            y -= 1;
            break;
        case 3: // West
            x -= 1;
            break;
        }

        // Increment the step counter
        steps++;

        // Switch mode automatically after a set number of steps
        if (steps >= switchAfter)
        {
            mode = (mode == 1) ? 2 : 1; // Toggle between 1 and 2
            steps = 0;                  // Reset the step counter
            logMessage(mode == 1 ? "Switching to Left-Hand Rule..." : "Switching to Right-Hand Rule...");
        }
    }

    return 0;
}
