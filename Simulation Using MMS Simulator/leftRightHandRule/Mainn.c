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

    while (1)
    {
        // Execute the current algorithm
        if (mode == 1) // Left-Hand Rule
        {
            if (!API_wallLeft())
            {
                API_turnLeft();
            }
            while (API_wallFront())
            {
                API_turnRight();
            }
            API_moveForward();
            logMessage("Using Left-Hand Rule...");
        }
        else if (mode == 2) // Right-Hand Rule
        {
            if (!API_wallRight())
            {
                API_turnRight();
            }
            while (API_wallFront())
            {
                API_turnLeft();
            }
            API_moveForward();
            logMessage("Using Right-Hand Rule...");
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
