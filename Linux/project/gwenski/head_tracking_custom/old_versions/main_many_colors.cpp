
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#define INI_FILE_PATH "config.ini"

#define U2D_DEV_NAME "/dev/ttyUSB0"

int main()
{

    ColorFinder *green_finder = new ColorFinder(120, 45, 35, 0, 0.3, 40.0);
    ColorFinder *blue_finder = new ColorFinder(225, 15, 45, 0, 0.3, 50.0);
    // created 2 new ColorFinder objects to find color green and blue

    BallTracker marker_tracker = BallTracker();
    // created a new BallTracker object to use camera tracking

    // inside while loop
    while (1)
    {
        else if (StatusCheck::m_cur_mode == SPRINT) // under a new mode called Sprint
        {
            Point2D green, blue, center;
            // created 3 objects of Point2D class

            green = green_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
            blue = blue_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
            // store position of green and blue color

            if (green.X < 0 || blue.X < 0)
                center.X = -1;
            else
                center.X = (green.X + blue.X) / 2;
            if (green.Y < 0 || blue.Y < 0)
                center.Y = -1;
            else
                center.Y = (green.Y + blue.Y) / 2;
            // calculate center of blue and green

            marker_tracker.process(center);
            // use camera tracking
        }
    }
}