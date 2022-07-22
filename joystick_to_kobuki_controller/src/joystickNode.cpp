#include <SDL2/SDL.h>
#include <ros/ros.h>
#include <iostream>
//maximum of num: 32800
//minimum of num: 3200
int scaleTo100(int num)
{
    int result = (abs(num) - 3200) * 100
    result /= 32800 - 3200;

    if (num < 0)
    {
        result *= -1;
    }
    return int(round(result));
}

int main()
{
    ros::NodeHandle n;
    ros::Publisher joystick_pub = n.advertise<std_msgs::Int16MultiArray>("joystick", 10);
    
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0)
    {
        cout << "Error initializing SDL library: " << SDL_GetError();
        exit(1);
    }
    else
    {
        cout << "Joystick detected! There are: " << SDL_NumJoysticks() << " joysticks." << endl;
    }

    SDL_JoystickEventState(SDL_ENABLE);
    SDL_Joystick *joystick = SDL_JoystickOpen(0);

    int xPos = 0;
    int yPos = 0;

    bool gameIsRunning = true;

    while (gameIsRunning)
    {
        SDL_Event event;
        while(SDL_PollEvent(&event))
        {
            switch (event.type)
            {
                case SDL_JOYAXISMOTION:
                    if ((event.jaxis.value <-3200) || (event.jaxis.value > 3200))
                    {
                        if (event.jaxis.axis == 0) //getting values for the x-axis
                        {
                            xPos = event.jaxis.value;
                        }

                        if (event.jaxis.axis == 1) //getting values for the y-axis
                        {
                            yPos = event.jaxis.value;
                        }
                    }
                    break;
                
                case SDL_JOYBUTTONDOWN:
                    gameIsRunning = false;
                    break;
                case SDL_QUIT:
                    gameIsRunning = false;
                    break;
            }
            int position[] = {xPos, yPos};
            joystick_pub.publish(position);
        }
        SDL_Delay(100);
    }
    return 0;
}