#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <signal.h>
#include <libgen.h>
#include "cmd_process.h"
#include "mjpg_streamer.h"

#define INI_FILE_PATH       "../../../Data/config.ini"

using namespace Robot;

LinuxCM730 linux_cm730("/dev/ttyUSB0");
CM730 cm730(&linux_cm730);

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
    tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}

void moveRightHand();

bool WaitWhileServoMoving(CM730 &cm730, int servo_id)
{
    int moving_status;
    int timeout = 100; // 100 * 10ms = 1 second timeout
    while (
        cm730.ReadByte(servo_id, MX28::P_MOVING, &moving_status, 0) == CM730::SUCCESS &&
        moving_status == 1 &&
        timeout-- > 0)
    {
        usleep(2*10000);
    }

    if (timeout <= 0)
    {
        printf("\nTimeout: Servo did not reach goal!\n");
        return false;
    }
    return true;
}

int main(int argc, char *argv[])
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

    mjpg_streamer* streamer = new mjpg_streamer(0, 0);
    httpd::ini = ini;

    //////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
        return 0;
    }
    MotionManager::GetInstance()->LoadINISettings(ini);
    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////

    DrawIntro(&cm730);
    MotionManager::GetInstance()->SetEnable(true);
    moveRightHand();

    while(1)
    {
        int ch = _getch();
        if(ch == 0x1b)
        {
            ch = _getch();
            if(ch == 0x5b)
            {
                ch = _getch();
                if(ch == 0x41) // Up arrow key
                    MoveUpCursor();
                else if(ch == 0x42) // Down arrow key
                    MoveDownCursor();
                else if(ch == 0x44) // Left arrow key
                    MoveLeftCursor();
                else if(ch == 0x43)
                    MoveRightCursor();
            }
        }
        else if( ch == '[' )
            DecreaseValue(false);
        else if( ch == ']' )
            IncreaseValue(false);
        else if( ch == '{' )
            DecreaseValue(true);
        else if( ch == '}' )
            IncreaseValue(true);
        else if( ch >= 'A' && ch <= 'z' )
        {
            char input[128] = {0,};
            char *token;
            int input_len;
            char cmd[80];
            char strParam[20][30];
            int num_param;

            int idx = 0;

            BeginCommandMode();

            printf("%c", ch);
            input[idx++] = (char)ch;

            while(1)
            {
                ch = _getch();
                if( ch == 0x0A )
                    break;
                else if( ch == 0x7F )
                {
                    if(idx > 0)
                    {
                        ch = 0x08;
                        printf("%c", ch);
                        ch = ' ';
                        printf("%c", ch);
                        ch = 0x08;
                        printf("%c", ch);
                        input[--idx] = 0;
                    }
                }
                else if( ch >= 'A' && ch <= 'z' )
                {
                    if(idx < 127)
                    {
                        printf("%c", ch);
                        input[idx++] = (char)ch;
                    }
                }
            }

            fflush(stdin);
            input_len = strlen(input);
            if(input_len > 0)
            {
                token = strtok( input, " " );
                if(token != 0)
                {
                    strcpy( cmd, token );
                    token = strtok( 0, " " );
                    num_param = 0;
                    while(token != 0)
                    {
                        strcpy(strParam[num_param++], token);
                        token = strtok( 0, " " );
                    }

                    if(strcmp(cmd, "exit") == 0)
                    {
                        if(AskSave() == false)
                            break;
                    }
                    if(strcmp(cmd, "re") == 0)
                        DrawScreen();
                    else if(strcmp(cmd, "save") == 0)
                    {
                        Walking::GetInstance()->SaveINISettings(ini);
                        SaveCmd();
                    }
                    else if(strcmp(cmd, "mon") == 0)
                    {
                        MonitorCmd();
                    }
                    else if(strcmp(cmd, "help") == 0)
                        HelpCmd();
                    else
                        PrintCmd("Bad command! please input 'help'");
                }
            }

            EndCommandMode();
        }
    }

    DrawEnding();
    return 0;
}

void moveRightHand(){
    cm730.WriteByte(JointData::ID_R_SHOULDER_PITCH, MX28::P_P_GAIN, 6, 0);
    cm730.WriteByte(JointData::ID_R_SHOULDER_ROLL, MX28::P_P_GAIN, 6, 0);
    cm730.WriteByte(JointData::ID_R_ELBOW, MX28::P_P_GAIN, 6, 0);
    cm730.WriteByte(JointData::ID_R_WRIST, MX28::P_P_GAIN, 6, 0);

    cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, 1946, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_SHOULDER_ROLL);

    cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, 1660, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_SHOULDER_PITCH);

    cm730.WriteWord(JointData::ID_R_ELBOW, MX28::P_GOAL_POSITION_L, 2216, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_ELBOW);

    cm730.WriteWord(JointData::ID_R_WRIST, MX28::P_GOAL_POSITION_L, 2263, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_WRIST);
}
