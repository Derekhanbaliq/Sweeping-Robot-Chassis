#include <Arduino.h>
#line 1 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
/*
 * Author       :  Derek Zhou
 * Date         :  07/26/2021
 * Version      :  Generation 1 Version 3 Pattern 1
 * New Function :  Protocol-PID Ring + 4 wheels
 */

/********************************include*******************************/
#include <FlexiTimer2.h> //Pin9 & Pin10 cannot be used for PWM!

/********************************define********************************/
#define FRAME_BUFFER_SIZE   3
#define TIMER_PERIOD        20
#define TRANSMISSION_RATIO  90

typedef struct
{
    bool StartFlag;
    bool FinishFlag;
    char Buffer[FRAME_BUFFER_SIZE];
    uint8_t BufferCnt;
}Struct_FrameBuffer;

typedef struct
{
    char RobotStatus;
    int8_t RobotSpeed;
    int8_t ExpectWheelSpeed[4];
}Struct_RobotCommand;

typedef struct
{
    int16_t EncoderCnt[4];
    float MeasuredWheelSpeed[4];
}Struct_EncoderData;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Error;
    float Integral;
    float Derivative;
    float PreError;
    float ILimit;
    float Output;
}Struct_Pid;

/********************************extern********************************/


/********************************global********************************/
const uint8_t Pin_Serial0Rx=0;
const uint8_t Pin_Serial0Tx=1;
const uint8_t Pin_Serial3Rx=15;
const uint8_t Pin_Serial3Tx=14;

const uint8_t Pin_Motor1Enable=4;
const uint8_t Pin_Motor1In1=22;
const uint8_t Pin_Motor1In2=23;
const uint8_t Pin_Motor2Enable=5;
const uint8_t Pin_Motor2In1=24;
const uint8_t Pin_Motor2In2=25;
const uint8_t Pin_Motor3Enable=6;
const uint8_t Pin_Motor3In1=50;
const uint8_t Pin_Motor3In2=51;
const uint8_t Pin_Motor4Enable=7;
const uint8_t Pin_Motor4In1=52;
const uint8_t Pin_Motor4In2=53;

const uint8_t Pin_Encoder1A=2; //int0
const uint8_t Pin_Encoder2A=3; //int1
const uint8_t Pin_Encoder3A=19; //int4
const uint8_t Pin_Encoder4A=18; //int5
const uint8_t Pin_Encoder1B=30;
const uint8_t Pin_Encoder2B=31;
const uint8_t Pin_Encoder3B=32;
const uint8_t Pin_Encoder4B=33;

Struct_FrameBuffer FrameBuffer;
Struct_RobotCommand RobotCommand;
Struct_EncoderData EncoderData;
Struct_Pid WheelPid[4];
bool PidFlag[4];

/*******************************function*******************************/
#line 88 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void setup();
#line 98 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void loop();
#line 132 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void SerialInit(void);
#line 137 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void FrameBufferLoad(char inchar, Struct_FrameBuffer *framebuffer);
#line 155 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
bool FrameBufferCheckError(char buffer[]);
#line 173 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void FrameBufferClear(Struct_FrameBuffer *framebuffer);
#line 180 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void RobotCommandLoad(Struct_RobotCommand *robotcmd, char buffer[]);
#line 264 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Serial0PrintFrameBuffer(char buffer[]);
#line 276 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Serial0PrintErrorCheck(char buffer[]);
#line 292 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Serial0PrintAnalysis(Struct_RobotCommand *robotcmd);
#line 356 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Serial0PrintEnding(void);
#line 363 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Serial3PrintOk(void);
#line 368 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void TimerInit(void);
#line 373 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void TimerCallback(void);
#line 404 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void EncoderInit(void);
#line 420 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Encoder1ACallback(void);
#line 445 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Encoder2ACallback(void);
#line 470 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Encoder3ACallback(void);
#line 495 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Encoder4ACallback(void);
#line 520 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
float EncoderCalculateSpeed(int16_t cnt);
#line 533 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void EncoderGetWheelsSpdRpm(void);
#line 540 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Serial0PrintEncoderCntAndWheelSpdRpm(void);
#line 556 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void EncoderCntClearOut(void);
#line 566 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void MotorInit(void);
#line 594 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void RobotStatusAndPidFlagConfig(char status, bool pidflag[]);
#line 650 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void RobotPinAndPwmConfig(int pwm[], bool pidflag[]);
#line 793 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Serial0PrintCalculatedPwmOutput(int pwmoutput[]);
#line 802 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Serial0PrintRealPwmOutput(int pwmoutput[]);
#line 811 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void Serial0PrintMotorPinLevel(void);
#line 822 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void PidInit();
#line 844 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
float PositionPid(Struct_Pid *pid, float expectvalue, float measurevalue);
#line 88 "e:\\Z SenseTime 商汤科技 实习\\2 扫地机器人\\2 Arduino代码\\G2V1E1\\G2V1E1.ino"
void setup()
{
    SerialInit();
    MotorInit();
    EncoderInit();
    PidInit();
    TimerInit();

    Serial.println("\nRobot has been initialized!");
}
void loop()
{   
    char InChar;
    bool ErrorCmdFlag;
    
    while(Serial3.available() && FrameBuffer.FinishFlag==false) //jump out as long as "true"
    {
        InChar=(char)Serial3.read();
        FrameBufferLoad(InChar, &FrameBuffer);
        delay(10); //wait the input char to be safely received into the buffer
    }
    if(FrameBuffer.FinishFlag==true)
    {
        Serial3PrintOk();

        Serial0PrintFrameBuffer(FrameBuffer.Buffer);
        
        Serial0PrintErrorCheck(FrameBuffer.Buffer);
        ErrorCmdFlag=FrameBufferCheckError(FrameBuffer.Buffer);
        
        if(!ErrorCmdFlag)
        {
            RobotCommandLoad(&RobotCommand, FrameBuffer.Buffer); //get robot status & speed + wheel expect speed
            Serial0PrintAnalysis(&RobotCommand);
            RobotStatusAndPidFlagConfig(RobotCommand.RobotStatus, PidFlag); //config only when protocol commands
        }
        
        Serial0PrintEnding();
        FrameBufferClear(&FrameBuffer);
    }

    delay(10); // avoid serial jam
}

void SerialInit(void)
{
    Serial.begin(115200);
    Serial3.begin(115200);
}
void FrameBufferLoad(char inchar, Struct_FrameBuffer *framebuffer)
{
    if(inchar=='#')
    {
        framebuffer->StartFlag=true;
        framebuffer->FinishFlag=false;
    }
    else if(framebuffer->StartFlag==true && framebuffer->FinishFlag==false && (framebuffer->BufferCnt>=0 && framebuffer->BufferCnt<FRAME_BUFFER_SIZE))
    {
        framebuffer->Buffer[framebuffer->BufferCnt]=inchar;
        framebuffer->BufferCnt++;
    }
    else if(inchar=='\n') 
    {
        framebuffer->StartFlag=false;
        framebuffer->FinishFlag=true;
    }
}
bool FrameBufferCheckError(char buffer[])
{
    bool errorflag=false; //must initialize!

    if(buffer[0]!='B' && buffer[0]!='H' && buffer[0]!='E' && buffer[0]!='S' && buffer[0]!='W' && buffer[0]!='N' \
    && buffer[0]!='O' && buffer[0]!='P' && buffer[0]!='Q' && buffer[0]!='R' && buffer[0]!='C' && buffer[0]!='A')
    {
        errorflag=true;
    }
    if((buffer[0]=='E' || buffer[0]=='S' || buffer[0]=='W' || buffer[0]=='N' || buffer[0]=='O' || buffer[0]=='P' \
     || buffer[0]=='Q' || buffer[0]=='R' || buffer[0]=='C' || buffer[0]=='A') && !((buffer[1]>='0' && buffer[1]<='9') \
    && (buffer[2]>='0' && buffer[2]<='9')) || ((buffer[0]=='B' || buffer[0]=='H') && ((buffer[1]!='X') || (buffer[2]!='X'))))
    {
        errorflag=true;
    }

    return errorflag;
}
void FrameBufferClear(Struct_FrameBuffer *framebuffer)
{
    framebuffer->StartFlag=false;
    framebuffer->FinishFlag=false;
    framebuffer->Buffer[FRAME_BUFFER_SIZE]={};
    framebuffer->BufferCnt=0;
}
void RobotCommandLoad(Struct_RobotCommand *robotcmd, char buffer[])
{
    robotcmd->RobotStatus=buffer[0];
    robotcmd->RobotSpeed=int(buffer[1]-48)*10+int(buffer[2]-48);
    
    if(robotcmd->RobotStatus=='B' || robotcmd->RobotStatus=='H')
    {
        robotcmd->RobotSpeed=0;
        for(int i=0; i<4; i++)
        {
            robotcmd->ExpectWheelSpeed[i]=0;
        }
    }
    else if(robotcmd->RobotStatus=='E')
    {
        robotcmd->ExpectWheelSpeed[0]=-1*robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[1]=robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[2]=-1*robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[3]=robotcmd->RobotSpeed;
    }
    else if(robotcmd->RobotStatus=='S')
    {
        for(int i=0; i<4; i++)
        {
            robotcmd->ExpectWheelSpeed[i]=-1*robotcmd->RobotSpeed;
        }
    }
    else if(robotcmd->RobotStatus=='W')
    {
        robotcmd->ExpectWheelSpeed[0]=robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[1]=-1*robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[2]=robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[3]=-1*robotcmd->RobotSpeed;
    }
    else if(robotcmd->RobotStatus=='N')
    {
        for(int i=0; i<4; i++)
        {
            robotcmd->ExpectWheelSpeed[i]=robotcmd->RobotSpeed;
        }
    }
    else if(robotcmd->RobotStatus=='O')
    {
        robotcmd->ExpectWheelSpeed[0]=0;
        robotcmd->ExpectWheelSpeed[1]=robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[2]=0;
        robotcmd->ExpectWheelSpeed[3]=robotcmd->RobotSpeed;
    }
    else if(robotcmd->RobotStatus=='P')
    {
        robotcmd->ExpectWheelSpeed[0]=-1*robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[1]=0;
        robotcmd->ExpectWheelSpeed[2]=-1*robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[3]=0;
    }
    else if(robotcmd->RobotStatus=='Q')
    {
        robotcmd->ExpectWheelSpeed[0]=0;
        robotcmd->ExpectWheelSpeed[1]=-1*robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[2]=0;
        robotcmd->ExpectWheelSpeed[3]=-1*robotcmd->RobotSpeed;
    }
    else if(robotcmd->RobotStatus=='R')
    {
        robotcmd->ExpectWheelSpeed[0]=robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[1]=0;
        robotcmd->ExpectWheelSpeed[2]=robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[3]=0;
    }
    else if(robotcmd->RobotStatus=='C')
    {
        robotcmd->ExpectWheelSpeed[0]=-1*robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[1]=-1*robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[2]=robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[3]=robotcmd->RobotSpeed;
    }
    else if(robotcmd->RobotStatus=='A')
    {
        robotcmd->ExpectWheelSpeed[0]=robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[1]=robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[2]=-1*robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[3]=-1*robotcmd->RobotSpeed;
    }
}
void Serial0PrintFrameBuffer(char buffer[])
{
    Serial.print("\n");
    Serial.println("********************************");
    
    Serial.print("Frame Buffer: ");
    for(int i=0; i<FRAME_BUFFER_SIZE; i++)
    {
        Serial.print(buffer[i]);
    }
    Serial.print(".\n");
}
void Serial0PrintErrorCheck(char buffer[])
{
    Serial.println("********************************");

    if(buffer[0]!='B' && buffer[0]!='H' && buffer[0]!='E' && buffer[0]!='S' && buffer[0]!='W' && buffer[0]!='N' \
    && buffer[0]!='O' && buffer[0]!='P' && buffer[0]!='Q' && buffer[0]!='R' && buffer[0]!='C' && buffer[0]!='A')
    {
        Serial.println("Error: wrong status byte!");
    }
    if((buffer[0]=='E' || buffer[0]=='S' || buffer[0]=='W' || buffer[0]=='N' || buffer[0]=='O' || buffer[0]=='P' \
     || buffer[0]=='Q' || buffer[0]=='R' || buffer[0]=='C' || buffer[0]=='A') && !((buffer[1]>='0' && buffer[1]<='9') \
    && (buffer[2]>='0' && buffer[2]<='9')) || ((buffer[0]=='B' || buffer[0]=='H') && ((buffer[1]!='X') || (buffer[2]!='X'))))
    {
        Serial.println("Error: wrong speed bytes!");
    }
}
void Serial0PrintAnalysis(Struct_RobotCommand *robotcmd)
{

    Serial.print("Robot ");
    
    if(robotcmd->RobotStatus=='B')
    {
        Serial.println("brakes.");
    }
    else if(robotcmd->RobotStatus=='H')
    {
        Serial.println("hangs.");
    }
    else if(robotcmd->RobotStatus=='E')
    {
        Serial.print("goes east ");
    }
    else if(robotcmd->RobotStatus=='S')
    {
        Serial.print("goes south ");
    }
    else if(robotcmd->RobotStatus=='W')
    {
        Serial.print("goes west ");
    }
    else if(robotcmd->RobotStatus=='N')
    {
        Serial.print("goes north ");
    }
    else if(robotcmd->RobotStatus=='O')
    {
        Serial.print("goes northeast ");
    }
    else if(robotcmd->RobotStatus=='P')
    {
        Serial.print("goes southeast ");
    }
    else if(robotcmd->RobotStatus=='Q')
    {
        Serial.print("goes southwest ");
    }
    else if(robotcmd->RobotStatus=='R')
    {
        Serial.print("goes northwest ");
    }
    else if(robotcmd->RobotStatus=='C')
    {
        Serial.print("goes clockwise ");
    }
    else if(robotcmd->RobotStatus=='A')
    {
        Serial.print("goes anti-clockwise ");
    }
    
    if(robotcmd->RobotStatus!='B' && robotcmd->RobotStatus!='H')
    {
        Serial.print("with "); Serial.print(robotcmd->RobotSpeed); Serial.println(" rpm wheel speed.");
    }

    for(int i=0; i<4; i++)
    {
        Serial.print("ExpectWheelSpd"); Serial.print(i+1); Serial.print("="); Serial.println(robotcmd->ExpectWheelSpeed[i]);
    }
}
void Serial0PrintEnding(void)
{
    Serial.println("********************************");
    Serial.println("The frame has been executed. ");
    Serial.println("********************************");
    Serial.print("\n");
}
void Serial3PrintOk(void)
{
    Serial3.print("#ok\n");
}

void TimerInit(void)
{
    FlexiTimer2::set(TIMER_PERIOD, 1/1000, TimerCallback);
    FlexiTimer2::start();
}
void TimerCallback(void)
{
    int pwmoutput[4]={0, 0, 0, 0};

    //get measure value - EncoderData.MeasuredWheelSpeed[i]
    EncoderGetWheelsSpdRpm();
    Serial0PrintEncoderCntAndWheelSpdRpm();

    //process PID
    for(int i=0; i<4; i++)
    {
        if(PidFlag[i]==true)
        {
            pwmoutput[i]=int(PositionPid(&WheelPid[i], float(RobotCommand.ExpectWheelSpeed[i]), EncoderData.MeasuredWheelSpeed[i]));
        }
        else if(PidFlag[i]==false)
        {
            pwmoutput[i]=0;
        }
    }
    
    //execute PWM
    // Serial0PrintCalculatedPwmOutput(pwmoutput);
    RobotPinAndPwmConfig(pwmoutput, PidFlag);
    // Serial0PrintRealPwmOutput(pwmoutput);

    // Serial0PrintMotorPinLevel();

    EncoderCntClearOut();
}

void EncoderInit(void)
{
    pinMode(Pin_Encoder1A, INPUT);
    pinMode(Pin_Encoder2A, INPUT);
    pinMode(Pin_Encoder3A, INPUT);
    pinMode(Pin_Encoder4A, INPUT);
    pinMode(Pin_Encoder1B, INPUT);
    pinMode(Pin_Encoder2B, INPUT);
    pinMode(Pin_Encoder3B, INPUT);
    pinMode(Pin_Encoder4B, INPUT);

    attachInterrupt(0, Encoder1ACallback, CHANGE);
    attachInterrupt(1, Encoder2ACallback, CHANGE);
    attachInterrupt(4, Encoder3ACallback, CHANGE);
    attachInterrupt(5, Encoder4ACallback, CHANGE);
}
void Encoder1ACallback(void)
{
    if(digitalRead(Pin_Encoder1A)==HIGH) //A rising
    {
        if(digitalRead(Pin_Encoder1B)==HIGH)
        {
            EncoderData.EncoderCnt[0]++;
        }
        else if(digitalRead(Pin_Encoder1B)==LOW)
        {
            EncoderData.EncoderCnt[0]--;
        }
    }
    else if(digitalRead(Pin_Encoder1A)==LOW) //A falling
    {
        if(digitalRead(Pin_Encoder1B)==HIGH)
        {
            EncoderData.EncoderCnt[0]--;
        }
        else if(digitalRead(Pin_Encoder1B)==LOW)
        {
            EncoderData.EncoderCnt[0]++;
        }
    }
}
void Encoder2ACallback(void)
{
    if(digitalRead(Pin_Encoder2A)==HIGH) //A rising
    {
        if(digitalRead(Pin_Encoder2B)==HIGH)
        {
            EncoderData.EncoderCnt[1]++;
        }
        else if(digitalRead(Pin_Encoder2B)==LOW)
        {
            EncoderData.EncoderCnt[1]--;
        }
    }
    else if(digitalRead(Pin_Encoder2A)==LOW) //A falling
    {
        if(digitalRead(Pin_Encoder2B)==HIGH)
        {
            EncoderData.EncoderCnt[1]--;
        }
        else if(digitalRead(Pin_Encoder2B)==LOW)
        {
            EncoderData.EncoderCnt[1]++;
        }
    }
}
void Encoder3ACallback(void)
{
    if(digitalRead(Pin_Encoder3A)==HIGH) //A rising
    {
        if(digitalRead(Pin_Encoder3B)==HIGH)
        {
            EncoderData.EncoderCnt[2]--;
        }
        else if(digitalRead(Pin_Encoder3B)==LOW)
        {
            EncoderData.EncoderCnt[2]++;
        }
    }
    else if(digitalRead(Pin_Encoder3A)==LOW) //A falling
    {
        if(digitalRead(Pin_Encoder3B)==HIGH)
        {
            EncoderData.EncoderCnt[2]++;
        }
        else if(digitalRead(Pin_Encoder3B)==LOW)
        {
            EncoderData.EncoderCnt[2]--;
        }
    }
}
void Encoder4ACallback(void)
{
    if(digitalRead(Pin_Encoder4A)==HIGH) //A rising
    {
        if(digitalRead(Pin_Encoder4B)==HIGH)
        {
            EncoderData.EncoderCnt[3]--;
        }
        else if(digitalRead(Pin_Encoder4B)==LOW)
        {
            EncoderData.EncoderCnt[3]++;
        }
    }
    else if(digitalRead(Pin_Encoder4A)==LOW) //A falling
    {
        if(digitalRead(Pin_Encoder4B)==HIGH)
        {
            EncoderData.EncoderCnt[3]++;
        }
        else if(digitalRead(Pin_Encoder4B)==LOW)
        {
            EncoderData.EncoderCnt[3]--;
        }
    }
}
float EncoderCalculateSpeed(int16_t cnt)
{
    float spddps, spdrpm, wheelspdrpm;

    spddps=float(cnt)*(float(360)/11/2)/(float(TIMER_PERIOD)/1000); //degree per second
    spdrpm=spddps/float(360)*60;
    wheelspdrpm=spdrpm/TRANSMISSION_RATIO;
    // Serial.print("cnt="); Serial.println(cnt);
    // Serial.print("spddps="); Serial.println(spddps);
    // Serial.print("wheelspdrpm="); Serial.println(wheelspdrpm); 

    return wheelspdrpm;
}
void EncoderGetWheelsSpdRpm(void)
{
    for(int i=0; i<4; i++)
    {
        EncoderData.MeasuredWheelSpeed[i]=EncoderCalculateSpeed(EncoderData.EncoderCnt[i]);
    }
}
void Serial0PrintEncoderCntAndWheelSpdRpm(void)
{
    // Serial.print("Encoder1Cnt="); Serial.println(EncoderData.EncoderCnt[0]);
    // Serial.print("Encoder2Cnt="); Serial.println(EncoderData.EncoderCnt[1]);
    // Serial.print("Encoder3Cnt="); Serial.println(EncoderData.EncoderCnt[2]);
    // Serial.print("Encoder4Cnt="); Serial.println(EncoderData.EncoderCnt[3]);

    // Serial.print("MeasuredWheelSpeed1="); 
    // Serial.println(EncoderData.MeasuredWheelSpeed[0]);
    // Serial.print("Wheel2SpeedRpm="); 
    // Serial.println(EncoderData.MeasuredWheelSpeed[1]);
    // Serial.print("Wheel3SpeedRpm="); 
    // Serial.println(EncoderData.MeasuredWheelSpeed[2]);
    // Serial.print("Wheel4SpeedRpm="); 
    // Serial.println(EncoderData.MeasuredWheelSpeed[3]);
}
void EncoderCntClearOut(void)
{
    //for speed ring position PID
    for(int i=0; i<4; i++)
    {
        EncoderData.EncoderCnt[i]=0;
    }
}


void MotorInit(void)
{
    pinMode(Pin_Motor1Enable, OUTPUT);
    pinMode(Pin_Motor1In1, OUTPUT);
    pinMode(Pin_Motor1In2, OUTPUT);
    pinMode(Pin_Motor2Enable, OUTPUT);
    pinMode(Pin_Motor2In1, OUTPUT);
    pinMode(Pin_Motor2In2, OUTPUT);
    pinMode(Pin_Motor3Enable, OUTPUT);
    pinMode(Pin_Motor3In1, OUTPUT);
    pinMode(Pin_Motor3In2, OUTPUT);
    pinMode(Pin_Motor4Enable, OUTPUT);
    pinMode(Pin_Motor4In1, OUTPUT);
    pinMode(Pin_Motor4In2, OUTPUT);
    
    digitalWrite(Pin_Motor1Enable, LOW);
    digitalWrite(Pin_Motor1In1, HIGH);
    digitalWrite(Pin_Motor1In2, HIGH);
    digitalWrite(Pin_Motor2Enable, LOW);
    digitalWrite(Pin_Motor2In1, HIGH);
    digitalWrite(Pin_Motor2In2, HIGH);
    digitalWrite(Pin_Motor3Enable, LOW);
    digitalWrite(Pin_Motor3In1, HIGH);
    digitalWrite(Pin_Motor3In2, HIGH);
    digitalWrite(Pin_Motor4Enable, LOW);
    digitalWrite(Pin_Motor4In1, HIGH);
    digitalWrite(Pin_Motor4In2, HIGH); //hang 4 motors
}
void RobotStatusAndPidFlagConfig(char status, bool pidflag[])
{    
    if(status=='B')
    {
        digitalWrite(Pin_Motor1In1, LOW);
        digitalWrite(Pin_Motor1In2, LOW);
        digitalWrite(Pin_Motor2In1, LOW);
        digitalWrite(Pin_Motor2In2, LOW);
        digitalWrite(Pin_Motor3In1, LOW);
        digitalWrite(Pin_Motor3In2, LOW);
        digitalWrite(Pin_Motor4In1, LOW);
        digitalWrite(Pin_Motor4In2, LOW);

        for(int i=0; i<4; i++)
        {
            pidflag[i]=false;
        }
    }
    else if(status=='H')
    {
        digitalWrite(Pin_Motor1In1, HIGH);
        digitalWrite(Pin_Motor1In2, HIGH);
        digitalWrite(Pin_Motor2In1, HIGH);
        digitalWrite(Pin_Motor2In2, HIGH);
        digitalWrite(Pin_Motor3In1, HIGH);
        digitalWrite(Pin_Motor3In2, HIGH);
        digitalWrite(Pin_Motor4In1, HIGH);
        digitalWrite(Pin_Motor4In2, HIGH);

        for(int i=0; i<4; i++)
        {
            pidflag[i]=false;
        }
    }
    else if(status=='E' || status=='S' || status=='W' || status=='N' || status=='C' || status=='A')
    {
        for(int i=0; i<4; i++)
        {
            pidflag[i]=true;
        }
    }
    else if(status=='O' || status=='Q')
    {
        pidflag[0]=false;
        pidflag[1]=true;
        pidflag[2]=false;
        pidflag[3]=true;
    }
    else if(status=='P' || status=='R')
    {
        pidflag[0]=true;
        pidflag[1]=false;
        pidflag[2]=true;
        pidflag[3]=false;
    }
}
void RobotPinAndPwmConfig(int pwm[], bool pidflag[])
{
    if(pidflag[0]==true)
    {
        if(pwm[0]==0) //hang
        {
            digitalWrite(Pin_Motor1Enable, LOW);
        }
        else if(pwm[0]>0) //foreward
        {
            digitalWrite(Pin_Motor1In1, HIGH);
            digitalWrite(Pin_Motor1In2, LOW);
            if(pwm[0]<40) //about 15% - dead zone
            {
                pwm[0]=0;
            }
            else if(pwm[0]>255)
            {
                pwm[0]=255;
            }
            analogWrite(Pin_Motor1Enable, pwm[0]);
        }
        else if(pwm[0]<0) //reverse
        {
            digitalWrite(Pin_Motor1In1, LOW);
            digitalWrite(Pin_Motor1In2, HIGH);
            if(pwm[0]>-40)
            {
                pwm[0]=0;
            }
            else if(pwm[0]<-255)
            {
                pwm[0]=-255;
            }
            analogWrite(Pin_Motor1Enable, abs(pwm[0]));
        }
    }
    if(pidflag[1]==true)
    {
        if(pwm[1]==0) //hang
        {
            digitalWrite(Pin_Motor2Enable, LOW);
        }
        else if(pwm[1]>0) //foreward
        {
            digitalWrite(Pin_Motor2In1, HIGH);
            digitalWrite(Pin_Motor2In2, LOW);
            if(pwm[1]<40)
            {
                pwm[1]=0;
            }
            else if(pwm[1]>255)
            {
                pwm[1]=255;
            }
            analogWrite(Pin_Motor2Enable, pwm[1]);
        }
        else if(pwm[1]<0) //reverse
        {
            digitalWrite(Pin_Motor2In1, LOW);
            digitalWrite(Pin_Motor2In2, HIGH);
            if(pwm[1]>-40)
            {
                pwm[1]=0;
            }
            else if(pwm[1]<-255)
            {
                pwm[1]=-255;
            }
            analogWrite(Pin_Motor2Enable, abs(pwm[1]));
        }
    }
    if(pidflag[2]==true)
    {
        if(pwm[2]==0) //hang
        {
            digitalWrite(Pin_Motor3Enable, LOW);
        }
        else if(pwm[2]>0) //foreward
        {
            digitalWrite(Pin_Motor3In1, HIGH);
            digitalWrite(Pin_Motor3In2, LOW);
            if(pwm[2]<40) //about 15% - dead zone
            {
                pwm[2]=0;
            }
            else if(pwm[2]>255)
            {
                pwm[2]=255;
            }
            analogWrite(Pin_Motor3Enable, pwm[2]);
        }
        else if(pwm[2]<0) //reverse
        {
            digitalWrite(Pin_Motor3In1, LOW);
            digitalWrite(Pin_Motor3In2, HIGH);
            if(pwm[2]>-40)
            {
                pwm[2]=0;
            }
            else if(pwm[2]<-255)
            {
                pwm[2]=-255;
            }
            analogWrite(Pin_Motor3Enable, abs(pwm[2]));
        }
    }
    if(pidflag[3]==true)
    {
        if(pwm[3]==0) //hang
        {
            digitalWrite(Pin_Motor4Enable, LOW);
        }
        else if(pwm[3]>0) //foreward
        {
            digitalWrite(Pin_Motor4In1, HIGH);
            digitalWrite(Pin_Motor4In2, LOW);
            if(pwm[3]<40) //about 15% - dead zone
            {
                pwm[3]=0;
            }
            else if(pwm[3]>255)
            {
                pwm[3]=255;
            }
            analogWrite(Pin_Motor4Enable, pwm[3]);
        }
        else if(pwm[3]<0) //reverse
        {
            digitalWrite(Pin_Motor4In1, LOW);
            digitalWrite(Pin_Motor4In2, HIGH);
            if(pwm[3]>-40)
            {
                pwm[3]=0;
            }
            else if(pwm[3]<-255)
            {
                pwm[3]=-255;
            }
            analogWrite(Pin_Motor4Enable, abs(pwm[3]));
        }
    }
}
void Serial0PrintCalculatedPwmOutput(int pwmoutput[])
{
    Serial.print("Calculated PWM Output: ");
    for(int i=0; i<4; i++)
    {
        Serial.print(pwmoutput[i]); Serial.print(" "); 
    }
    Serial.println(); 
}
void Serial0PrintRealPwmOutput(int pwmoutput[])
{
    Serial.print("Real       PWM Output: ");
    for(int i=0; i<4; i++)
    {
        Serial.print(pwmoutput[i]); Serial.print(" "); 
    }
    Serial.println(); 
}
void Serial0PrintMotorPinLevel(void)
{
    Serial.println("Motor1: ");
    Serial.print("In1:    "); Serial.println(digitalRead(Pin_Motor1In1));
    Serial.print("In2:    "); Serial.println(digitalRead(Pin_Motor1In2));
    Serial.print("Enable: "); Serial.println(digitalRead(Pin_Motor1Enable));
    Serial.println();


}

void PidInit()
{
    WheelPid[0].Kp=4.25;
    WheelPid[0].Ki=0.5;
    WheelPid[0].Kd=0.3;
    WheelPid[0].ILimit=150;

    WheelPid[1].Kp=4.25;
    WheelPid[1].Ki=0.5;
    WheelPid[1].Kd=0.3;
    WheelPid[1].ILimit=150;

    WheelPid[2].Kp=4.25;
    WheelPid[2].Ki=0.5;
    WheelPid[2].Kd=0.3;
    WheelPid[2].ILimit=150;

    WheelPid[3].Kp=4.25;
    WheelPid[3].Ki=0.5;
    WheelPid[3].Kd=0.3;
    WheelPid[3].ILimit=150;
}
float PositionPid(Struct_Pid *pid, float expectvalue, float measurevalue)
{
    pid->Error=expectvalue-measurevalue;
    pid->Integral+=pid->Ki*pid->Error;
    if(pid->Integral>pid->ILimit)
    {
        pid->Integral=pid->ILimit;
    }
    else if(pid->Integral<-pid->ILimit)
    {
        pid->Integral=-1*pid->ILimit;
    }
    pid->Derivative=pid->Error-pid->PreError;

    pid->Output=pid->Kp*pid->Error+pid->Integral+pid->Kd*pid->Derivative;

    pid->PreError=pid->Error;

    return pid->Output;
}


