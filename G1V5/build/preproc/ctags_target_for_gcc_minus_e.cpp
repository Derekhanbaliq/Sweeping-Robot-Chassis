# 1 "e:\\Z SenseTime 商汤科技 实习\\X 其他项目\\扫地机器人\\2 Arduino代码\\Z 双轮\\G1V5\\G1V5.ino"
/*

 * Author       :  Derek Zhou

 * Date         :  07/30/2021

 * Version      :  Generation 1 Version 5

 * New Function :  Protocol-PID Ring + 4 wheels

 */
# 8 "e:\\Z SenseTime 商汤科技 实习\\X 其他项目\\扫地机器人\\2 Arduino代码\\Z 双轮\\G1V5\\G1V5.ino"
/********************************include*******************************/
# 10 "e:\\Z SenseTime 商汤科技 实习\\X 其他项目\\扫地机器人\\2 Arduino代码\\Z 双轮\\G1V5\\G1V5.ino" 2

/********************************define********************************/





typedef struct
{
    bool StartFlag;
    bool FinishFlag;
    char Buffer[3];
    uint8_t BufferCnt;
}Struct_FrameBuffer;

typedef struct
{
    char RobotStatus;
    int8_t RobotSpeed;
    int8_t ExpectWheelSpeed[2];
}Struct_RobotCommand;

typedef struct
{
    int16_t EncoderCnt[2];
    float MeasuredWheelSpeed[2];
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

const uint8_t Pin_Encoder1A=2; //int0
const uint8_t Pin_Encoder2A=3; //int1
const uint8_t Pin_Encoder1B=30;
const uint8_t Pin_Encoder2B=31;

Struct_FrameBuffer FrameBuffer;
Struct_RobotCommand RobotCommand;
Struct_EncoderData EncoderData;
Struct_Pid WheelPid[2];
bool PidFlag[2];

/*******************************function*******************************/
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
    else if(framebuffer->StartFlag==true && framebuffer->FinishFlag==false && (framebuffer->BufferCnt>=0 && framebuffer->BufferCnt<3))
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

    if(buffer[0]!='B' && buffer[0]!='H' && buffer[0]!='S' && buffer[0]!='N' && buffer[0]!='C' && buffer[0]!='A')
    {
        errorflag=true;
    }
    else if((buffer[0]=='S' || buffer[0]=='N' || buffer[0]=='C' || buffer[0]=='A') && !((buffer[1]>='0' && buffer[1]<='9')
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
    framebuffer->Buffer[3]={};
    framebuffer->BufferCnt=0;
}
void RobotCommandLoad(Struct_RobotCommand *robotcmd, char buffer[])
{
    robotcmd->RobotStatus=buffer[0];
    robotcmd->RobotSpeed=int(buffer[1]-48)*10+int(buffer[2]-48);

    if(robotcmd->RobotStatus=='B' || robotcmd->RobotStatus=='H')
    {
        robotcmd->RobotSpeed=0;
        for(int i=0; i<2; i++)
        {
            robotcmd->ExpectWheelSpeed[i]=0;
        }
    }
    else if(robotcmd->RobotStatus=='S')
    {
        for(int i=0; i<2; i++)
        {
            robotcmd->ExpectWheelSpeed[i]=-1*robotcmd->RobotSpeed;
        }
    }
    else if(robotcmd->RobotStatus=='N')
    {
        for(int i=0; i<2; i++)
        {
            robotcmd->ExpectWheelSpeed[i]=robotcmd->RobotSpeed;
        }
    }
    else if(robotcmd->RobotStatus=='C')
    {
        robotcmd->ExpectWheelSpeed[0]=-1*robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[1]=robotcmd->RobotSpeed;
    }
    else if(robotcmd->RobotStatus=='A')
    {
        robotcmd->ExpectWheelSpeed[0]=robotcmd->RobotSpeed;
        robotcmd->ExpectWheelSpeed[1]=-1*robotcmd->RobotSpeed;
    }
}
void Serial0PrintFrameBuffer(char buffer[])
{
    Serial.print("\n");
    Serial.println("********************************");

    Serial.print("Frame Buffer: ");
    for(int i=0; i<3; i++)
    {
        Serial.print(buffer[i]);
    }
    Serial.print(".\n");
}
void Serial0PrintErrorCheck(char buffer[])
{
    Serial.println("********************************");

    if(buffer[0]!='B' && buffer[0]!='H' && buffer[0]!='S' && buffer[0]!='N' && buffer[0]!='C' && buffer[0]!='A')
    {
        Serial.println("Error: wrong status byte!");
    }
    if((buffer[0]=='S' || buffer[0]=='N' || buffer[0]=='C' || buffer[0]=='A') && !((buffer[1]>='0' && buffer[1]<='9')
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
    else if(robotcmd->RobotStatus=='S')
    {
        Serial.print("goes south ");
    }
    else if(robotcmd->RobotStatus=='N')
    {
        Serial.print("goes north ");
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

void MotorInit(void)
{
    pinMode(Pin_Motor1Enable, 0x1);
    pinMode(Pin_Motor1In1, 0x1);
    pinMode(Pin_Motor1In2, 0x1);
    pinMode(Pin_Motor2Enable, 0x1);
    pinMode(Pin_Motor2In1, 0x1);
    pinMode(Pin_Motor2In2, 0x1);

    digitalWrite(Pin_Motor1Enable, 0x0);
    digitalWrite(Pin_Motor1In1, 0x1);
    digitalWrite(Pin_Motor1In2, 0x1);
    digitalWrite(Pin_Motor2Enable, 0x0);
    digitalWrite(Pin_Motor2In1, 0x1);
    digitalWrite(Pin_Motor2In2, 0x1); //hang 2 motors
}
void RobotStatusAndPidFlagConfig(char status, bool pidflag[]) //IN1 & IN2
{
    if(status=='B')
    {
        digitalWrite(Pin_Motor1In1, 0x0);
        digitalWrite(Pin_Motor1In2, 0x0);
        digitalWrite(Pin_Motor2In1, 0x0);
        digitalWrite(Pin_Motor2In2, 0x0);

        for(int i=0; i<2; i++)
        {
            pidflag[i]=false;
        }
    }
    else if(status=='H')
    {
        digitalWrite(Pin_Motor1In1, 0x1);
        digitalWrite(Pin_Motor1In2, 0x1);
        digitalWrite(Pin_Motor2In1, 0x1);
        digitalWrite(Pin_Motor2In2, 0x1);

        for(int i=0; i<2; i++)
        {
            pidflag[i]=false;
        }
    }
    else if(status=='S' || status=='N' || status=='C' || status=='A')
    {
        for(int i=0; i<2; i++)
        {
            pidflag[i]=true;
        }
    }
}
void RobotPinAndPwmConfig(int pwm[], bool pidflag[])
{
    if(pidflag[0]==true)
    {
        if(pwm[0]==0) //hang
        {
            digitalWrite(Pin_Motor1Enable, 0x0);
        }
        else if(pwm[0]>0) //foreward
        {
            digitalWrite(Pin_Motor1In1, 0x1);
            digitalWrite(Pin_Motor1In2, 0x0);
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
            digitalWrite(Pin_Motor1In1, 0x0);
            digitalWrite(Pin_Motor1In2, 0x1);
            if(pwm[0]>-40)
            {
                pwm[0]=0;
            }
            else if(pwm[0]<-255)
            {
                pwm[0]=-255;
            }
            analogWrite(Pin_Motor1Enable, ((pwm[0])>0?(pwm[0]):-(pwm[0])));
        }
    }
    if(pidflag[1]==true)
    {
        if(pwm[1]==0) //hang
        {
            digitalWrite(Pin_Motor2Enable, 0x0);
        }
        else if(pwm[1]>0) //foreward
        {
            digitalWrite(Pin_Motor2In1, 0x1);
            digitalWrite(Pin_Motor2In2, 0x0);
            if(pwm[1]<40) //about 15% - dead zone
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
            digitalWrite(Pin_Motor2In1, 0x0);
            digitalWrite(Pin_Motor2In2, 0x1);
            if(pwm[1]>-40)
            {
                pwm[1]=0;
            }
            else if(pwm[1]<-255)
            {
                pwm[1]=-255;
            }
            analogWrite(Pin_Motor2Enable, ((pwm[1])>0?(pwm[1]):-(pwm[1])));
        }
    }
}

void TimerInit(void)
{
    FlexiTimer2::set(20, 1/1000, TimerCallback);
    FlexiTimer2::start();
}
void TimerCallback(void)
{
    int pwmoutput[2];

    //get measure value - EncoderData.MeasuredWheelSpeed[i]
    EncoderGetWheelsSpdRpm();
    Serial0PrintEncoderCntAndWheelSpdRpm();

    //process PID
    for(int i=0; i<2; i++)
    {
        if(PidFlag[i]==true)
        {
            pwmoutput[i]=int(PositionPid(&WheelPid[i], float(RobotCommand.ExpectWheelSpeed[i]), EncoderData.MeasuredWheelSpeed[i]));
            // Serial.print("pwmoutput"); Serial.print((i+1)); Serial.print("="); Serial.println(pwmoutput[i]);
        }
    }

    //execute PWM
    RobotPinAndPwmConfig(pwmoutput, PidFlag);

    EncoderCntClearOut();
}

void EncoderInit(void)
{
    pinMode(Pin_Encoder1A, 0x0);
    pinMode(Pin_Encoder2A, 0x0);
    pinMode(Pin_Encoder1B, 0x0);
    pinMode(Pin_Encoder2B, 0x0);

    attachInterrupt(0, Encoder1ACallback, 1);
    attachInterrupt(1, Encoder2ACallback, 1);
}
void Encoder1ACallback(void)
{
    if(digitalRead(Pin_Encoder1A)==0x1) //A rising
    {
        if(digitalRead(Pin_Encoder1B)==0x1)
        {
            EncoderData.EncoderCnt[0]--;
        }
        else if(digitalRead(Pin_Encoder1B)==0x0)
        {
            EncoderData.EncoderCnt[0]++;
        }
    }
    else if(digitalRead(Pin_Encoder1A)==0x0) //A falling
    {
        if(digitalRead(Pin_Encoder1B)==0x1)
        {
            EncoderData.EncoderCnt[0]++;
        }
        else if(digitalRead(Pin_Encoder1B)==0x0)
        {
            EncoderData.EncoderCnt[0]--;
        }
    }
}
void Encoder2ACallback(void)
{
    if(digitalRead(Pin_Encoder2A)==0x1) //A rising
    {
        if(digitalRead(Pin_Encoder2B)==0x1)
        {
            EncoderData.EncoderCnt[1]--;
        }
        else if(digitalRead(Pin_Encoder2B)==0x0)
        {
            EncoderData.EncoderCnt[1]++;
        }
    }
    else if(digitalRead(Pin_Encoder2A)==0x0) //A falling
    {
        if(digitalRead(Pin_Encoder2B)==0x1)
        {
            EncoderData.EncoderCnt[1]++;
        }
        else if(digitalRead(Pin_Encoder2B)==0x0)
        {
            EncoderData.EncoderCnt[1]--;
        }
    }
}
float EncoderCalculateSpeed(int16_t cnt)
{
    float spddps, spdrpm, wheelspdrpm;

    spddps=float(cnt)*(float(360)/11/2)/(float(20)/1000); //degree per second
    spdrpm=spddps/float(360)*60;
    wheelspdrpm=spdrpm/90;
    // Serial.print("cnt="); Serial.println(cnt);
    // Serial.print("spddps="); Serial.println(spddps);
    // Serial.print("wheelspdrpm="); Serial.println(wheelspdrpm); 

    return wheelspdrpm;
}
void EncoderGetWheelsSpdRpm(void)
{
    for(int i=0; i<2; i++)
    {
        EncoderData.MeasuredWheelSpeed[i]=EncoderCalculateSpeed(EncoderData.EncoderCnt[i]);
    }
}
void Serial0PrintEncoderCntAndWheelSpdRpm(void)
{
    Serial.print("Encoder1Cnt="); Serial.println(EncoderData.EncoderCnt[0]);
    // Serial.print("Encoder2Cnt="); Serial.println(EncoderData.EncoderCnt[1]);

    // Serial.print("Wheel1SpeedRpm="); 
    Serial.println(EncoderData.MeasuredWheelSpeed[0]);
    // Serial.print("Wheel2SpeedRpm="); 
    // Serial.println(EncoderData.MeasuredWheelSpeed[1]);
}
void EncoderCntClearOut(void)
{
    //for speed ring position PID
    for(int i=0; i<2; i++)
    {
        EncoderData.EncoderCnt[i]=0;
    }
}

void PidInit()
{
    WheelPid[0].Kp=4.5;
    WheelPid[0].Ki=0.5;
    WheelPid[0].Kd=0.3;
    WheelPid[0].ILimit=150;

    WheelPid[1].Kp=4.5;
    WheelPid[1].Ki=0.5;
    WheelPid[1].Kd=0.3;
    WheelPid[1].ILimit=150;
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
