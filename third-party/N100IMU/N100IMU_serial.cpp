#include "N100IMU_serial.h"
static std::mutex dataMutex;


//读取数据需要用到的相关变量
uint8_t IMU_Data[64];       //IMU数据类型长度
uint8_t IMU_Acc_Data[24];   //IMU_Acc数据类型长度,只有IMU机体三轴加速度信息
uint8_t AHRS_Data[56];      //AHRS数据类型长度
uint8_t Fd_IMU_data[64];        //用于存放接收串口数据
uint8_t Fd_IMU_Acc_data[24];        //用于存放接收串口数据
uint8_t Fd_AHRS_data[56];        //用于存放接收串口数据
bool Data_of_IMU = 0;       //用于表示IMU数据已经就绪，可以进行解包
bool Data_of_IMU_Acc = 0;   //用于表示IMU_Acc数据已经就绪，可以进行解包
bool Data_of_AHRS = 0;      //用于表示AHRS数据已经就绪，可以进行解包
bool ledstate;
ssize_t bytes_read;


N100IMU::IMUPacket PacketIMU;         //实例化IMU数据类型对象
N100IMU::IMUAccPacket PacketIMUAcc;   //实例化IMU_Acc数据类型对象
N100IMU::AHRSPacket PacketAHRS;       //实例化AHRS数据类型对象
//N100IMU::Hardware hardware;       


//读取N100惯导模块数据函数

using namespace N100IMU;

void Hardware::Read_N100Data(void)
{  
  static uint8_t Count_IMU=0; //用于计算当前获取到的数据量
  static uint8_t Count_IMU_Acc=0; //用于计算当前获取到的数据量
  static uint8_t Count_AHRS=0; //用于计算当前获取到的数据量
  uint8_t Usart_Receive;  //用于读取串口接收到的数据
  static uint8_t Last_Receive; //用于保存上一次的接收
  static uint8_t Before_Last_Receive; //用于保存上上一次的接收
  uint8_t tmpch[1] ;
  bytes_read = read(fd,tmpch,sizeof(uint8_t));

  if(tmpch != NULL) //串口接收到数据
  {
    //printf("Serial port receive data: %x\n",*tmpch);
    Usart_Receive = *tmpch; //读取串口的数据
    //Fd_IMU_data[Count_IMU] = Usart_Receive; //将接收到的数据填入数据
    Fd_IMU_Acc_data[Count_IMU_Acc] = Last_Receive; //将接收到的数据填入数据
    Fd_AHRS_data[Count_AHRS] = Last_Receive; //将接收到的数据填入数据

    //接收到上一帧的帧尾和本帧帧头则开始计数（较大程度避免数据误读浪费时间，造成数据丢包）
    if((Before_Last_Receive==FRAME_END&&Last_Receive==FRAME_HEAD&&Usart_Receive==TYPE_IMU) || Count_IMU>0)  
    {
      Count_IMU++;
    }
    else if((Before_Last_Receive==FRAME_END&&Last_Receive==FRAME_HEAD&&Usart_Receive==TYPE_IMU_Acc) || Count_IMU_Acc>0)
    {
      Count_IMU_Acc++;
      //printf("Count_IMU_Acc = %d\n",Count_IMU_Acc);
    }
    else if((Before_Last_Receive==FRAME_END&&Last_Receive==FRAME_HEAD&&Usart_Receive==TYPE_AHRS) || Count_AHRS>0)
    {
      Count_AHRS++;
      //printf("Count_AHRS = %d\n",Count_AHRS);
    }
    else
    {
      Count_IMU=0;
      Count_IMU_Acc=0;
      Count_AHRS=0;
    }

    Before_Last_Receive = Last_Receive;   //保存上上次数据
    Last_Receive = Usart_Receive;         //保存本次数据
    
    
    //满足IMU数据长度
    if(Count_IMU==IMU_TYPE_LEN)
    {
      //数据类型、长度、帧尾均满足满足要求
      if(Fd_IMU_data[1]==TYPE_IMU&&Fd_IMU_data[2]==IMU_LEN&&Fd_IMU_data[IMU_TYPE_LEN-1]==FRAME_END)
      {
        Count_IMU = 0;//清空计数等待下次计数
        Count_IMU_Acc = 0;
        Count_AHRS = 0;
        Data_of_IMU=1,memcpy(IMU_Data,Fd_IMU_data,sizeof(Fd_IMU_data));
      }
    }

    //满足IMU_Acc数据长度
    if(Count_IMU_Acc==IMU_Acc_TYPE_LEN)
    {
      //数据类型、长度、帧尾均满足满足要求
      if(Fd_IMU_Acc_data[1]==TYPE_IMU_Acc&&Fd_IMU_Acc_data[2]==IMU_Acc_LEN&&Fd_IMU_Acc_data[IMU_Acc_TYPE_LEN-1]==FRAME_END)
      {
        //Count_IMU = 0;//清空计数等待下次计数
        Count_IMU_Acc = 0;
        //Count_AHRS = 0;
        Data_of_IMU_Acc=1,memcpy(IMU_Acc_Data,Fd_IMU_Acc_data,sizeof(Fd_IMU_Acc_data));
      }
    }

    //满足AHRS数据长度要求
    if(Count_AHRS==AHRS_TYPE_LEN)
    {
      if(Fd_AHRS_data[1]==TYPE_AHRS&&Fd_AHRS_data[2]==AHRS_LEN&&Fd_AHRS_data[AHRS_TYPE_LEN-1]==FRAME_END)
      {
        //Count_IMU = 0;//清空计数等待下次计数
        //Count_IMU_Acc = 0;
        Count_AHRS = 0;
        Data_of_AHRS=1,memcpy(AHRS_Data,Fd_AHRS_data,sizeof(Fd_AHRS_data));
      }
    }

    //均不满足要求，数据超出长度，放弃计数等待下次接收
    //if(Count_IMU>IMU_TYPE_LEN) Count_IMU = 0;
    if(Count_IMU_Acc>IMU_Acc_TYPE_LEN) Count_IMU_Acc = 0;
    if(Count_AHRS>AHRS_TYPE_LEN) Count_AHRS = 0;
   }
}


//数据解包函数
void Hardware::DataUnpacking(void)
{
  //printf("DataUnpacking\n");
  if(Data_of_IMU==1)                               //IMU数据接收完毕，开始解包
  {
    //printf("Data type is IMU, start unpacking\n");
    if(IMU_Data[1]==TYPE_IMU&&IMU_Data[2]==IMU_LEN) //再次校验数据类型和数据长度
    {
      uint8_t temp[4];
      temp[0] = IMU_Data[7] , temp[1] = IMU_Data[8] , temp[2] = IMU_Data[9], temp[3] = IMU_Data[10];
      PacketIMU.gyroscope_x = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[11] , temp[1] = IMU_Data[12] , temp[2] = IMU_Data[13], temp[3] = IMU_Data[14];
      PacketIMU.gyroscope_y = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[15] , temp[1] = IMU_Data[16] , temp[2] = IMU_Data[17], temp[3] = IMU_Data[18];
      PacketIMU.gyroscope_z = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[19] , temp[1] = IMU_Data[20] , temp[2] = IMU_Data[21], temp[3] = IMU_Data[22];
      PacketIMU.accelerometer_x = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[23] , temp[1] = IMU_Data[24] , temp[2] = IMU_Data[25], temp[3] = IMU_Data[26];
      PacketIMU.accelerometer_y = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[27] , temp[1] = IMU_Data[28] , temp[2] = IMU_Data[29], temp[3] = IMU_Data[30];
      PacketIMU.accelerometer_z = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[31] , temp[1] = IMU_Data[32] , temp[2] = IMU_Data[33], temp[3] = IMU_Data[34];
      PacketIMU.magnetometer_x = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[35] , temp[1] = IMU_Data[36] , temp[2] = IMU_Data[37], temp[3] = IMU_Data[38];
      PacketIMU.magnetometer_y = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[39] , temp[1] = IMU_Data[40] , temp[2] = IMU_Data[41], temp[3] = IMU_Data[42];
      PacketIMU.magnetometer_z = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[43] , temp[1] = IMU_Data[44] , temp[2] = IMU_Data[45], temp[3] = IMU_Data[46];
      PacketIMU.imu_temperature = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[47] , temp[1] = IMU_Data[48] , temp[2] = IMU_Data[49], temp[3] = IMU_Data[50];
      PacketIMU.Pressure = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[51] , temp[1] = IMU_Data[52] , temp[2] = IMU_Data[53], temp[3] = IMU_Data[54];
      PacketIMU.pressure_temperature = HEX_to_Float_New(temp,true);

      
      //下面是数据解包过程
      PacketIMU.Timestamp = timestamp(IMU_Data[58],IMU_Data[57],IMU_Data[56],IMU_Data[55]);
     
      //数据解包完毕，清空标志位等待下次数据解包
      Data_of_IMU = 0;

      printf("gyro_x = %f\n", PacketIMU.gyroscope_x);
      printf("gyro_y = %f\n", PacketIMU.gyroscope_y);
      printf("gyro_z = %f\n\n", PacketIMU.gyroscope_z);
      //使用模拟串口打印数据,默认只开通x、y、z三轴打印
//      Serial.print("gyro_x = ");
//      Serial.println(PacketIMU.gyroscope_x,7);
//      Serial.print("gyro_y = ");
//      Serial.println(PacketIMU.gyroscope_y,7);
//      Serial.print("gyro_z = ");
//      Serial.println(PacketIMU.gyroscope_z,7);
//      Serial.print("accel_X = ");
//      Serial.println(PacketIMU.accelerometer_x,7);
//      Serial.print("accel_y = ");
//      Serial.println(PacketIMU.accelerometer_y,7);
//      Serial.print("accel_z = ");
//      Serial.println(PacketIMU.accelerometer_z,7);
//      Serial.print("mag_X = ");
//      Serial.println(PacketIMU.magnetometer_x,7);
//      Serial.print("mag_y = ");
//      Serial.println(PacketIMU.magnetometer_y,7);
//      Serial.print("mag_z = ");
//      Serial.println(PacketIMU.magnetometer_z,7);
//      Serial.print("temperature = ");
//      Serial.println(PacketIMU.imu_temperature);
//      Serial.print("Pressure = ");
//      Serial.println(PacketIMU.Pressure,7);
//      Serial.print("pressure_temperature = ");
//      Serial.println(PacketIMU.pressure_temperature,7);
//      Serial.print("Timestamp = ");
//      Serial.println(PacketIMU.Timestamp);
      usleep(1000);
    }
  }

  if(Data_of_IMU_Acc==1) //IMU_Acc数据接收完毕，开始解包
  {
    //printf("Data type is IMU_Acc, start unpacking\n");
    if(IMU_Acc_Data[1]==TYPE_IMU_Acc&&IMU_Acc_Data[2]==IMU_Acc_LEN) //再次校验数据类型和数据长度
    {
      uint8_t temp[4];

      temp[0] = IMU_Acc_Data[7] , temp[1] = IMU_Acc_Data[8] , temp[2] = IMU_Acc_Data[9], temp[3] = IMU_Acc_Data[10];
      PacketIMUAcc.accelerometer_x = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Acc_Data[11] , temp[1] = IMU_Acc_Data[12] , temp[2] = IMU_Acc_Data[13], temp[3] = IMU_Acc_Data[14];
      PacketIMUAcc.accelerometer_y = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Acc_Data[15] , temp[1] = IMU_Acc_Data[16] , temp[2] = IMU_Acc_Data[17], temp[3] = IMU_Acc_Data[18];
      PacketIMUAcc.accelerometer_z = HEX_to_Float_New(temp,true);

      //Data Parket for Cheetah Software 
      Hardware::acc[0] = PacketIMUAcc.accelerometer_x;
      Hardware::acc[1] = PacketIMUAcc.accelerometer_y;
      Hardware::acc[2] = PacketIMUAcc.accelerometer_z;

      //下面是数据解包过程
      PacketIMUAcc.Timestamp = timestamp(IMU_Acc_Data[22],IMU_Acc_Data[21],IMU_Acc_Data[20],IMU_Acc_Data[19]);
     
      //数据解包完毕，清空标志位等待下次数据解包
      Data_of_IMU_Acc = 0;

      printf("accel_x = %f\n", PacketIMUAcc.accelerometer_x);
      printf("accel_y = %f\n", PacketIMUAcc.accelerometer_y);
      printf("accel_z = %f\n\n", PacketIMUAcc.accelerometer_z);

//      Serial.print("Timestamp = ");
//      Serial.println(PacketIMUAcc.Timestamp);

      usleep(10000);
    }
  }

  if(Data_of_AHRS==1) //AHRS数据接收完毕，开始解包
  {
    if(AHRS_Data[1]==TYPE_AHRS&&AHRS_Data[2]==AHRS_LEN)//再次校验数据类型和数据长度
    {
      uint8_t temp[4];
      temp[0] = AHRS_Data[7] , temp[1] = AHRS_Data[8] , temp[2] = AHRS_Data[9], temp[3] = AHRS_Data[10];
      PacketAHRS.RollSpeed = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[11] , temp[1] = AHRS_Data[12] , temp[2] = AHRS_Data[13], temp[3] = AHRS_Data[14];
      PacketAHRS.PitchSpeed = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[15] , temp[1] = AHRS_Data[16] , temp[2] = AHRS_Data[17], temp[3] = AHRS_Data[18];
      PacketAHRS.HeadingSpeed = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[19] , temp[1] = AHRS_Data[20] , temp[2] = AHRS_Data[21], temp[3] = AHRS_Data[22];
      PacketAHRS.Roll = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[23] , temp[1] = AHRS_Data[24] , temp[2] = AHRS_Data[25], temp[3] = AHRS_Data[26];
      PacketAHRS.Pitch = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[27] , temp[1] = AHRS_Data[28] , temp[2] = AHRS_Data[29], temp[3] = AHRS_Data[30];
      PacketAHRS.Heading = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[31] , temp[1] = AHRS_Data[32] , temp[2] = AHRS_Data[33], temp[3] = AHRS_Data[34];
      PacketAHRS.Qw = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[35] , temp[1] = AHRS_Data[36] , temp[2] = AHRS_Data[37], temp[3] = AHRS_Data[38];
      PacketAHRS.Qx = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[39] , temp[1] = AHRS_Data[40] , temp[2] = AHRS_Data[41], temp[3] = AHRS_Data[42];
      PacketAHRS.Qy = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[43] , temp[1] = AHRS_Data[44] , temp[2] = AHRS_Data[45], temp[3] = AHRS_Data[46];
      PacketAHRS.Qz = HEX_to_Float_New(temp,true);
      PacketAHRS.Timestamp = timestamp(AHRS_Data[50],AHRS_Data[49],AHRS_Data[48],AHRS_Data[47]); //unit: us

      //Data Parket for Cheetah Software 
      Hardware::gyro[0] = PacketAHRS.RollSpeed;
      Hardware::gyro[1] = PacketAHRS.PitchSpeed;
      Hardware::gyro[2] = PacketAHRS.HeadingSpeed;
      Hardware::quat[0] = PacketAHRS.Qw;
      Hardware::quat[1] = PacketAHRS.Qx;
      Hardware::quat[2] = PacketAHRS.Qy;
      Hardware::quat[3] = PacketAHRS.Qz;




      //数据解包完毕，清空标志位等待下次数据解包
      Data_of_AHRS = 0;


      printf("RollSpeed = %f\n", PacketAHRS.RollSpeed);
      printf("PitchSpeed = %f\n", PacketAHRS.PitchSpeed);
      printf("HeadingSpeed = %f\n", PacketAHRS.HeadingSpeed);
      printf("Roll = %f\n", PacketAHRS.Roll);
      printf("Pitch = %f\n", PacketAHRS.Pitch);
      printf("Heading = %f\n", PacketAHRS.Heading);
      printf("Qw = %f\n", PacketAHRS.Qw);
      printf("Qx = %f\n", PacketAHRS.Qx);
      printf("Qy = %f\n", PacketAHRS.Qy);
      printf("Qz = %f\n\n", PacketAHRS.Qz);

      //printf("Timestamp = %f\n\n", PacketAHRS.Timestamp);
      usleep(10000);

    }
  }

}

float Hardware::HEX_to_Float_New(uint8_t *data,bool mode)
{
  float fa=0;
  uint8_t uc[4];

  if(mode==false)
  {
    uc[3] = data[0];
    uc[2] = data[1];
    uc[1] = data[2];
    uc[0] = data[3];
  }
  else
  {
    uc[0] = data[0];
    uc[1] = data[1];
    uc[2] = data[2];
    uc[3] = data[3];
   }
   memcpy(&fa,uc,4);
   return fa;
}

//时间戳解包函数
long long Hardware::timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4)
{
  unsigned long temp; //32位16进制数
  uint16_t H_16,L_16; //存放高16位、低16位

  H_16 = Data_1 << 8 | Data_2;
  L_16 = Data_3 << 8 | Data_4;

  //将融合成16位的数据组合成32位数据
  temp = (unsigned long)H_16<<16|(unsigned long)L_16;

  return temp;
}


void Hardware::updateLCM(LCM::N100IMU_lcm *message) {
  dataMutex.lock();
  for(uint32_t i = 0; i < 4; i++) {
    message->quat[i] = quat[i];
  }

  Vec3<float> rpy = ori::quatToRPY(quat);
  for(uint32_t i = 0; i < 3; i++) {
    message->rpy[i] = rpy[i];
    message->acc[i] = acc[i];
    message->omega[i] = gyro[i];
  }

  message->good_packets = good_packets;
  message->bad_packets = invalid_packets + unknown_packets + timeout_packets;
  dataMutex.unlock();
}

bool Hardware::initIMU(){
    char *path;

    //获取串口设备描述符
    path = (char *)default_path;
    //printf("This is N100 IMU data reading demo.\n");


    //==========串口打开============//
    fd = open(path, O_RDWR | O_NOCTTY);//noctty,not influenced by terminal,
    //fcntl(fd,F_SETFL,0);
    if(fd<0){
        printf("Fail to Open %s device\n", path);
        return false;
    }

    //==========配置串口============//
    struct termios opt;           //BASIC SETTING
    tcgetattr(fd, & opt);         //获取终端控制属性


    cfsetospeed(&opt, B921600);   //设置串口输出波特率
    cfsetispeed(&opt, B921600);   //设置串口输入波特率


    /* c_lflag 本地模式 */
    opt.c_lflag &= 0;
    //opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);     //Lord:0; means raw input


    /* c_cflag 控制模式 */
    opt.c_cflag &= ~CSIZE;        //Mask the character size bits
    opt.c_cflag |= CS8;           //set the number of data bits.
    opt.c_cflag &= ~PARENB;       //Set parity to None
    opt.c_cflag &= ~CSTOPB;       //set the number of stop bits to 1
    opt.c_cflag |= CLOCAL | CREAD;
    opt.c_cflag &= ~CRTSCTS;//不使用流控制


    /* c_iflag 输入模式 */
    opt.c_iflag &= ~IGNPAR;       //Lord:IGNPAR;
    opt.c_iflag &= ~(BRKINT | ICRNL | ISTRIP | IXON);


    /* c_oflag 输出模式 */
    //opt.c_oflag &= ~OPOST;      //Lord:0; means raw output
    opt.c_oflag &= 0;


    
    /* c_cc[NCCS] 控制字符 */     //设置停止time -- won't work with NDELAY option in the call to open
    opt.c_cc[VTIME]=0;
    opt.c_cc[VMIN]=0;           //block reading until RX x characers. If x = 0,
                                //it is non-blocking.


    /*TCIFLUSH  刷清输入队列
      TCOFLUSH  刷清输出队列
      TCIOFLUSH 刷清输入、输出队列*/
    tcsetattr(fd, TCSANOW, &opt);
    tcflush(fd,TCIOFLUSH);


    std::cout << "Device ttyUSB0 is set to 921600bps,8N1\n" << std::endl;
    printf("Init IMU Successfully\n");

    return true;
}


void Hardware::run(){
     Read_N100Data();//通过串口读取N100惯导模块的数据
     DataUnpacking();//数据解包
}
