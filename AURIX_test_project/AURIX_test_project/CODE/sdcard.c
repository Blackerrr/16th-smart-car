#include "sdcard.h"

//unsigned char *SDInfo1 = "SD Init Success.";
//unsigned char *SDInfo2 = "SD Init Fail.";


uint16 ReadBuffer[128];
uint16 WriteBuffer[128];
uint8 rdbuffer[256];

/*SD卡是以块为单位,初始化时有配置,一般为512字节,一个扇区有4096个块*/
uint16 BlockSize;
uint32 BlockNR;

static void SD_card_GPIO_Init(void)
{
    gpio_init(sd_cse, GPO, 0, PUSHPULL);
    gpio_init(sd_clk, GPO, 0, PUSHPULL);
    gpio_init(sd_dao, GPO, 0, OPENDRAIN); // 开漏输出
    gpio_init(sd_dai, GPI, 0, NO_PULL);   // 输入
}

static void Delay5us(void)
{
    systick_delay_us(STM0, 5);
}
//********************************************
void SD_2Byte_Write(uint16 IOData)
{
    unsigned char BitCounter;

    for (BitCounter = 0; BitCounter < 16; BitCounter++)
    {
        SD_CLK(0); //CLK Low

        if (IOData & 0x8000) //If the MSB of IOData is 1, then Do=1, else Do=0.
            SD_Dao(1);      //Do High
        else
            SD_Dao(0); //Do Low

        SD_CLK(1); //CLK High
        Delay5us();

        IOData = IOData << 1; //Because the MSB is transmitted firstly, shift to next lower bit.
    }
}
//********************************************
void SD_Write(uint16 IOData)
{
    unsigned char BitCounter;
    IOData = IOData << 8;

    for (BitCounter = 0; BitCounter < 8; BitCounter++)
    {
        SD_CLK(0); //CLK Low

        if (IOData & 0x8000) //If the MSB of IOData is 1, then Do=1, else Do=0.
            SD_Dao(1);      //Do High
        else
            SD_Dao(0); //Do Low

        SD_CLK(1); //CLK High
        Delay5us();

        IOData = IOData << 1; //Because the MSB is transmitted firstly, shift to next lower bit.
    }
}
//********************************************
uint16 SD_2Byte_Read()
{
    uint16 Buffer;
    unsigned char BitCounter;
    Buffer = 0;

    for (BitCounter = 0; BitCounter < 16; BitCounter++)
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1);           //CLK High
        Buffer = Buffer << 1; //Because the MSB is transmitted firstly, shift to next lower bit.
                              //Because the LSB will be damaged, we can not put this line under next line.
        if (SD_Dai)
            Buffer++; //If SPI_Din=1 then the LSB_of_Buffer=1.
    }

    return Buffer;
}
//********************************************
uint16 SD_Read()
{
    uint16 Buffer;
    unsigned char BitCounter;
    Buffer = 0xffff;

    for (BitCounter = 0; BitCounter < 8; BitCounter++)
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1);           //CLK High
        Buffer = Buffer << 1; //Because the MSB is transmitted firstly, shift to next lower bit.
                              //Because the LSB will be damaged, we can not put this line under next line.
        if (SD_Dai)
            Buffer++; //If SPI_Din=1 then the LSB_of_Buffer=1.
    }

    return Buffer;
}
//********************************************
uint16 SD_CMD_Write(uint16 CMDIndex, uint32 CMDArg, uint16 ResType, uint16 CSLowRSV) //ResType:Response Type, send 1 for R1; send 2 for R1b; send 3 for R2.
{                                                                                                                   //There are 7 steps need to do.(marked by [1]-[7])
    uint16 temp, Response, Response2, CRC, MaximumTimes;
    Response2 = 0;
    MaximumTimes = 10;
    CRC = 0x0095; //0x0095 is only valid for CMD0
    if (CMDIndex != 0)
        CRC = 0x00ff;

    SD_CS(0); //[1] CS Low

    SD_2Byte_Write(((CMDIndex | 0x0040) << 8) + (CMDArg >> 24)); //[2] Transmit Command_Index & 1st Byte of Command_Argument.
    SD_2Byte_Write((CMDArg & 0x00ffff00) >> 8);                  //[2] 2nd & 3rd Byte of Command_Argument
    SD_2Byte_Write(((CMDArg & 0x000000ff) << 8) + CRC);          //[2] 4th Byte of Command_Argument & CRC only for CMD0

    SD_Dao(1); //[3] Do High
                //[3] Restore Do to High Level

    for (temp = 0; temp < 8; temp++) //[4] Provide 8 extra clock after CMD
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    switch (ResType) //[5] wait response
    {
    case 1: //R1
    {
        do
            Response = SD_Read();
        while (Response == 0xffff);
        break;
    }
    case 2: //R1b
    {
        do
            Response = SD_Read();
        while (Response == 0xffff); //Read R1 firstly

        do
            Response2 = SD_Read() - 0xff00;
        while (Response2 != 0); //Wait until the Busy_Signal_Token is non-zero
        break;
    }
    case 3:
        Response = SD_2Byte_Read();
        break; //R2
    }

    if (CSLowRSV == 0)
        SD_CS(1); //[6] CS High (if the CMD has data block response CS should be kept low)

    for (temp = 0; temp < 8; temp++) //[7] Provide 8 extra clock after card response
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }
    return Response;
}
//********************************************
uint16 SD_Reset_Card()
{
    uint16 temp, MaximumTimes;
    MaximumTimes = 10;

    /* sd卡上电后，必须先向sd卡发送74个时钟周期延时，以完成sd卡上电过程 */
    for (temp = 0; temp < 80; temp++) //Send 74+ Clocks
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    /*sd卡上电后会自动进入sd卡总线模式*/
    /*在sd卡总线模式下向sd卡发送复位信号CMD0， 若此时片选信为低，则sd卡进入spi总线模式*/
    return SD_CMD_Write(0x0000, 0x00000000, 1, 0); //Send CMD0
}
//********************************************
uint16 SD_Initiate_Card() //Polling the card after reset
{
    uint16 temp, Response, MaximumTimes;
    MaximumTimes = 50;

    for (temp = 0; temp < MaximumTimes; temp++)
    {
        Response = SD_CMD_Write(0x0037, 0x00000000, 1, 0); //Send CMD55
        Response = SD_CMD_Write(0x0029, 0x00000000, 1, 0); //Send ACMD41
        if (Response == 0xff00)
            temp = MaximumTimes;
    }

    return Response;
}
//********************************************
uint16 SD_Get_CardInfo(void) //Read CSD register
{
    uint16 temp, Response, MaximumTimes;
    MaximumTimes = 50;

    for (temp = 0; temp < MaximumTimes; temp++)
    {
        Response = SD_CMD_Write(9, 0x00000000, 1, 1); //Send CMD9
        if (Response == 0xff00)
            temp = MaximumTimes;
    }

    for (temp = 0; temp < 8; temp++) //Provide 8 clock to romove the first byte of data response (0x00fe)
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    for (temp = 0; temp < 8; temp++)
        ReadBuffer[temp] = SD_2Byte_Read(); //Get the CSD data

    for (temp = 0; temp < 16; temp++) //Provide 16 clock to remove the last 2 bytes of data response (CRC)
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    SD_CS(1); //CS_High()

    for (temp = 0; temp < 8; temp++) //Provide 8 extra clock after data response
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    BlockNR = ((ReadBuffer[3] << 2) & 0x0fff) + ((ReadBuffer[4] >> 14) & 0x0003) + 1;                         //Calcuate MULT
    BlockNR = BlockNR * (0x0002 << (((ReadBuffer[4] << 1) & 0x0007) + ((ReadBuffer[5] >> 15) & 0x0001) + 1)); //Calcuate Block_Number
    return Response;
}
//********************************************
uint16 SD_Overall_Initiation(void)
{
    uint16 Response, Response_2;
    Response = 0x0000;
    Response_2 = 0xff00;

    SD_Dao(1);  //[1] Do High
                //[1] Do must be High when there is no transmition
    do
        Response = SD_Reset_Card(); //[2] Send CMD0
    while (Response != 0xff01);

    if (Response != 0xff01)
        Response_2 += 8;

//    Response=SD_CMD_Write(8,0x00000000,1,0);//Send CMD8

    Response = SD_Initiate_Card(); //[3] Send CMD55+ACMD41
    if (Response == 0xff00)
        ;
    else
    {
        Response_2 += 4;
        ;
    }

    do
        Response = SD_Get_CardInfo(); //[4] Read CSD
    while (Response != 0xff00);
    if (Response == 0xff01)
        Response_2 += 2;

    return Response_2;
    //  0000|0000||0000|0000 Response_2
    //                  |||_CSD Fail
    //                  ||__CMD55+ACMD41 Fail
    //                  |___CMD0 Fail
}
//********************************************
uint16 SD_Get_CardID(void) //Read CID register
{
    uint16 temp, Response, MaximumTimes;
    MaximumTimes = 10;

    for (temp = 0; temp < MaximumTimes; temp++)
    {
        Response = SD_CMD_Write(10, 0x00000000, 1, 1); //Send CMD9
        if (Response == 0xff00)
            temp = MaximumTimes;
    }

    for (temp = 0; temp < 8; temp++) //Provide 8 clock to romove the first byte of data response (0x00fe)
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    for (temp = 0; temp < 8; temp++)
        ReadBuffer[temp] = SD_2Byte_Read(); //Get the CID data

    for (temp = 0; temp < 16; temp++) //Provide 16 clock to remove the last 2 bytes of data response (CRC)
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    SD_CS(1); //CS_High()

    for (temp = 0; temp < 8; temp++) //Provide 8 extra clock after data response
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    return Response;
}
//********************************************
uint16 Read_Single_Block(uint32 BlockAddress)
{
    uint16 temp, Response, MaximumTimes;
    MaximumTimes = 10;

    if (BlockAddress > BlockNR)
        return 0xff20; //whether BlockAddress out of range?

    for (temp = 0; temp < MaximumTimes; temp++)
    {
        Response = SD_CMD_Write(17, BlockAddress, 1, 1); //Send CMD17
        if (Response == 0xff00)
            temp = MaximumTimes;
    }

    while (SD_Read() != 0xfffe)
    {
        ;
    }
    //这里为了使只有512byte的单片机能够读写SD卡，特意节省了RAM的使用量，每次读写只有两个重复的128byte
    //如果您使用的单片机拥有1K以上的RAM请将"%128"去掉
    for (temp = 0; temp < 256; temp++)
        ReadBuffer[temp % 128] = SD_2Byte_Read(); //Get the readed data

    for (temp = 0; temp < 16; temp++) //Provide 16 clock to remove the last 2 bytes of data response (CRC)
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    SD_CS(1); //CS_High()

    for (temp = 0; temp < 8; temp++) //Provide 8 extra clock after data response
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    return Response;
}
//********************************************
uint16 Write_Single_Block(uint32 BlockAddress)
{
    uint16 temp, Response, MaximumTimes;
    MaximumTimes = 10;

    if (BlockAddress > BlockNR)
        return 0xff20; //whether BlockAddress out of range?

    for (temp = 0; temp < MaximumTimes; temp++)
    {
        Response = SD_CMD_Write(24, BlockAddress, 1, 1); //Send CMD24
        if (Response == 0xff00)     /*接收到sd卡的响应信号（00）*/
            temp = MaximumTimes;
    }

    for (temp = 0; temp < 8; temp++) //Provide 8 extra clock after CMD response
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    SD_Write(0x00fe); //Send Start Block Token    /*发送数据起始标志*/
    //这里为了使只有512byte的单片机能够读写SD卡，特意节省了RAM的使用量，每次读写只有两个重复的128byte
    //如果您使用的单片机拥有1K以上的RAM请将"%128"去掉
    for (temp = 0; temp < 256; temp++)
        SD_2Byte_Write(WriteBuffer[temp % 128]); //Data Block
    SD_2Byte_Write(0xffff);                      //Send 2 Bytes CRC

    Response = SD_Read();
    while (SD_Read() != 0xffff)
    {
        ;
    }

    SD_CS(1); //CS_High()

    for (temp = 0; temp < 8; temp++) //Provide 8 extra clock after data response
    {
        SD_CLK(0); //CLK Low
        Delay5us();
        SD_CLK(1); //CLK High
        Delay5us();
    }

    return Response;
}

void SDCard_Init(void)
{
    uint16 M_Response, data;
    for(uint8 i = 0; i < 128; i++)
    {
        if (i < 64)
        {
        WriteBuffer[i]=0x4141;
        }
        else
        {
        WriteBuffer[i]=0x4242;
        }
    }
    SD_card_GPIO_Init();
    M_Response=0x0000;
    M_Response=SD_Overall_Initiation();
    M_Response=SD_CMD_Write(16,512,1,0);
    data=SD_Get_CardID();
    Write_Single_Block(0x0000);
    Read_Single_Block(0x0000);
    for(uint16 i = 0; i < 256; i++)
    {
        rdbuffer[i * 2]     = ReadBuffer[i] >> 8;
        rdbuffer[i * 2 + 1] = ReadBuffer[i] && 0x00ff;
    }
    seekfree_wireless_send_buff(rdbuffer, 256);

}
