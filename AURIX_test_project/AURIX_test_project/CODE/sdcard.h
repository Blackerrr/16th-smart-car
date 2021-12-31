#ifndef __SD_CARD_H
#define __SD_CARD_H
#include "headfile.h"

#define sd_cse P33_4     // SDCS
#define sd_dao P33_5     //  ‰≥ˆ  MOSI
#define sd_clk P33_6     // CLK
#define sd_dai P33_7     //  ‰»Î MISO

#define SD_CS(n)         gpio_set(sd_cse, n)
#define SD_Dao(n)        gpio_set(sd_dao, n)
#define SD_CLK(n)        gpio_set(sd_clk, n)
#define SD_Dai           gpio_get(sd_dai)

void SDCard_Init(void);
uint16 Write_Single_Block(uint32 BlockAddress);
uint16 Read_Single_Block(uint32 BlockAddress);
uint16 SD_Get_CardID(void);
uint16 SD_Overall_Initiation(void);
uint16 SD_Get_CardInfo(void);
uint16 SD_Initiate_Card();
uint16 SD_Reset_Card();
uint16 SD_CMD_Write(uint16 CMDIndex, unsigned long CMDArg, uint16 ResType, uint16 CSLowRSV);
uint16 SD_Read();
uint16 SD_2Byte_Read();
void SD_Write(uint16 IOData);
void SD_2Byte_Write(uint16 IOData);
#endif
