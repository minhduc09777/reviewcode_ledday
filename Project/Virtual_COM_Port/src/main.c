
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "math.h"
#include <stdio.h>
#include "stm32f10x_spi.h"
#include "stm32f10x_dma.h"
 #include "stm32f10x_flash.h"
 
#include "usb_prop.h"
#include <stdlib.h>
#define Setbit(ADDRESS,BIT) (ADDRESS |= (1<<BIT)) 
#define Clearbit(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT)) 
#define Checkbit(ADDRESS,BIT) (ADDRESS & (1<<BIT))
	
#define LEDUSB_1    GPIOB->BSRR = GPIO_Pin_7///
#define LEDUSB_0    GPIOB->BRR = GPIO_Pin_7////
 
#define LED1_1    GPIOB->BSRR = GPIO_Pin_8////
#define LED1_0    GPIOB->BRR = GPIO_Pin_8////

#define LED2_0    GPIOB->BSRR = GPIO_Pin_9
#define LED2_1    GPIOB->BRR = GPIO_Pin_9

#define LED3_0    GPIOC->BSRR = GPIO_Pin_13
#define LED3_1    GPIOC->BRR = GPIO_Pin_13

#define LED4_0    GPIOC->BSRR = GPIO_Pin_14
#define LED4_1    GPIOC->BRR = GPIO_Pin_14

#define LED5_0    GPIOC->BSRR = GPIO_Pin_15
#define LED5_1    GPIOC->BRR = GPIO_Pin_15

#define LEDERROR_1    GPIOB->BSRR = GPIO_Pin_0///
#define LEDERROR_0    GPIOB->BRR = GPIO_Pin_0///

#define NUT_LEN GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)
#define NUT_XUONG GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)
#define NUT_TEST GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)



#define Dummy_Byte                    0xFF                            //???

__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08007C00)   /* Start @ of user Flash area */
uint32_t  Address = 0x00;

#define Flash_CS_High    GPIO_SetBits(GPIOA, GPIO_Pin_15);
#define Flash_CS_Low    GPIO_ResetBits(GPIOA, GPIO_Pin_15);


#define MOSI_1   GPIOA->BSRR = GPIO_Pin_0
#define MOSI_0     GPIOA->BRR = GPIO_Pin_0


#define SCLK_1    GPIOA->BSRR = GPIO_Pin_1
#define SCLK_0     GPIOA->BRR = GPIO_Pin_1
//extern unsigned char USB_buff [64];



const uint8_t gamma8[] ={
0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

const uint8_t pass[]= {
16, 242, 89, 109, 228, 32, 143, 252, 34, 6, 163, 158, 148, 138, 14, 14, 165, 39, 163, 204, 248, 74, 41, 14, 37, 9, 81, 98, 228, 91, 160, 84, 115, 17, 79, 165, 90, 176, 120, 10, 127, 147, 136, 64, 217, 198, 217, 155, 50, 101, 117, 117, 218, 112, 234, 141, 201, 75, 72, 218, 207, 45, 24, 198, 101, 79, 43, 130, 58, 104, 179, 244, 174, 152, 76, 101, 193, 232, 52, 94, 48, 181, 47, 126, 230, 42, 121, 3, 39, 193, 220, 15, 103, 158, 59, 82, 123, 194, 218, 137, 155, 86, 96, 201, 177, 128, 199, 65, 36, 133, 203, 224, 22, 233, 170, 25, 253, 154, 224, 58, 213, 39, 91, 120, 140, 164, 140, 16, 93, 250, 234, 255, 14, 171, 195, 217, 85, 102, 204, 102, 99, 56, 223, 161, 246, 251, 248, 126, 243, 34, 84, 224, 225, 159, 179, 115, 251, 231, 61, 12, 243, 182, 227, 41, 155, 204, 210, 107, 38, 208, 195, 151, 206, 122, 215, 157, 72, 185, 130, 144, 172, 14, 28, 59, 165, 9, 29, 110, 247, 79, 222, 110, 41, 191, 114, 172, 84, 181, 6, 41, 41, 140, 88, 34, 94, 189, 18, 103, 200, 22, 42, 65, 100, 172, 96, 229, 153, 168, 132, 145, 174, 99, 116, 214, 242, 85, 110, 14, 7, 42, 72, 147, 179, 89, 103, 32, 182, 250, 220, 232, 246, 182, 166, 224, 36, 156, 10, 124, 95, 141, 18, 171, 49, 116, 123, 197, 23, 129, 48, 9, 78, 3, 95, 158, 205, 151, 142, 11, 69, 63, 46, 159, 204, 155, 154, 247, 189, 147, 177, 85, 75, 241, 175, 122, 23, 156, 98, 62, 221, 161, 8, 52, 202, 211, 153, 243, 86, 7, 18, 141, 14, 254, 55, 31, 94, 115, 223, 149, 217, 131, 8, 131, 208, 218, 190, 93, 84, 228, 134, 49, 53, 80, 45, 163, 55, 249, 213, 249, 2, 167, 160, 181, 129, 36, 71, 76, 186, 144, 28, 164, 189, 204, 100, 90, 153, 132, 100, 239, 20, 60, 159, 114, 129, 225, 87, 189, 209, 147, 231, 100, 157, 230, 147, 50, 226, 82, 2, 171, 146, 114, 163, 195, 200, 226, 115, 104, 235, 211, 10, 179, 149, 229, 228, 197, 8, 46, 52, 159, 204, 137, 122, 38, 168, 183, 6, 214, 57, 50, 119, 70, 130, 185, 93, 162, 152, 219, 169, 158, 109, 239, 210, 253, 137, 52, 207, 183, 2, 205, 41, 179, 30, 98, 223, 9, 247, 125, 26, 16, 219, 156, 47, 185, 92, 196, 48, 219, 136, 65, 44, 45, 133, 149, 50, 221, 183, 124, 156, 116, 232, 78, 212, 107, 8, 195, 39, 31, 217, 84, 171, 27, 192, 152, 195, 209, 192, 24, 161, 197, 3, 243, 47, 192, 160, 252, 46, 136, 48, 2, 81, 122, 224, 166, 40, 140, 200, 195, 176, 152, 156, 17, 2, 52, 149, 17, 108, 236, 210, 97, 28, 158, 218, 215, 112, 244, 31, 19, 203, 12, 148, 158, 238, 215, 2, 48, 59, 26, 112, 54, 8, 252, 103, 9, 40, 241, 241, 251, 42, 143, 143, 66, 182, 207, 229, 178, 159, 9, 53, 51, 108, 180, 247, 164, 3, 253, 34, 42, 49, 101, 213, 82, 123, 155, 88, 32, 54, 209, 174, 126, 3, 36, 233, 60, 124, 82, 232, 31, 242, 79, 145, 50, 228, 60, 201, 84, 5, 195, 5, 43, 207, 198, 201, 197, 185, 61, 198, 27, 119, 197, 123, 206, 90, 182, 48, 72, 202, 187, 39, 177, 21, 3, 56, 225, 211, 163, 109, 63, 209, 3, 27, 115, 203, 131, 175, 60, 90, 35, 130, 62, 139, 204, 168, 45, 212, 195, 215, 46, 11, 129, 138, 73, 155, 83, 21, 203, 142, 231, 242, 22, 208, 135, 73, 116, 170, 8, 2, 215, 6, 244, 237, 167, 57, 233, 38, 72, 91, 11, 13, 16, 203, 62, 51, 79, 121, 209, 215, 119, 109, 227, 173, 115, 49, 108, 110, 4, 130, 52, 131, 203, 192, 0, 39, 23, 147, 226, 16, 178, 45, 205, 112, 131, 217, 225, 74, 145, 83, 250, 251, 216, 49, 138, 106, 35, 70, 122, 118, 205, 123, 33, 197, 39, 136, 66, 79, 203, 23, 28, 188, 150, 193, 36, 73, 159, 114, 42, 153, 79, 33, 220, 176, 136, 57, 170, 154, 54, 149, 3, 208, 25, 108, 66, 55, 12, 7, 73, 251, 151, 145, 126, 122, 55, 61, 100, 22, 12, 64, 202, 176, 28, 224, 38, 171, 158, 113, 60, 237, 221, 167, 24, 53, 19, 76, 162, 41, 126, 236, 61, 151, 40, 47, 1, 189, 163, 97, 121, 75, 212, 148, 141, 246, 203, 83, 180, 56, 70, 151, 152, 154, 203, 91, 30, 239, 120, 72, 102, 235, 154, 58, 151, 32, 2, 50, 135, 237, 223, 104, 8, 60, 248, 241, 25, 72, 26, 34, 106, 192, 151, 106, 227, 231, 83, 163, 204, 205, 16, 17, 69, 90, 251, 142, 176, 153, 252, 109, 243, 153, 203, 198, 89, 222, 143, 92, 162, 218, 66, 213, 213, 86, 222, 166, 83, 48, 87, 176, 204, 95, 101, 186, 220, 205, 35, 173, 65, 106, 64, 127, 76, 173, 46, 209, 244, 209, 17, 5, 87, 247, 0, 242, 138, 59, 60, 238, 123, 84, 58,
            128, 66, 76, 161, 148, 51, 17, 234, 139, 186, 225, 26, 56, 153, 194, 137, 40, 39, 13, 178, 129, 30, 81, 88, 212, 94, 115, 47, 160, 145, 52, 49, 237, 47, 192, 60, 0, 11, 153, 157, 227, 230, 133, 74, 118, 221, 200, 89, 199, 178, 19, 199, 110, 162, 185, 97, 241, 243, 204, 54, 192, 225, 73, 250, 134, 175, 44, 9, 216, 133, 137, 28, 141, 111, 207, 155, 153, 6, 226, 246, 150, 25, 104, 238, 125, 109, 195, 75, 118, 95, 75, 12, 62, 99, 221, 1, 157, 138, 196, 198, 176, 190, 128, 66, 41, 247, 57, 255, 210, 29, 200, 234, 17, 214, 160, 166, 121, 90, 125, 25, 236, 225, 147, 121, 58, 124, 221, 42, 146, 205 };
	
const uint32_t mau1903[256]={0,8388608,32768,8421376,128,8388736,32896,8421504,12632256,16711680,65280,16776960,255,16711935,65535,16777215,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,51,102,153,204,255,13056,13107,13158,13209,13260,13311,26112,26163,26214,26265,26316,26367,39168,39219,39270,39321,39372,39423,52224,52275,52326,52377,52428,52479,65280,65331,65382,65433,65484,65535,3342336,3342387,3342438,3342489,3342540,3342591,3355392,3355443,3355494,3355545,3355596,3355647,3368448,3368499,3368550,3368601,3368652,3368703,3381504,3381555,3381606,3381657,3381708,3381759,3394560,3394611,3394662,3394713,3394764,3394815,3407616,3407667,3407718,3407769,3407820,3407871,6684672,6684723,6684774,6684825,6684876,6684927,6697728,6697779,6697830,6697881,6697932,6697983,6710784,6710835,6710886,6710937,6710988,6711039,6723840,6723891,6723942,6723993,6724044,6724095,6736896,6736947,6736998,6737049,6737100,6737151,6749952,6750003,6750054,6750105,6750156,6750207,10027008,10027059,10027110,10027161,10027212,10027263,10040064,10040115,10040166,10040217,10040268,10040319,10053120,10053171,10053222,10053273,10053324,10053375,10066176,10066227,10066278,10066329,10066380,10066431,10079232,10079283,10079334,10079385,10079436,10079487,10092288,10092339,10092390,10092441,10092492,10092543,13369344,13369395,13369446,13369497,13369548,13369599,13382400,13382451,13382502,13382553,13382604,13382655,13395456,13395507,13395558,13395609,13395660,13395711,13408512,13408563,13408614,13408665,13408716,13408767,13421568,13421619,13421670,13421721,13421772,13421823,13434624,13434675,13434726,13434777,13434828,13434879,16711680,16711731,16711782,16711833,16711884,16711935,16724736,16724787,16724838,16724889,16724940,16724991,16737792,16737843,16737894,16737945,16737996,16738047,16750848,16750899,16750950,16751001,16751052,16751103,16763904,16763955,16764006,16764057,16764108,16764159,16776960,16777011,16777062,16777113,16777164,16777215};
const uint16_t mau6803[256]={0,16384,512,16896,16,16400,528,16912,25368,31744,992,32736,31,31775,1023,32767,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,12,19,25,31,192,198,204,211,217,223,384,390,396,403,409,415,608,614,620,627,633,639,800,806,812,819,825,831,992,998,1004,1011,1017,1023,6144,6150,6156,6163,6169,6175,6336,6342,6348,6355,6361,6367,6528,6534,6540,6547,6553,6559,6752,6758,6764,6771,6777,6783,6944,6950,6956,6963,6969,6975,7136,7142,7148,7155,7161,7167,12288,12294,12300,12307,12313,12319,12480,12486,12492,12499,12505,12511,12672,12678,12684,12691,12697,12703,12896,12902,12908,12915,12921,12927,13088,13094,13100,13107,13113,13119,13280,13286,13292,13299,13305,13311,19456,19462,19468,19475,19481,19487,19648,19654,19660,19667,19673,19679,19840,19846,19852,19859,19865,19871,20064,20070,20076,20083,20089,20095,20256,20262,20268,20275,20281,20287,20448,20454,20460,20467,20473,20479,25600,25606,25612,25619,25625,25631,25792,25798,25804,25811,25817,25823,25984,25990,25996,26003,26009,26015,26208,26214,26220,26227,26233,26239,26400,26406,26412,26419,26425,26431,26592,26598,26604,26611,26617,26623,31744,31750,31756,31763,31769,31775,31936,31942,31948,31955,31961,31967,32128,32134,32140,32147,32153,32159,32352,32358,32364,32371,32377,32383,32544,32550,32556,32563,32569,32575,32736,32742,32748,32755,32761,32767};

static uint8_t fac_us=0;

uint8_t data1[3072];
uint8_t data2[3072];	
uint8_t mahoa[8];
uint8_t dangnap=0;
uint8_t baonap=0;
int dem_play=0;
int tong_play=0;
int den_trungbinh=0;
uint8_t loaiden=0;
uint8_t vantoc=1;
uint8_t dosang=1;
int vtport1=0;
int tongport1=0;
int vtport2=0;
int tongport2=0;
uint8_t loaiday=0;
extern char TT_XOA;
volatile uint32_t counter = 0;

void RCC_Configuration1(void);
void NVIC_Configuration(void);
void Delay(__IO uint32_t nCount);

GPIO_InitTypeDef GPIO_InitStruct;

EXTI_InitTypeDef   EXTI_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;
SPI_InitTypeDef SPI_InitStructure;
 
void delay_ms(uint32_t time) // why use this function
{
  volatile uint32_t i;
	while (time-- > 0)
	{
		i = 50;
		while (i-- > 0)
		{
			
		}
	}
}
void RCC_Configuration1(void)
{   
__IO uint32_t  HSEStatus = 0;		
  /* RCC system reset(for debug purpose) */

  RCC_DeInit();

 /* Enable HSE */

  RCC_HSEConfig(RCC_HSE_ON);

 /* Wait till HSE is ready */

  HSEStatus = RCC_WaitForHSEStartUp();

 if(HSEStatus == SUCCESS)

  {

    /* Enable Prefetch Buffer */

    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

   /* Flash 2 wait state */

    FLASH_SetLatency(FLASH_Latency_2);

  /* HCLK = SYSCLK */

    RCC_HCLKConfig(RCC_SYSCLK_Div1); 

     /* PCLK2 = HCLK */

    RCC_PCLK2Config(RCC_HCLK_Div1); 

  /* PCLK1 = HCLK/2 */

    RCC_PCLK1Config(RCC_HCLK_Div2);

 /* PLLCLK = 8MHz * 9 = 72 MHz */

    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);

  /* Enable PLL */ 

    RCC_PLLCmd(ENABLE);

  /* Wait till PLL is ready */

    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)

    {

    }

   /* Select PLL as system clock source */

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

 /* Wait till PLL is used as system clock source */

    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }

  }

}

static void stm32_dma_transfer(char receive,	const unsigned char *buff, u16 cc){
	DMA_InitTypeDef DMA_InitStructure;


	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* shared DMA configuration values */
  DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned long)(&(SPI1->DR));
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_BufferSize = cc;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_DeInit(DMA1_Channel2);
	DMA_DeInit(DMA1_Channel3);

	
		/* DMA1 channel2 configuration SPI1 RX ---------------------------------------------*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned long)buff;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_Init(DMA1_Channel2, &DMA_InitStructure);

		/* DMA1 channel3 configuration SPI1 TX ---------------------------------------------*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned long)0xFF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
		DMA_Init(DMA1_Channel3, &DMA_InitStructure);



	/* Enable DMA1 Channel2 */
	DMA_Cmd(DMA1_Channel2, ENABLE);
	/* Enable DMA1 Channel3 */
	DMA_Cmd(DMA1_Channel3, ENABLE);

	/* Enable SPI1 TX/RX request */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	/* Wait until DMA1_Channel 3 Transfer Complete */
	// not needed: while (DMA_GetFlagStatus(DMA1_FLAG_TC3) == RESET) { ; }
	/* Wait until DMA1_Channel 2 Receive Complete */
	while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET) { ; }
	// same w/o function-all:
	// while ( ( ( DMA1->ISR ) & DMA1_FLAG_TC2 ) == RESET ) { ; }

	/* Disable DMA1 Channel2 */
	DMA_Cmd(DMA1_Channel2, DISABLE);
	/* Disable DMA1 Channel3 */
	DMA_Cmd(DMA1_Channel3, DISABLE);

	/* Disable SPI1 RX/TX request */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
}


uint8_t Flash_SendByte(uint8_t byte)
{
  
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  /*??8??? */
  SPI_I2S_SendData(SPI1, byte);
 /* ????8??? */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  /* ?????8??? */
  return SPI_I2S_ReceiveData(SPI1);
}
void Flash_Write_Enable(void)
{
	Flash_CS_Low;
	Flash_SendByte(0x06);
	Flash_CS_High;
}
void Flash_WaitWriteEnd(void){
	
uint8_t FLASH_Status = 0;
   /* ???? */
Flash_CS_Low;
  /*??????? */
Flash_SendByte(0x05);
  /*?????????FLASH????*/
do
{
	/* ????? */
	FLASH_Status = Flash_SendByte(0xFF);
}
while ((FLASH_Status & 0x01) == SET); /* ??????*/
  /*????*/
 Flash_CS_High;

}



void Flash_Read(uint8_t *pBuffer,uint32_t ReadAddress,uint16_t ReadByteNum)
{
	// address 32 bit
	Flash_CS_Low;
	Flash_SendByte(0x03);
	Flash_SendByte(ReadAddress >> 16);
	Flash_SendByte(ReadAddress >> 8);
	Flash_SendByte(ReadAddress);
	 /*
	while(ReadByteNum--)
	{
	*pBuffer = Flash_SendByte(0xFF);
	pBuffer++;
	*/
	stm32_dma_transfer(1,pBuffer,ReadByteNum);
	Flash_CS_High;
}

void Flash_PageWrite(uint8_t *pBuffer,uint32_t PageAddress,uint16_t WriteByteNum)
{
	Flash_Write_Enable();
	Flash_CS_Low;
	Flash_SendByte(0x02);
	Flash_SendByte(PageAddress >> 16);
	Flash_SendByte(PageAddress >> 8);
	Flash_SendByte(PageAddress);
		
	if(WriteByteNum > 256)  //the biggest num is 256
	{
	 WriteByteNum = 256;
	}
	 
	while(WriteByteNum--)
	{
	  Flash_SendByte(*pBuffer);
	  pBuffer++;
	}
	 
	Flash_CS_High;
	Flash_WaitWriteEnd();
}

void Flash2048(uint8_t *pBuffer,uint32_t PageAddress) // Flash2048 8 page, 1 page 256 byte
{
	int x;
	for ( x = 0; x < 8; x++)
	{
		
    Flash_PageWrite(pBuffer,PageAddress,256);
		pBuffer=pBuffer+256;
	  PageAddress=PageAddress+256;
	}
} 

void Flash_EraseSector(uint32_t SectorAddress)
{
	Flash_Write_Enable();
	//Flash_WaitWriteEnd();
	Flash_CS_Low;
	Flash_SendByte(0xD8);
	Flash_SendByte((SectorAddress & 0xFF0000) >> 16);
	Flash_SendByte((SectorAddress & 0xFF00) >> 8);
	Flash_SendByte(SectorAddress & 0xFF);
	Flash_CS_High;
	Flash_WaitWriteEnd();
}

void xoachip(void)
{
  /* πƒ‹–¥»Î*/
 Flash_Write_Enable();
 Flash_CS_Low;
 Flash_SendByte(0xC7);

 Flash_CS_High;
 Flash_WaitWriteEnd();
}





void delay_init(void)
{
  //	SysTick->CTRL&=0xfffffffb;//bit2??,??????  HCLK/8
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//??????  HCLK/8
	fac_us=48/8;		    
	//fac_ms=(uint16_t)fac_us*1000;
}		

void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //????	  		 
	SysTick->VAL=0x00;        //?????
	SysTick->CTRL=0x01 ;      //???? 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//??????   
	SysTick->CTRL=0x00;       //?????
	SysTick->VAL =0X00;       //?????	 
}

 
void RCC_Configuration_HSI_48Mhz_with_USBclock(void){
	
 ErrorStatus HSIStartUpStatus; 
 RCC_DeInit();
 RCC_HSICmd(ENABLE);
 while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
 FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
 FLASH_SetLatency(FLASH_Latency_2);
 RCC_HCLKConfig(RCC_SYSCLK_Div1);
 RCC_PCLK1Config(RCC_HCLK_Div2);
 RCC_PCLK2Config(RCC_HCLK_Div1);
 RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);

 RCC_PLLCmd(ENABLE);
 while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
 RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
 while(RCC_GetSYSCLKSource() != 0x08); 
	
}
/////////////////////////////////////// 
uint8_t get_flash(int vt)
{
 
	uint32_t ff;
	uint32_t s1=vt/4;
	uint32_t s2=vt%4;
	uint8_t tt[4];
  Address = FLASH_USER_START_ADDR +s1*4;
	 
  ff = *(__IO uint32_t *)Address;
 
          
	tt[0]=(uint8_t)((ff>>24) & 0xFF);
	tt[1]=(uint8_t)((ff>>16) & 0xFF);
	tt[2]=(uint8_t)((ff>>8) & 0xFF);
	tt[3]=(uint8_t)((ff) & 0xFF);
	
	if(s2==0) return tt[0];
	else if(s2==1) return tt[1];
	else if(s2==2) return tt[2];
	else if(s2==3) return tt[3];
	 
	return 0;
	
}
void luu_flash(void)  // save flash in stm
{
	uint32_t ik,ij;
	uint32_t ff;
	uint8_t tt[4];
	uint32_t	paper=0;
	Address = FLASH_USER_START_ADDR;

  data1[0]=vantoc;
  data1[1]=dosang;
  data1[20]=mahoa[0];
	data1[21]=mahoa[1];
	data1[22]=mahoa[2];
	data1[23]=mahoa[3];
	data1[24]=mahoa[4];
	data1[25]=mahoa[5];
	data1[26]=mahoa[6];
	data1[27]=mahoa[7];
	 // datam[2]=maxxbass;
	//FLASH_Unlock();
  FLASH_UnlockBank1();

 FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
if (FLASH_ErasePage(FLASH_USER_START_ADDR+paper*FLASH_PAGE_SIZE )!= FLASH_COMPLETE)
	{
	
		while (1)
		{
		}
	}
  Address = FLASH_USER_START_ADDR+paper*FLASH_PAGE_SIZE ;
	for(ik=0;ik<256;ik++)
    {
	 
      ff=data1[ik*4]*16777216 +data1[ik*4+1]*65536+data1[ik*4+2]*256+data1[ik*4+3];
			FLASH_ProgramWord(Address,ff) ;
      Address = Address + 4; 
		}
	
	
  FLASH_LockBank1();

	
}
 
 //#define M_1_1    GPIOA->BSRR = (0x03 << 16) | 0x03 ;
 //#define M_0_0    GPIOA->BSRR = (0x03 << 16) | 0x00 ;
// #define M_0_1    GPIOA->BSRR = (0x03 << 16) | 0x02 ;
 //#define M_1_0    GPIOA->BSRR = (0x03 << 16) | 0x01 ;
  #define M_1_1    GPIOA->ODR=0x03;
  #define M_0_0    GPIOA->ODR=0x00;
  #define M_0_1    GPIOA->ODR=0x02;
  #define M_1_0    GPIOA->ODR=0x01;

void send1903m8(int count,uint8_t bit)
{
	int k,h1=0,h2=6144;
	uint8_t i,l,d=0;
	uint32_t mask=0,da1,da2,colo1,colo2;
	uint8_t d1_r,d1_g,d1_b;
	uint8_t d2_r,d2_g,d2_b;
	
	for (k = 0; k < count; k++) 
	{
		 
	 colo1=mau1903[data1[k]];
	 colo2=mau1903[data2[k]];
	 d1_r=((uint8_t)(colo1>>16))/bit;
	 d1_g=((uint8_t)(colo1>>8))/bit;
	 d1_b=((uint8_t)(colo1))/bit;
	 da1=d1_r<<16 | d1_g<<8 | d1_b;
	 d2_r=((uint8_t)(colo2>>16))/bit;
	 d2_g=((uint8_t)(colo2>>8))/bit;
	 d2_b=((uint8_t)(colo2))/bit;
	 da2=d2_r<<16 | d2_g<<8 | d2_b;
		
	 mask=0x800000;
	 for (i = 0; i <24; i++) 
	 {
		 if((mask & da1) && (mask & da2))
		 {
				// The three PWM output ports, OUTR, OUTG and OUTB, send signals in a 4-ms period, with different duty cycles corresponding to the 24-bit data received
				// The input signal from the DIN pin is a RESET signal, the UCS1903 will send the received data for display
			  // Check timing signal to detail
			  M_1_1;M_1_1;M_1_1; 
			  M_1_1;M_1_1;M_1_1;
			  M_1_1;M_1_1;M_1_1;
			  M_1_1;M_1_1;M_1_1;
			  M_1_1; M_1_1;M_1_1;
				M_0_0;M_0_0;M_0_0; M_0_0; 				 
		 } 
		 if((mask & da1) && !(mask & da2))
		 {
				M_1_1;M_1_1;M_1_1;
			  M_1_1;M_1_0;M_1_0;
			  M_1_0;M_1_0;M_1_0;
			  M_1_0;M_1_0;M_1_0;
			  M_1_0; M_1_0;M_1_0; 
				M_0_0;M_0_0;M_0_0;
			  M_0_0;
		 } if(!(mask & da1) && (mask & da2))
		 {
				M_1_1;M_1_1;M_1_1;
			  M_1_1;M_0_1;M_0_1;
			  M_0_1;M_0_1;M_0_1;
			  M_0_1;M_0_1;M_0_1;
			  M_0_1; M_0_1;M_0_1;
				M_0_0;M_0_0;M_0_0;
			  M_0_0; 
		 }
			if(!(mask & da1) && !(mask & da2))
		 {
				M_1_1;M_1_1;M_1_1;M_1_1; 
				M_0_0;M_0_0;M_0_0;M_0_0;
			  M_0_0;M_0_0;M_0_0;M_0_0;
			  M_0_0;M_0_0;M_0_0;M_0_0;
			  M_0_0;M_0_0; M_0_0; 
		 }
			mask>>=1;
		} 
			
	}
 
}

void send1903m8_1914(int count,uint8_t bit) // What is 1914 --> that is IC or what? // bit is light?
{
	int k,h1=0,h2=6144;
	uint8_t i,l,d=0;
	uint32_t mask=0,da1,da2,colo1,colo2;
	uint8_t d1_r,d1_g,d1_b;
	uint8_t d2_r,d2_g,d2_b;
  uint8_t pa[6]={0xFF,0xFF,0xFF,0x00,0x00,0x00};
	uint32_t cc[2]={0xFFFFFF,0x000000};
 	
 
 
  uint32_t ca=0x808080;
  
	
 
 /*
  for (k = 0; k < 6; k++) 
{
 
	  for (i = 0; i <8; i++) 
	{
          if(Checkbit(pa[k],i))
					{
						  
M_0_0;M_0_0;M_0_0;M_0_0;
M_0_0;M_0_0;M_0_0;M_0_0;						
M_1_1;M_1_1;M_1_1;M_1_1;
M_1_1;M_1_1;M_1_1;M_1_1;		
M_1_1;M_1_1;M_1_1;M_1_1;		
M_1_1;M_1_1; 							
					}else 
					{
M_0_0;M_0_0;M_0_0;M_0_0;
M_0_0;M_0_0;M_0_0;M_0_0;						
M_0_0;M_0_0;M_0_0;M_0_0;
						M_0_0;M_0_0;
M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;		
M_0_0;M_1_1;M_1_1;M_1_1;		
        
					}

 	     
	} 
	    
}
 */
 
 
	 for (k = 0; k < 2; k++) 
	 {
			mask=0x800000;
			for (i = 0; i <24; i++) 
		{
		if((mask & cc[k]) )
		{
				M_1_1;M_1_1;M_1_1;M_1_1; M_1_1;
				
			
				M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0; 
			
		}else 
		{
				M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;
				M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;  		
		}
				M_1_1;
				mask>>=1;
		} 
	}

	for (k = 0; k < count; k++) 
	{
			 colo1=mau1903[data1[k]];
			 colo2=mau1903[data2[k]];
			 d1_r=(gamma8[(uint8_t)(colo1>>16)])/bit;
			 d1_g=(gamma8[(uint8_t)(colo1>>8)])/bit;
			 d1_b=(gamma8[(uint8_t)(colo1)])/bit;
			 da1=d1_r<<16 | d1_g<<8 | d1_b;
			 d2_r=(gamma8[(uint8_t)(colo2>>16)])/bit;
			 d2_g=(gamma8[(uint8_t)(colo2>>8)])/bit;
			 d2_b=(gamma8[(uint8_t)(colo2)])/bit;
			 da2=d2_r<<16 | d2_g<<8 | d2_b;
			 mask=0x800000;
			for (i = 0; i <24; i++) 
			{
			/*
						if((mask & da1) && (mask & da2))
						 {
							 
								M_1_1;M_1_1;M_1_1;M_1_1; M_1_1;
								M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0; 
							 
						 } 
						 if((mask & da1) && !(mask & da2))
						 {
								M_1_0;M_1_0;M_1_0;M_1_0; M_1_0;
								M_0_0;M_0_0;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1; 
							 
						 } if(!(mask & da1) && (mask & da2))
						 {
								M_0_1;M_0_1;M_0_1;M_0_1; M_0_1;
								M_0_0;M_0_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0; 
						 }
							if(!(mask & da1) && !(mask & da2))
						 {
								M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;
								M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;  
						

						 }
						 */
			/*
			
			
						 if((mask & colo1) )
						{
								M_1_1;M_1_1;M_1_1;M_1_1; M_1_1;
								M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0; 
							
						}else 
						{
								M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;
								M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;  
							
						}
			
			*/
			/*
							if((mask & colo1) )
						{
								M_1_1;M_1_1;M_1_1;M_1_1; M_1_1;
								M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0; 
								M_1_1; 
						}else 
						{
								M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;
								M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;  
							
						}
			*/
			if((mask & da1) && (mask & da2))
			 {
				 
					M_1_1;M_1_1;M_1_1;M_1_1; M_1_1;//5
					M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;// 19
					M_1_1; 
			 } 
			 if((mask & da1) && !(mask & da2))
			 {
					M_1_0;M_1_0;M_1_0;M_1_0; M_1_0;
					M_0_0;M_0_0;M_0_0;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1; 
					M_1_1; 
				 
			 } if(!(mask & da1) && (mask & da2))
			 {
					M_0_1;M_0_1;M_0_1;M_0_1; M_0_1;
					M_0_0;M_0_0;M_0_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0; 
					M_1_1; 
			 }
				if(!(mask & da1) && !(mask & da2))
			 {
					M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;//8
					M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1; //17
			 }
				mask>>=1;
		} 
	} 
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_1_1;M_1_1;M_1_1;M_1_1;
	M_0_0;M_0_0; M_0_0;
	M_1_1;
/*	
for (k = 0; k < count; k++) 
{
	 
 colo1=mau1903[data1[k]];
 colo2=mau1903[data2[k]];
	   d1_r=((uint8_t)(colo1>>16))/bit;
	   d1_g=((uint8_t)(colo1>>8))/bit;
	   d1_b=((uint8_t)(colo1))/bit;
	da1=d1_r<<16 | d1_g<<8 | d1_b;
     d2_r=((uint8_t)(colo2>>16))/bit;
	   d2_g=((uint8_t)(colo2>>8))/bit;
	   d2_b=((uint8_t)(colo2))/bit;
	da2=d2_r<<16 | d2_g<<8 | d2_b;
	
	 
	  mask=0x800000;
	 for (i = 0; i <24; i++) 
	{
		       if((mask & da1) && (mask & da2))
					 {
						 
              M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1; M_1_1;M_1_1;
              M_0_0;M_0_0;M_0_0;M_0_0; 				 
					 } 
					 if((mask & da1) && !(mask & da2))
					 {
              M_1_1;M_1_1;M_1_1;M_1_1;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0; M_1_0;M_1_0; 
              M_0_0;M_0_0;M_0_0;M_0_0;
           } if(!(mask & da1) && (mask & da2))
					 {
              M_1_1;M_1_1;M_1_1;M_1_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1; M_0_1;M_0_1;
              M_0_0;M_0_0;M_0_0;M_0_0; 
           }
					  if(!(mask & da1) && !(mask & da2))
					 {
              M_1_1;M_1_1;M_1_1;M_1_1; 
              M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0; M_0_0; 


					 }

					mask>>=1;
	} 
	  
	  
	 

}
M_1_1;
 delay_ms(100);
 */
}

void send1903m24(int count,uint8_t bit)// what is difference m8/m24
{
	int k,h1=0,h2=6144;
	uint8_t i,l,d=0;
	uint32_t mask=0,da1,da2;
	uint8_t d1_r,d1_g,d1_b;
	uint8_t d2_r,d2_g,d2_b;
 
	for (k = 0; k < count; k++) 
	{
	 d1_r=data1[(k*3+0)]/bit;
	 d1_g=data1[(k*3+1)]/bit;
	 d1_b=data1[(k*3+2)]/bit;
	 da1=d1_r<<16 | d1_g<<8 | d1_b;
   d2_r=data2[(k*3+0)]/bit;

	 d2_g=data2[(k*3+1)]/bit;
	 d2_b=data2[(k*3+2)]/bit;
	 da2=d2_r<<16 | d2_g<<8 | d2_b;
	
	 
	  mask=0x800000;
	 for (i = 0; i <24; i++) 
	 {
		       if((mask & da1) && (mask & da2))
					 {
						 
              M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1;M_1_1; M_1_1;M_1_1;
              M_0_0;M_0_0;M_0_0;M_0_0; 				 
					 } 
					 if((mask & da1) && !(mask & da2))
					 {
              M_1_1;M_1_1;M_1_1;M_1_1;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0;M_1_0; M_1_0;M_1_0; 
              M_0_0;M_0_0;M_0_0;M_0_0;
           } if(!(mask & da1) && (mask & da2))
					 {
              M_1_1;M_1_1;M_1_1;M_1_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1;M_0_1; M_0_1;M_0_1;
              M_0_0;M_0_0;M_0_0;M_0_0; 
           }
					  if(!(mask & da1) && !(mask & da2))
					 {
              M_1_1;M_1_1;M_1_1;M_1_1; 
              M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0;M_0_0; M_0_0; 
					 }
					mask>>=1;
	  } 	 
  }
 
}


void send6803m(int count,uint8_t bit) // 16 bit 
{
  int i, k,l;
	uint8_t r,g,b,mask;

	
	
    SCLK_0;
    MOSI_0;

	  // First shift in 32bit ‚Äú0‚Äù as start frame, then shift in all data frame, start frame and dataframe both are shift by high-bit, every data is input on DCLK rising edge.
    /* Start frame */
    for (i = 0; i < 32; i++) {
        SCLK_1;
        SCLK_0;
    }
 
    /* Color cell output */
    for (k = 0; k < count; k++) {
        /* Start bit */
			// The first data frame is corresponding LED light nearest from shift-in polar, its format includes 1bit as start ‚Äú1‚Äù plus 3 groups 5bits grey level.
			MOSI_1;
			SCLK_1;
			SCLK_0;
      if(k<3072) l=mau6803[data1[k]];
			else l=mau6803[data2[k-3072]];
      
			   r= (gamma8[( (l>>10)&0x1f)*8]/8)  /bit;
				 g= (gamma8[( (l>>5)&0x1f)*8]/8)  /bit;
				 b= (gamma8[( (l)&0x1f)*8]/8)  /bit;
			  mask=0x10;
        for (i = 0; i < 5; i++) {
            if(mask &r )MOSI_1;
            else MOSI_0;					
            SCLK_1;
            SCLK_0;        
					  mask>>=1;
        }
				 mask=0x10;
        for (i = 0; i < 5; i++) {
            if(mask &g )MOSI_1;
            else MOSI_0;					
            SCLK_1;
            SCLK_0;        
					mask>>=1;
        }
				 mask=0x10;
        for (i = 0; i < 5; i++) {
            if(mask &b )MOSI_1;
            else MOSI_0;					
            SCLK_1;
            SCLK_0;        
					mask>>=1;
        }
			 
        
    }
 
    /* End frame */
		// after output all nDots data, need add nDots pulse
    MOSI_0;
    for (k = 0; k < count; k++) {
        SCLK_1;   
        SCLK_0;
    }
	
}
// not understand data[0] to data[27];
void set_pixel( int vt,uint8_t mau)
{
	 if(loaiday==1)
		{
			if(vt<3072)data1[vt]=mau;
			else if(vt>=3072 && vt<6144)data2[vt-3072]=mau;
			
		}
		 else
		{
			
			if(vtport2==0 && tongport2==0)
			{
				if(vt<3072)data1[vt]=mau;
			}
			else
			{
				if(vt<tongport1)
				{
					data1[vt]=mau;
				}
				else  if((vt-tongport1)<3072 &&  (vt-tongport1)>=0 )data2[vt-tongport1]=mau;
			}
		}
}

void set_led(int vt)
{
	LED1_0;LED2_0;LED3_0;LED4_0;LED5_0; 
	if(vt==0) LED1_1;
	else  if(vt==1) LED2_1;
	else  if(vt==2) LED3_1;
	else  if(vt==3) LED4_1;
	else    LED5_1;
	 
}
void set_fill_led(int vt)
{
	LED1_0;LED2_0;LED3_0;LED4_0;LED5_0; 
  if(vt==1) {LED1_1;}
	else  if(vt==2) {LED1_1;LED2_1;}
	else  if(vt==3) {LED1_1;LED2_1;LED3_1;}
	else  if(vt==4) {LED1_1;LED2_1;LED3_1;LED4_1;}
	else  if(vt==5) {LED1_1;LED2_1;LED3_1;LED4_1;LED5_1;}
	 
}

void loi(void)
{
	if(dangnap==0)
	{
		LEDERROR_0;
		Delay(300000);
 
		 LEDERROR_1;
		 Delay(300000);
	}else
	
	{
	//	LEDUSB_1;
		
	}
}	

void nhay_doi(void)
{
	LED1_1;LED2_1;LED3_1;LED4_1;LED5_1; 
	  Delay(300000);
	 LED1_0;LED2_0;LED3_0;LED4_0;LED5_0; 
	  Delay(300000);
 	LED1_1;LED2_1;LED3_1;LED4_1;LED5_1; 
	  Delay(300000);
	 LED1_0;LED2_0;LED3_0;LED4_0;LED5_0; 
	  Delay(300000);
	LED1_1;LED2_1;LED3_1;LED4_1;LED5_1; 
	  Delay(300000);
	 LED1_0;LED2_0;LED3_0;LED4_0;LED5_0; 
	  Delay(300000);
	LED1_1;LED2_1;LED3_1;LED4_1;LED5_1; 
	  Delay(300000);
	 LED1_0;LED2_0;LED3_0;LED4_0;LED5_0; 
	  Delay(300000);
	LED1_1;LED2_1;LED3_1;LED4_1;LED5_1; 
	  Delay(300000);
	 LED1_0;LED2_0;LED3_0;LED4_0;LED5_0; 
	  Delay(300000);	
}	
#define OB_RDP_LEVEL_1 ((uint8_t)0x55)
#define FLASH_KEY1               ((u32)0x45670123)
#define FLASH_KEY2               ((u32)0xCDEF89AB)
#define  FLASH_OPTKEY1                       FLASH_KEY1                    /*!< Option Byte Key1 */
#define  FLASH_OPTKEY2                       FLASH_KEY2                    /*!< Option Byte Key2 */
#define OB_RDP_LEVEL1 OB_RDP_LEVEL_1
void lock(void)
{
	WRITE_REG(FLASH->KEYR, FLASH_KEY1);
WRITE_REG(FLASH->KEYR, FLASH_KEY2);
#if defined(FLASH_BANK2_END)
WRITE_REG(FLASH->KEYR2, FLASH_KEY1);
WRITE_REG(FLASH->KEYR2, FLASH_KEY2);
#endif /* FLASH_BANK2_END */
WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY1);
WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY2);
if(READ_BIT(FLASH->OBR, FLASH_OBR_RDPRT)!=FLASH_OBR_RDPRT)
{
while((FLASH->SR & FLASH_FLAG_BSY)==FLASH_FLAG_BSY);
SET_BIT(FLASH->CR, FLASH_CR_OPTER);
SET_BIT(FLASH->CR, FLASH_CR_STRT);
while((FLASH->SR & FLASH_FLAG_BSY)==FLASH_FLAG_BSY);
CLEAR_BIT(FLASH->CR, FLASH_CR_OPTER);
SET_BIT(FLASH->CR, FLASH_CR_OPTPG);
WRITE_REG(OB->RDP, OB_RDP_LEVEL_1);
while((FLASH->SR & FLASH_FLAG_BSY)==FLASH_FLAG_BSY);
CLEAR_BIT(FLASH->CR, FLASH_CR_OPTPG);
/* Initiates a system reset request to launch the option byte loading */
NVIC_SystemReset();
} 
}	


void giaima(uint8_t da,int bd,int tong)
{
	int i;
	if(da==0)
	{
		
	   	for (i = 0; i < tong; i++) 
			 {
					 data1[i]=data1[i]^mahoa[(i+bd)%8];
			 }
	}else
	{
			for (i = 0; i < tong; i++) 
			 {
					 data2[i]=data2[i]^mahoa[(i+bd)%8];
			 }
	}
			 
}
unsigned char demloi[]={1,1,1,1};
void readbandau(void)
{
	 int i,j;
	 tong_play=0;
	 den_trungbinh=0;
	 loaiden=0;
   vantoc=0;
   //dosang=0;
	 dem_play=0;
 
	 Flash_Read(data1,0,32); // why dont separate
	//dosang=data1[13];
	if( data1[0]=='L' && data1[1]=='E' && data1[2]=='D'  )
	{
		  
	}
	else
	{
		while(1)
		{
		 	loi(); // if error happens 
		//	printf("loi L E D ");
			demloi[0]=0;
		}
	} 
	loaiden=data1[3];
	loaiday=data1[4];
	//vantoc=data1[5];
	//dosang=data1[6];
	vtport1=data1[7]+data1[8]*256; 
	tongport1=data1[9]+data1[10]*256; 
  vtport2=data1[11]+data1[12]*256; 
	tongport2=data1[13]+data1[14]*256;            
  den_trungbinh=data1[15]+data1[16]*256;                    
  tong_play=data1[17]+data1[18]*256;              
   
  vantoc=get_flash(0); if(vantoc>100)vantoc=100;
  dosang=get_flash(1);if(dosang<=0)dosang=1;if(dosang>10)dosang=10;
  mahoa[0]=get_flash(20);
  mahoa[1]=get_flash(21);
  mahoa[2]=get_flash(22);
  mahoa[3]=get_flash(23);
  mahoa[4]=get_flash(24);
  mahoa[5]=get_flash(25);
  mahoa[6]=get_flash(26);
  mahoa[7]=get_flash(27);	
  
	if(tong_play<=0)
	{
		while(1)
		{
				loi();
	//      printf("loi tong play < 0");
			  demloi[1]=0;
		}
	}
	if(loaiday==1 &&den_trungbinh>6144 )
	{
			while(1)
			{
				loi();
		//		printf("loi loaiday==1 && den_trungbinh>6144: %d ",den_trungbinh);
				demloi[2]=0;
			}
	}
	if(loaiday==0 &&den_trungbinh>3072 )
	{
			while(1)
			{
				loi();
			//	printf("loaiday==0 &&den_trungbinh>3072: %d ",den_trungbinh);
				demloi[3]=0; 
			}
	}
	

	for (i = 0; i < 3072; i++)
	{
		data1[i]=0;
		data2[i]=0;
	}
  set_led(loaiden);
//	printf("success :)");
}

int main(void)
{
  int i,j;
	int dem=1;
	int demnut=0;
	uint8_t chedo=0;
	uint8_t test=0;
	uint8_t demmau=0;
	int demauto=0;
	uint8_t autochay=0;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 ,ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);       //??IO?????
 GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);

	 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_0;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	 GPIO_InitStruct.GPIO_Pin   =  GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_10;
	 GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
	  GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3  | GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;                            //????SPI2
  GPIO_Init(GPIOB, &GPIO_InitStruct);
		 GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_4;
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	 GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //???  
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //???
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //8?
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //????NSS
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //????? SYSCLK/16
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //??????
  SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRC??????????7 
  SPI_Init(SPI1, &SPI_InitStructure);

 // SPI_CalculateCRC(SPI1, DISABLE);
  SPI_Cmd(SPI1, ENABLE); 
	//GPIO_init( );
 // DMA_init( );
 // TIM2_init( );
	// init_adc();
RCC_Configuration_HSI_48Mhz_with_USBclock();
  Set_USBClock();    
	
  //  RTC_Init();	
  NVIC_Configuration();											  //?????

//Set_USBClock();    
delay_init();	
//lock();
USB_Init();
 
LED1_0;LED2_0;LED3_0;LED4_0;LED5_0; 
 /*
 		while (1)
{
	LEDUSB_0;LED1_0;LED2_0;LED3_0;LED4_0;LED5_0;LEDERROR_0;
	Delay(100000);
		LEDUSB_1;LED1_1;LED2_1;LED3_1;LED4_1;LED5_1;LEDERROR_1;
	Delay(100000);
} 
*/
  Delay(50000);
  Flash_CS_Low; 
  Flash_CS_High;
  Delay(50000);
  Flash_CS_Low; 
  Flash_CS_High;
//  loi();
  readbandau();
	
	while (1)
 {
	if(dangnap==0)
	{
		
		if(test==0)
		{
		if(loaiday==1)
		{
         if(tongport1<=3072)
		     {
             Flash_Read(data1,32 +dem_play*tongport1,tongport1);    giaima(0,0,tongport1);
					 
		     }
		     else
		     {
			       Flash_Read(data1,32 +dem_play*tongport1,3072);giaima(0,0,3072);
		  	     Flash_Read(data2,32 +dem_play*tongport1 + 3072,tongport1-3072);giaima(1,3072,tongport1-3072);
		     }	 
	  	    send6803m(tongport1,dosang);	
		}
		else
		{
			if(vtport2==0 && tongport2==0)
			{
			 	Flash_Read(data1,32+dem_play*tongport1 ,tongport1);giaima(0,0,tongport1);
			}else
			{
				Flash_Read(data1,32+dem_play*(tongport1+tongport2) ,tongport1);giaima(0,0,tongport1);
				Flash_Read(data2,32+dem_play*(tongport1+tongport2)+tongport1 ,tongport2);giaima(1,tongport1,tongport2);
				 
			}
			 if(loaiden==2)send1903m8_1914(den_trungbinh,dosang);
			 else send1903m8(den_trungbinh,dosang);
		}
		
		dem_play++; if(dem_play>=tong_play)dem_play=0;
    Delay(10000);
	  for (i = 0; i < vantoc; i++) Delay(10000);
		
	//if(NUT_LEN==0 &&  dosang>1 )dosang--;
	//if(NUT_XUONG==0 &&  dosang<15 )dosang++;
		
		if(NUT_TEST==0)
		{
					demnut=0;
				
				while(NUT_TEST ==0)
				{
					demnut++;		 
				}
				if(demnut>1000000)
				{
						if(chedo==0)chedo=1;
						else chedo=0;
						nhay_doi();
						set_led(loaiden);
				}else
				{
						if(demnut>10000)
						{
							test=1;
							demmau=0;
						}
				}
				delay_ms(300);
			}
			
			 if(NUT_XUONG==0)
				 {
					
					 if(chedo==0  )
					 {
								if(vantoc<99)
								{
								 vantoc++;
								 luu_flash();			 
								 delay_ms(300);
								}else
								{
										 LEDERROR_1;
										 Delay(300000);
										 LEDERROR_0;
								}
						 
					 }else if(chedo==1)
					 {
							if(dosang<10)
								{
									 dosang++;
									 luu_flash();
									 delay_ms(300);;
								}else
								{
									 LEDERROR_1;
									 Delay(300000);
									 LEDERROR_0;
								}
					 }
				 }   
				 if(NUT_LEN==0)
				 {
						
						 if(chedo==0 )
						 {
							 if(vantoc>0)
									{
									 vantoc--;
									 luu_flash();			 
									 delay_ms(300);
									}else
									{
											 LEDERROR_1;
											 Delay(300000);
											 LEDERROR_0;
									}		 
						 }else if(chedo==1)
						 {
								 if( dosang>2)
									{
									 dosang--;
									 luu_flash();
									 delay_ms(300);;
									}else
									{
											 LEDERROR_1;
											 Delay(300000);
											 LEDERROR_0;
									}
						 }
					} 
					if(	baonap==0)
					{
						 LEDUSB_0; 
					}else
					{
						 if(dem_play%2==0  )LEDUSB_0; 
					   else LEDUSB_1; 
					}					
		 }
     else if(test==1)
     {	

			for (i = 0; i < tongport1+tongport2; i++)
			{
				 if(demmau==0)	set_pixel(i,1);
				 else if(demmau==1)	set_pixel(i,2);
				 else if(demmau==2)	set_pixel(i,4);
				 else  	set_pixel(i,15);
			}
			
		 if(loaiday==1)
			{
						
				 
						send6803m(tongport1,dosang);	
			}
			else
			{
						if(loaiden==2)send1903m8_1914(den_trungbinh,dosang);
						else send1903m8(den_trungbinh,dosang);
				 
				
			}
			Delay(10000);
			for (i = 0; i < vantoc*10; i++) Delay(10000);
				
			demmau++;
			if(demmau>=4) demmau=0;
			
		if(NUT_TEST==0)
				{
					demnut=0;
					
					while(NUT_TEST==0  )
					{
						demnut++;
							 
					 }
					if(demnut>10000)
					{
					 test=2;
						autochay=0;
						demauto=0;
					} 
					delay_ms(300);
				}
      }	
  	
	else if(test==2)
{

	if(autochay==0)
	{
		
		for (i = 0; i < tongport1+tongport2; i++)
	{
	      set_pixel(i,0);
	}
	for (i = 0; i < demauto; i++)
	{
	      set_pixel(i,15);
	}
	set_pixel(0,1);
	demauto++;
	if(demauto>tongport1+tongport2)demauto=0;
	
	if(demauto<tongport1+tongport2 && NUT_LEN==0)
	{
			autochay=1;
		 delay_ms(300);
	}
	if(demauto>1 && NUT_XUONG==0)
	{
		autochay=1;
		 delay_ms(300);
	}
		
		
	}else
	{
		
	for (i = 0; i < tongport1+tongport2; i++)
	{
	      set_pixel(i,0);
	}
	for (i = 0; i < demauto; i++)
	{
	      set_pixel(i,15);
	}
	  set_pixel(0,1);
		if(demauto<tongport1+tongport2 && NUT_LEN==0)
		{
	     demauto++;
	     delay_ms(300);
		}
		if(demauto>1 && NUT_XUONG==0)
		{
	     demauto--;
	     delay_ms(300);
		}
	}

 if(loaiday==1)
	{
				
				send6803m(tongport1,dosang);	
	}
	else
	{
		 
		 if(loaiden==2)send1903m8_1914(den_trungbinh,dosang);
		 else send1903m8(den_trungbinh,dosang);
		
	}
	   Delay(10000);
//	if(autochay==0)  for (i = 0; i < vantoc*10; i++)Delay(10000);
	//	else for (i = 0; i < vantoc; i++)Delay(10000);
	 for (i = 0; i < vantoc; i++)Delay(10000);
	
  if(NUT_TEST==0)
	{
		demnut=0;
		
		while(NUT_TEST==0  )
		{
			demnut++;
				 
		 }
		if(demnut>10000)
		{
		 test=0;
		} 
		delay_ms(300);
	}

 } 	 
}
	else
	
	{
		//LEDUSB_1;
		
	}
}
	

}

/* Private functions ---------------------------------------------------------*/

/**							  
  * @brief  Main program
  * @param  None						 
  * @retval : None
  */


void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}


/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */


/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
 
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);		   
  


  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	    //USB ??
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 		//????? 1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  


}



#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
