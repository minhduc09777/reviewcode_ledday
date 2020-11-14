/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.2.1
* Date               : 07/05/2010
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "stm32f10x_GPIO.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"
#define Flash_CS_High    GPIO_SetBits(GPIOA, GPIO_Pin_15);
#define Flash_CS_Low    GPIO_ResetBits(GPIOA, GPIO_Pin_15);

unsigned char USB_buff [64];
char kkt=0;
int tongtruyen=0;
extern uint8_t data1[3072];
int itang=0;

uint8_t flag = 0;
int demt=0;
int tongnhan=0;
uint8_t ij=0;
int i1,i2;
unsigned long TT_XOA=0;
extern	uint8_t mahoa[8];
 extern uint8_t baonap;
 extern const uint8_t pass[];
extern void Flash_PageWrite(uint8_t *pBuffer,uint32_t PageAddress,uint16_t WriteByteNum);
extern	void Flash_EraseSector(uint32_t SectorAddress);
extern void Flash2048(uint8_t *pBuffer,uint32_t PageAddress);
extern void Delay(uint32_t num);
extern void  xoachip();
extern uint8_t dangnap;
extern void luu_flash();
extern void loi();
extern uint8_t vantoc;
extern uint8_t dosang;
int tongdata=0;
extern void Flash_Read(uint8_t *pBuffer,uint32_t ReadAddress,uint16_t ReadByteNum);
extern void set_fill_led(int vt);
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback (void)
{
      
 SetEPTxStatus(ENDP1, EP_TX_VALID);
	
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_OUT_Callback(void)
{
  USB_SIL_Read(EP1_OUT, USB_buff);	
	if(dangnap==0)
	{ 
			demt=0;
			tongnhan=0;
			dangnap=1;
			tongdata=USB_buff[4]+USB_buff[5]*256+USB_buff[6]*256*256;
			vantoc=USB_buff[7];
			dosang=USB_buff[8];
		  mahoa[0] = pass[USB_buff[20]];
		  mahoa[1] = pass[USB_buff[21]];
		  mahoa[2] = pass[USB_buff[22]];
		  mahoa[3] = pass[USB_buff[23]];
		  mahoa[4] = pass[USB_buff[24]];
		  mahoa[5] = pass[USB_buff[25]];
		  mahoa[6] = pass[USB_buff[26]];
		  mahoa[7] = pass[USB_buff[27]];
			dangnap=1;
		  if(USB_buff[9] != 'C' || USB_buff[10] != 'H' || USB_buff[11] != 'E' || USB_buff[12] != 'C' || USB_buff[13] != 'K')
			{
				  
			    while(1){
					   loi();
					}
			}
			luu_flash();
		   
		 //TT_XOA = tongdata%262144;
		 // if(TT_XOA==0) Flash_EraseSector(TT_XOA*262144);
		 /* else{
				 	for (i1 = 0; i1  < TT_XOA; i1 ++){
					     Flash_EraseSector((i1+1)*262144);
					}
			}*/
			//xoachip();
			//Flash_EraseSector();
			
			SetEPRxStatus(ENDP1, EP_RX_VALID);
	}
	else
	{
	 for (i1 = 0; i1  < 64; i1 ++)	data1[i1+demt*64]=USB_buff[i1]; 
		demt++;
		if(demt>=32)			
		{	
			demt=0;

     if(tongnhan==0){ 
			   Flash_EraseSector(0);
			}
			else if(tongnhan%32==0){
			   Flash_EraseSector(tongnhan*2048);
		  }
			//if(tongnhan%128==0)Flash_EraseSector(tongnhan*2048);
			Flash2048(data1,tongnhan*2048);			 
			tongnhan++;
			if(tongnhan>=tongdata)
			{
			   baonap=0;
			   SetEPRxStatus(ENDP1, EP_RX_VALID);
				 NVIC_SystemReset();
			}
			set_fill_led(tongnhan%5);
		};
     
      SetEPRxStatus(ENDP1, EP_RX_VALID);
  
    }
}


/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#ifdef STM32F10X_CL
void INTR_SOFINTR_Callback(void)
#else
void SOF_Callback(void)
#endif /* STM32F10X_CL */
{

   
}
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

