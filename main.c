
#include "stdio.h"
#include "NuMicro.h"
#include "M251.h"
#include "string.h"
#include "stdbool.h"
#include <math.h>

#define CLK_HIRC    0
#define CLK_HXT     1
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ

#define UART0_RXBUFSIZE   256
#define UART2_RXBUFSIZE   1024


#define SLAVE_ADDR 12
//#define DebugMode



#ifdef DebugMode
	#define debug_printf(...)  printf(_VA_ARGS_) 
#else
	#define debug_printf(...)
#endif

/---------------------------------------------------------------------------------------------------------/
/* Global variables                                                                                        */
/---------------------------------------------------------------------------------------------------------/
uint8_t UART0_g_au8RecData[UART0_RXBUFSIZE]  = {0};
volatile uint32_t UART0_g_u32ComRbytes = 0;
volatile uint32_t UART0_g_u32ComRhead  = 0;
volatile uint32_t UART0_g_u32ComRtail  = 0;

uint8_t UART2_g_au8RecData[UART2_RXBUFSIZE]  = {0};
volatile uint32_t UART2_g_u32ComRbytes = 0;
volatile uint32_t UART2_g_u32ComRhead  = 0;
volatile uint32_t UART2_g_u32ComRtail  = 0;


volatile int32_t g_i32Wait         = TRUE;

volatile uint8_t u8InChar =0;
volatile uint8_t u8InResponse =0;

volatile uint32_t g_au32TMRINTCount = 0;
volatile			uint8_t ptat25r[2] = {0,0};
volatile			uint16_t ptat25 = 0;
volatile			uint8_t M[2] = {0,0};
volatile			uint8_t U0[2] = {0,0};
volatile			uint8_t Uout1[2] = {0,0};
volatile			uint8_t TPambientr[2]= {0,0} ;
volatile			uint16_t TPambient  = 0;
volatile			uint32_t TPobject = 0;
volatile			uint8_t Tobj1 = 0;
volatile			float Tamb = 0;
volatile			float k = 0;
volatile			float Tobject = 0;
volatile			uint8_t TPobject1[3] = {0};
volatile      uint16_t M_total = 0;
volatile      uint16_t U0_total = 0;
volatile      uint32_t Uout1_total = 0;
volatile 			uint8_t protocol = 0;
volatile 			uint8_t ecr = 0;
volatile 			uint8_t adress = 0;
volatile      uint8_t lookup = 0;
volatile      uint8_t presence = 0;
volatile      uint8_t motion = 0;

volatile      float presence_temp_tresh = 26.7;

#if defined (_GNUC) && !defined(_ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif







/---------------------------------------------------------------------------------------------------------/
/* UART Callback function                                                                                  */
/---------------------------------------------------------------------------------------------------------/
void UART_TEST_HANDLE0(void)
{

    uint32_t u32IntSts = UART0->INTSTS;

    if ((u32IntSts & UART_INTSTS_RDAINT_Msk) || (u32IntSts & UART_INTSTS_RXTOINT_Msk))
    {

        /* Get all the input characters */
        while (UART_GET_RX_EMPTY(UART0) == 0)
        {
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART0);
						
            

            /* Check if buffer full */
            if (UART0_g_u32ComRbytes < UART0_RXBUFSIZE)
            {
                /* Enqueue the character */
                UART0_g_au8RecData[UART0_g_u32ComRtail] = u8InChar;
                UART0_g_u32ComRtail = (UART0_g_u32ComRtail == (UART0_RXBUFSIZE - 1)) ? 0 : (UART0_g_u32ComRtail + 1);
                UART0_g_u32ComRbytes++;
            }
        }


    }

   

    if (UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART0->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk);
    }
}







uint32_t ReadFromUartBuffer(uint8_t *rxBuffer, volatile uint32_t *head, volatile uint32_t *tail, volatile uint32_t *bytes, char *outputBuffer, uint32_t outputBufferSize, uint32_t bufferCapacity)
{
    if (*bytes == 0)
        return FALSE;  

    
    uint32_t idx = *head;
    uint32_t i = 0;
    bool found = false;
    while (i < *bytes)
    {
        char c = rxBuffer[idx];
        i++;
        if (c == '\r' || c == '\n')
        {
            found = true;
            break;
        }
        idx = (idx + 1) % bufferCapacity;
    }
    if (!found)
        return FALSE; 

   
    uint32_t toCopy = i < (outputBufferSize - 1) ? i : (outputBufferSize - 1);
    for (uint32_t i = 0; i < toCopy; i++)
    {
        outputBuffer[i] = rxBuffer[*head];
        *head = (*head + 1) % bufferCapacity;
    }
    outputBuffer[toCopy] = '\0';

    
    *bytes -= i;
    return TRUE;
}








void UART_TEST_HANDLE2(void)
{

    uint32_t u32IntSts = UART2->INTSTS;

    if ((u32IntSts & UART_INTSTS_RDAINT_Msk) || (u32IntSts & UART_INTSTS_RXTOINT_Msk))
    {

        /* Get all the input characters */
        while (UART_GET_RX_EMPTY(UART2) == 0)
        {
            /* Get the character from UART Buffer */
            u8InResponse = UART_READ(UART2);

            /* Check if buffer full */
            if (UART2_g_u32ComRbytes < UART2_RXBUFSIZE)
            {
                /* Enqueue the character */   
                UART2_g_au8RecData[UART2_g_u32ComRtail] = u8InResponse;
                UART2_g_u32ComRtail = (UART2_g_u32ComRtail == (UART2_RXBUFSIZE - 1)) ? 0 : (UART2_g_u32ComRtail + 1);
                UART2_g_u32ComRbytes++;
            }
        }


    }

    

    if (UART2->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART2->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk);
    }
}







/---------------------------------------------------------------------------------------------------------/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/---------------------------------------------------------------------------------------------------------/
void UART0_IRQHandler(void)
{
    UART_TEST_HANDLE0();
}

void UART2_IRQHandler(void)
{
    UART_TEST_HANDLE2();
}



/---------------------------------------------------------------------------------------------------------/
/* Init UART                                                                                               */
/---------------------------------------------------------------------------------------------------------/
void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void UART2_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART2, 115200);
}



void SYS_Init(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* If the macros do not exist in your project, please refer to the related clk.h in Header folder of the tool package */
    /* Enable clock source */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* Set PLL frequency */
    CLK->PLLCTL = (CLK->PLLCTL & ~(0x001FDE3FUL)) | 0x00084410UL;

    /* Waiting for PLL ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Set HCLK clock */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK-related clock */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Enable IP clock */
    CLK_EnableModuleClock(FMCIDLE_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);
    CLK_EnableModuleClock(ISP_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(UART2_MODULE);
		CLK_EnableModuleClock(TMR0_MODULE);

    /* Set IP clock */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_PCLK1, CLK_CLKDIV0_UART1(1));
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_PCLK0, CLK_CLKDIV4_UART2(1));
		CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
		
		 /* If the macros do not exist in your project, please refer to the corresponding header file in Header folder of the tool package */
    SYS->GPA_MFPH = 0x00000000;
    SYS->GPA_MFPL = 0x00000000;
    SYS->GPB_MFPH = SYS_GPB_MFPH_PB13MFP_UART0_TXD | SYS_GPB_MFPH_PB12MFP_UART0_RXD;
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB5MFP_INT0 | SYS_GPB_MFPL_PB3MFP_UART1_TXD | SYS_GPB_MFPL_PB2MFP_UART1_RXD;
    SYS->GPC_MFPH = 0x00000000;
    SYS->GPC_MFPL = SYS_GPC_MFPL_PC5MFP_I2C1_SCL | SYS_GPC_MFPL_PC4MFP_I2C1_SDA | SYS_GPC_MFPL_PC1MFP_UART2_TXD | SYS_GPC_MFPL_PC0MFP_UART2_RXD;
    SYS->GPD_MFPH = 0x00000000;
    SYS->GPD_MFPL = 0x00000000;
    SYS->GPE_MFPH = 0x00000000;
    SYS->GPE_MFPL = 0x00000000;
    SYS->GPF_MFPL = SYS_GPF_MFPL_PF1MFP_ICE_CLK | SYS_GPF_MFPL_PF0MFP_ICE_DAT;

    /* Lock protected registers */
    SYS_LockReg();


}



void I2C1_Init(void)
{
		I2C_Open(I2C1, 100000);
}



void tpis_1s_1385_Init()
{
			
			CLK_SysTickDelay(300000);
			I2C_WriteByteOneReg(I2C1,0x00,0x04,0x00);

			I2C_WriteByteOneReg(I2C1, 0x0C, 31,  0x80 );
			
			CLK_SysTickDelay(300000);
			
			protocol = I2C_ReadByteOneReg(I2C1, 0x0C, 32);
			debug_printf("protocol: %d \n",protocol);
			ecr = I2C_ReadByteOneReg(I2C1, 0x0C, 31) ;
			adress = I2C_ReadByteOneReg(I2C1, 0x0C, 63) ;
			debug_printf("adress: %d \n",adress);
			
			lookup = I2C_ReadByteOneReg(I2C1, 0x0C, 41 );
			debug_printf("lookup: %d \n",lookup);
			I2C_ReadMultiBytesOneReg(I2C1, 0x0C, 42, &ptat25r[0], 2);
	    ptat25 = ((ptat25r[0])<<8 )| ptat25r[1] ;
	
			I2C_ReadMultiBytesOneReg(I2C1, 0x0C, 44, &M[0], 2);
				M_total = ((M[0]<<8) | M[1]);
	      M_total /=100;

			I2C_ReadMultiBytesOneReg(I2C1, 0x0C, 46, &U0[0], 2);
				U0_total = ((U0[0]<<8) | U0[1]);
				U0_total += 32768 ;
			
			I2C_ReadMultiBytesOneReg(I2C1, 0x0C, 48, &Uout1[0], 2);
			Uout1_total = ((Uout1[0]<<8) | (Uout1[1]));
			Uout1_total *=2;
			
			Tobj1 = I2C_ReadByteOneReg(I2C1, 0x0C, 50);
			
			I2C_WriteByteOneReg(I2C1, 0x0C, 31,  0x00 );
			
}



void readValues()
{
		I2C_ReadMultiBytesOneReg(I2C1, 12, 3, &TPambientr[0], 2);
		TPambient = (((uint16_t)TPambientr[0] & 0x7F)<<8|(uint16_t)TPambientr[1]) ;
		I2C_ReadMultiBytesOneReg(I2C1, 12, 1, &TPobject1[0], 3);

		
		TPobject=  (((uint32_t)TPobject1[0]<<24) | ((uint32_t)TPobject1[1]<<16) | ((uint32_t)TPobject1[2] & 0x80 )<<8)>>15;
		presence = I2C_ReadByteOneReg(I2C1, 12, 15); 
		debug_printf("presence : %d \n",presence);
		motion = I2C_ReadByteOneReg(I2C1, 12, 16);  ;
		debug_printf("motion : %d \n",motion);
	
}


void calc_k()
{
	k = ((float)(Uout1_total-U0_total))/(powf((float)(Tobj1 + 273.15f), 3.3f)-powf((298.15f), 3.3f));
}

void Tamb_Calc()
{
	float ta1 = 0;
	float ta2 = 0;
	float ta3 = 0;
	
	ta1 = 298.15f ;
	ta2 = (float)(TPambient-ptat25) ;
	ta3 = 1.0f / (float)M_total ;
	Tamb = (float)ta1 + ((float)(ta2*ta3)) ;

}


void Tobject_Calc()
{
	float t1 = 0;
	float t2 = 0;
	float t3 = 0;
	
	t1 = ((float)TPobject - (float)U0_total) ;
	t2 = (t1 / (float)k) ;
  t3 = powf((float)Tamb, 3.3f) ;

	Tobject = powf((float)(t2+t3), 1.0f/3.3f) ;
	Tobject -=273.15;
	debug_printf("t1: %f\n", t1);
	debug_printf("t2: %f\n", t2);
	debug_printf("t3: %f\n", t3);
}






void UART_SendString(UART_T *uart, const char *str)
{
	while(*str != '\0')
	{
		while(UART_IS_TX_FULL(uart));
		UART_WRITE(uart,*str);
		str++;
	}
}





/*  Main Function                                                                                          */
/---------------------------------------------------------------------------------------------------------/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();
	
		char docklightRxLine[UART0_RXBUFSIZE]; 
    char quectelRxLine[UART2_RXBUFSIZE];
	
#if defined (_GNUC) && !defined(_ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init UART0 for printf and test */
    UART0_Init();
		
		UART2_Init();
		
		I2C1_Init();
		
		uint8_t i ;
		char uart0_out[100] = {0} ;
		
		
		
		NVIC_EnableIRQ(UART0_IRQn);
		UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
		UART_SetTimeoutCnt(UART0, 0x10);
		UART2->FIFO &= ~ UART_FIFO_RFITL_Msk;
		UART2->FIFO |= UART_FIFO_RFITL_4BYTES;


		NVIC_EnableIRQ(UART2_IRQn);
		UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
		UART_SetTimeoutCnt(UART2, 0x10);
		UART2->FIFO &= ~ UART_FIFO_RFITL_Msk;
		UART2->FIFO |= UART_FIFO_RFITL_4BYTES;
		
		
		
		
		
		tpis_1s_1385_Init();
		CLK_SysTickDelay(300);
		calc_k();
		Tamb_Calc();
		Tobject_Calc();

		TIMER_Delay(TIMER0, 100000);
		
		UART_SendString(UART2,"AT+QMTDISC=0\r\n");
		
		TIMER_Delay(TIMER0, 500000);
			
		UART_SendString(UART2,"AT+QMTCFG=\"recv/mode\",0,1,1\r\n");
		
		TIMER_Delay(TIMER0, 100000);
		
		UART_SendString(UART2,"AT+QMTCFG=\"keepalive\",0,3000\r\n");
		
		TIMER_Delay(TIMER0, 100000);
		
		UART_SendString(UART2,"AT+QMTCFG=\"timeout\",0,50,3,1\r\n");
		
		TIMER_Delay(TIMER0, 100000);
		
		UART_SendString(UART2,"AT+QMTOPEN=0,\"test.mosquitto.org\",1883\r\n");
		
		TIMER_Delay(TIMER0, 500000);
		
		UART_SendString(UART2,"AT+QMTCONN=0,\"clientid1\"\r\n");
		
		TIMER_Delay(TIMER0, 100000);
		
		UART_SendString(UART2,"AT+QMTSUB=0,1,\"termopile\",1\r\n");
		
		TIMER_Delay(TIMER0, 100000);
		
		UART_SendString(UART2,"AT+QMTSUB=0,2,\"temp_presence\",1\r\n");
		
		TIMER_Delay(TIMER0, 100000);
		
		
		
		
		UART_SendString(UART2, "AT+CMGD=1,4\r\n");
    while (1)
		{

			
			
			
			char docklightRxLine[UART0_RXBUFSIZE] = {0}; 
			char quectelRxLine[UART2_RXBUFSIZE] = {0};

		
		
			
			readValues();
			Tamb_Calc();
			Tobject_Calc();
			
			TIMER_Delay(TIMER0, 100000);
			
			char Tobject_str[32];
			sprintf(Tobject_str, "%.2f", Tobject);
			
			if(ReadFromUartBuffer(UART0_g_au8RecData, &UART0_g_u32ComRhead, &UART0_g_u32ComRtail, &UART0_g_u32ComRbytes, docklightRxLine, sizeof(docklightRxLine), UART0_RXBUFSIZE ))
			{
				UART_SendString(UART2, docklightRxLine);
				
				i = 0;
				for(i=0; i<100; i++)
				{
					uart0_out[i] = docklightRxLine ;
				}
				
				}
				else
				{
				i=0;
				memset(uart0_out,0,sizeof(uart0_out));
			
				}
			if(ReadFromUartBuffer(UART2_g_au8RecData, &UART2_g_u32ComRhead, &UART2_g_u32ComRtail, &UART2_g_u32ComRbytes, quectelRxLine, sizeof(quectelRxLine), UART2_RXBUFSIZE ))
			{

				UART_SendString(UART0, quectelRxLine);

			}
			
			
			
			uint16_t current_msgid1 = 1 ;
			char cmd1[128];
			sprintf( cmd1, "AT+QMTPUBEX=0,%d,1,0,\"termopile\",%d\r\n",current_msgid1,strlen(Tobject_str));
			current_msgid1++;
			TIMER_Delay(TIMER0, 100000);
			UART_SendString(UART2, Tobject_str);
			
			
			UART_SendString(UART2, "AT+QMTPUBEX=0,1,1,0,\"termopile\",");
			 
			char len_str[5];
			sprintf(len_str, "%d", strlen(Tobject_str)); 
			UART_SendString(UART2, len_str); 
			UART_SendString(UART2, "\r"); 
			TIMER_Delay(TIMER0, 100000);
			UART_SendString(UART2, Tobject_str); 
			UART_SendString(UART2, "\r\n"); 
			
//			UART_SendString(UART2,"AT+QMTRECV=0\r\n");
			
			
			TIMER_Delay(TIMER0, 100000);
			
			
			
			
			
			
			
			if(Tobject > 28.0)
			{
				
				UART_SendString(UART2, "AT+QMTPUBEX=0,2,1,0,\"temp_presence\",");
				char presen_m[20];
				sprintf(presen_m, "%d", 18);
				UART_SendString(UART2, presen_m); 
				UART_SendString(UART2, "\r");
				TIMER_Delay(TIMER0, 200000);
				UART_SendString(UART2, "someone broke in.\r\n"); 
				TIMER_Delay(TIMER0, 20000);
//				UART_SendString(UART2,"AT+QMTRECV=0\r\n");
				
				
				UART_SendString(UART2,"AT+CMGF=1\r\n");
		
				TIMER_Delay(TIMER0, 100000);
				
				UART_SendString(UART2,"AT+CNMI=2,1\r\n");
				
				TIMER_Delay(TIMER0, 100000);
				
				UART_SendString(UART2,"AT+CMGS=\"+905379739787\"\r\n");
				
				TIMER_Delay(TIMER0, 100000);
				
				UART_SendString(UART2,"human detected! \x1A\r\n");
				
				TIMER_Delay(TIMER0, 100000);
				
			}
			
			else
			{
				
				UART_SendString(UART2, "AT+QMTPUBEX=0,2,1,0,\"temp_presence\",");
				char presen_m[20];
				sprintf(presen_m, "%d", 12);
				UART_SendString(UART2, presen_m); 
				UART_SendString(UART2, "\r");
				TIMER_Delay(TIMER0, 200000);
				UART_SendString(UART2, "nobody home.\r\n"); 
				TIMER_Delay(TIMER0, 20000);
			
			
			
			}
			
			

			
			
				char sms_message[32] = {0};
				double num_sms_message;

				TIMER_Delay(TIMER0, 100000);
//				UART_SendString(UART2, "AT+CMGD=1,4\r\n"); 
//				TIMER_Delay(TIMER0, 100000);
//				UART_SendString(UART2, "AT+CMGR=1\r\n"); 
//				UART_SendString(UART2, "AT+CMGL=\"ALL\"\r\n"); 

			if(ReadFromUartBuffer(UART2_g_au8RecData, &UART2_g_u32ComRhead, &UART2_g_u32ComRtail, &UART2_g_u32ComRbytes, quectelRxLine, sizeof(quectelRxLine), UART2_RXBUFSIZE ))
			{
			
				if(ReadFromUartBuffer(UART2_g_au8RecData, &UART2_g_u32ComRhead, &UART2_g_u32ComRtail, &UART2_g_u32ComRbytes, quectelRxLine, sizeof(quectelRxLine), UART2_RXBUFSIZE ))
			{

				strcpy(sms_message,quectelRxLine);
				num_sms_message = atof(sms_message);
				if(num_sms_message>10 && num_sms_message<40)
				{
					
					presence_temp_tresh = num_sms_message;
				}

			}
			}
			
			
			
		}
	}
