/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


typedef enum{
	flag_PDMA_Abort = 0 ,
	flag_PDMA_Done ,	
	
	flag_DEFAULT	
}flag_Index;


volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))


#define FIFO_THRESHOLD 							(4)
#define RX_BUFFER_SIZE 							(256)
#define RX_TIMEOUT_CNT 							(60) //40~255

#define UART_RX_IDEL(uart) (((uart)->FIFOSTS & UART_FIFOSTS_RXIDLE_Msk )>> UART_FIFOSTS_RXIDLE_Pos)

typedef struct {
	uint8_t RX_Buffer[RX_BUFFER_SIZE];
	uint16_t Length;
	uint8_t RDA_Trigger_Cnt;
	uint8_t RXTO_Trigger_Cnt;
	
//	uint8_t end;
}UART_BUF_t;

UART_BUF_t uart0Dev;


#define PDMA_CHx  						(0)
#define EPWM_CAPx						(5)


__IO uint16_t g_u32Count[4];
volatile uint32_t g_u32CH0TestOver = 0;

uint8_t pwm_duty = 30 ;
const uint32_t pwm_freq = 15 ;

uint16_t capt_psc = 0;
uint32_t capt_div_freq = 0 ;


void CalPeriodTime(EPWM_T *EPWM, uint32_t u32Ch)
{
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;

    g_u32CH0TestOver = 0;
    /* Wait PDMA interrupt (g_u32CH0TestOver will be set at IRQ_Handler function) */
    while(g_u32CH0TestOver == 0);

    u16RisingTime = g_u32Count[1];

    u16FallingTime = g_u32Count[0];

    u16HighPeriod = g_u32Count[1] - g_u32Count[2] + 1;

    u16LowPeriod = 0xFFFF - g_u32Count[1];

    u16TotalPeriod = 0xFFFF - g_u32Count[2] + 1;

    printf("Rising=%5d,Falling=%5d,High=%5d,Low=%5d,Total=%5d,Freq=%5d,Duty=%3d.\r\n",
           u16RisingTime, 
           u16FallingTime, 
           u16HighPeriod, 
           u16LowPeriod, 
           u16TotalPeriod,
           capt_div_freq/u16TotalPeriod,
           u16HighPeriod*100/u16TotalPeriod);
}


void CalculationPolling(void)
{
	PH1 ^= 1;
	
    EPWM1->CAPCTL |= EPWM_CAPCTL_FCRLDEN5_Msk;	
	
    while((EPWM1->CNT[EPWM_CAPx]) == 0);

    /* Capture the Input Waveform Data */
    CalPeriodTime(EPWM1, EPWM_CAPx);
	
}

void EPWM_CAP_Disable(void)
{
    EPWM_Stop(EPWM1, EPWM_CH_5_MASK);

    /* Wait until EPWM1 channel 2 current counter reach to 0 */
    while((EPWM1->CNT[EPWM_CAPx] & EPWM_CNT5_CNT_Msk) != 0);

    EPWM_ForceStop(EPWM1, EPWM_CH_5_MASK);

    EPWM_DisableCapture(EPWM1, EPWM_CH_5_MASK);

    EPWM_ClearCaptureIntFlag(EPWM1, EPWM_CAPx, EPWM_CAPTURE_PDMA_RISING_FALLING_LATCH);

	EPWM_DisablePDMA(EPWM1, EPWM_CAPx);

    NVIC_DisableIRQ(PDMA_IRQn);

    PDMA_Close(PDMA);
	
}

void EPWM_CAP_PDMA_Init(void)
{

    PDMA_Open(PDMA,BIT0);

    PDMA_SetTransferCnt(PDMA,PDMA_CHx, PDMA_WIDTH_16, 4);

    PDMA_SetTransferAddr(PDMA,PDMA_CHx, (uint32_t)&(EPWM1->PDMACAP[2]), PDMA_SAR_FIX, (uint32_t)&g_u32Count[0], PDMA_DAR_INC);

    PDMA_SetTransferMode(PDMA,PDMA_CHx, PDMA_EPWM1_P3_RX, FALSE, 0);

    PDMA_SetBurstType(PDMA,PDMA_CHx, PDMA_REQ_SINGLE, PDMA_BURST_4);

    PDMA_EnableInt(PDMA,PDMA_CHx, PDMA_INT_TRANS_DONE);
	
    NVIC_EnableIRQ(PDMA_IRQn);

    EPWM_EnablePDMA(EPWM1, EPWM_CAPx, FALSE, EPWM_CAPTURE_PDMA_RISING_FALLING_LATCH);
}

void EPWM_CAP_Init(void)	
{
	uint32_t target_ns = 0;
		
	/*
			use PLL : 192000000
			use PCLK1 : depend on div. , exmaple : div = 2 , capture clock = 192000000/2 = 96000000
	
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency

           how to get PSC , make CNR+1 close to 0xFFFF , 
           target freq = 15  
           ==> CNR+1 = 192000000 / PSC / min_freq , when CNR = 65535 with min freq = 10 , will get PSC 293

           Capture unit time = 1/Capture clock source frequency/prescaler
           ==> target ns = 1/ 192000000 / 293 = 293/ 192000000 = 1526 ns

	*/
	capt_psc = 293;
	target_ns = 1526;
	capt_div_freq = FREQ_192MHZ/capt_psc;

    EPWM_ConfigCaptureChannel(EPWM1, EPWM_CAPx, target_ns, 0);	//target_ns

    EPWM_Start(EPWM1, EPWM_CH_5_MASK);

    EPWM_EnableCapture(EPWM1, EPWM_CH_5_MASK);

//    EPWM1->CAPCTL |= EPWM_CAPCTL_FCRLDEN5_Msk;

//    while((EPWM1->CNT[EPWM_CAPx]) == 0);

//    CalPeriodTime(EPWM1, EPWM_CAPx);
}


void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF0_Msk)
            g_u32CH0TestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA,PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF0_Msk)
        {		
			g_u32CH0TestOver = 1;
			
//			EPWM_CAP_Disable();
			EPWM_CAP_PDMA_Init();
			EPWM_CAP_Init();
        }
		PDMA_CLR_TD_FLAG(PDMA,PDMA_TDSTS_TDIF0_Msk);	
		
    }
    else
        printf("unknown interrupt !!\n");
}


void EPWM_DeInit(void)
{
    /* Set PWM0 channel 0 loaded value as 0 */
    EPWM_Stop(EPWM0, EPWM_CH_2_MASK);

    /* Wait until PWM0 channel 0 Timer Stop */
//    while((PWM0->CNT[0] & PWM_CNT_CNT_Msk) != 0);

    /* Disable Timer for PWM0 channel 0 */
    EPWM_ForceStop(EPWM0, EPWM_CH_2_MASK);

    /* Disable PWM Output path for PWM0 channel 0 */
    EPWM_DisableOutput(EPWM0, EPWM_CH_2_MASK);
}

void EPWM_Init(void)	//PA3
{
    EPWM_ConfigOutputChannel(EPWM0, 2, pwm_freq, pwm_duty);

    EPWM_EnableOutput(EPWM0, EPWM_CH_2_MASK);

    /* Start EPWM0 counter */
    EPWM_Start(EPWM0, EPWM_CH_2_MASK);
}



void TMR1_IRQHandler(void)
{
	static uint16_t CNT = 0;	
//	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
			PH0 ^= 1;
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);

		}
		
    }
}

void TIMER1_Init(void)
{
	GPIO_SetMode(PH, BIT0, GPIO_MODE_OUTPUT);	//monitor
	GPIO_SetMode(PH, BIT1, GPIO_MODE_OUTPUT);	//monitor
	
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}


void UART0_Process(uint8_t rx)
{
	switch(rx)
	{
		case '1':				
			pwm_duty = ( pwm_duty > 90) ? (90) : (pwm_duty+1) ;
    		EPWM_ConfigOutputChannel(EPWM0, 2, pwm_freq, pwm_duty);			
			break;

		case '2':
			pwm_duty = ( pwm_duty < 10) ? (10) : (pwm_duty-1) ;		
    		EPWM_ConfigOutputChannel(EPWM0, 2, pwm_freq, pwm_duty);			
			break;

		case 'Z':
		case 'z':
			NVIC_SystemReset();
			break;

		default :
			break;
	}	
}


void UART0_IRQHandler(void)
{
	uint8_t i;
	static uint16_t u16UART_RX_Buffer_Index = 0;

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk))    
    {
        /* UART receive data available flag */
        
        /* Record RDA interrupt trigger times */
        uart0Dev.RDA_Trigger_Cnt++;
        
        /* Move the data from Rx FIFO to sw buffer (RAM). */
        /* Every time leave 1 byte data in FIFO for Rx timeout */
        for(i = 0 ; i < (FIFO_THRESHOLD - 1) ; i++)
        {
            uart0Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;

            if (u16UART_RX_Buffer_Index >= RX_BUFFER_SIZE) 
                u16UART_RX_Buffer_Index = 0;
        }
    }
    else if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RXTOINT_Msk)) 
    {
        /* When Rx timeout flag is set to 1, it means there is no data needs to be transmitted. */

        /* Record Timeout times */
        uart0Dev.RXTO_Trigger_Cnt++;

        /* Move the last data from Rx FIFO to sw buffer. */
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            uart0Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;
        }

        /* Clear UART RX parameter */
        UART_DISABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
        u16UART_RX_Buffer_Index = 0;

//		set_flag(flag_UART0_Received_Data , ENABLE);

		#if 1
		printf("UART RX : 0x%2X \r\n" , uart0Dev.RX_Buffer[0]);
		
		UART0_Process(uart0Dev.RX_Buffer[0]);
		#else

        printf("\nUART0 Rx Received Data : %s\n",uart0Dev.RX_Buffer);
        printf("UART0 Rx RDA (Fifofull) interrupt times : %d\n",uart0Dev.RDA_Trigger_Cnt);
        printf("UART0 Rx RXTO (Timeout) interrupt times : %d\n",uart0Dev.RXTO_Trigger_Cnt);
		#endif

        /* Reset UART interrupt parameter */
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
		memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));

    }
	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, RX_TIMEOUT_CNT);

	/* Set UART FIFO RX interrupt trigger level to 4-bytes*/
    UART0->FIFO = ((UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES);

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);
	
	memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));

	UART_WAIT_TX_EMPTY(UART0);
	
//	set_flag(flag_UART0_Received_Data , DISABLE);

	printf("CLK_GetModuleClockSource : %8d\r\n",CLK_GetModuleClockSource(EPWM1_MODULE));
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

	

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    /* Enable IP module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* EPWM clock frequency is set double to PCLK: select EPWM module clock source as PLL */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PLL, (uint32_t)NULL);

    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA3MFP_Msk) | SYS_GPA_MFPL_PA3MFP_EPWM0_CH2;	

    /* Enable IP module clock */
    CLK_EnableModuleClock(EPWM1_MODULE);

    CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PLL, (uint32_t)NULL);

    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB6MFP_Msk );
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB6MFP_EPWM1_CH5 );	
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{	
    SYS_Init();

    UART0_Init();	
	TIMER1_Init();

    EPWM_Init();
	
	EPWM_CAP_PDMA_Init();
	EPWM_CAP_Init();

	/*
		output : EPWM0_CH2 (PA.3)
		capture : EPWM1_CH5 (PB.6)
	*/


    /* Got no where to go, just loop forever */
    while(1)
    {
	
		CalculationPolling();

    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
