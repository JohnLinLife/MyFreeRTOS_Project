/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x_it.h"

/* Demo app includes. */
#include "BlockQ.h"
#include "death.h"
#include "integer.h"
#include "blocktim.h"
#include "partest.h"
#include "semtest.h"
#include "PollQ.h"
#include "flash.h"
#include "comtest2.h"
#include "serial.h"

#include "oled.h"

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCREATOR_TASK_PRIORITY           ( tskIDLE_PRIORITY + 3 )
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainCOM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )



/* The check task uses the sprintf function so requires a little more stack. */
#define mainCHECK_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN						25

/* The time between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned long ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Baud rate used by the comtest tasks. */
#define mainCOM_TEST_BAUD_RATE		( 115200 )



/*-----------------------------------------------------------*/

/*
 * Configure the clocks, GPIO and other peripherals as required by the demo.
 */
static void prvSetupHardware( void );


/*
 * Retargets the C library printf function to the USART.
 */
int fputc( int ch, FILE *f );


/*
 * Configures the timers and interrupts for the fast interrupt test as
 * described at the top of this file.
 */
extern void vSetupTimerTest( void );

extern void vApplicationIdleHook( void );
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	
}

void Task1Function(void * param)
{
	while (1)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);
		vTaskDelay(1000);
		GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
		vTaskDelay(1000);
	}
}

void Task2Function(void * param)
{
	//TaskHandle_t xHandleTask1;
	uint16_t i = 0;
	OLED_Init();
	while (1)
	{
		OLED_ShowNum(64,0,i++,4,16,1);
		OLED_Refresh();
		printf("asd");
		vTaskDelay(500);
//		xTaskCreate(Task1Function, "Task1", 100, NULL, 1, &xHandleTask1);
	}
}

void Task3Funtion(void * param)
{
	while(1)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
		vTaskDelay(1000);
		GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
		vTaskDelay(1000);
	}
}



void LED_Init(void)
{
	GPIO_InitTypeDef GPIOStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIOStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOStruct.GPIO_Pin = GPIO_Pin_2;
	GPIOStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIOStruct);
	GPIOStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIOStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIOStruct);
	
}
/*----------------------------------------------------------------------*/
/*------------------------Task3Config-----------------------------------*/
StackType_t xTask3Stack[100];
StaticTask_t xHandleTask3;

StackType_t xIdleTaskStack[100];
StaticTask_t xIdleTaskTCB;
/*----------------------------------------------------------------------*/

/*静态分配任务*/
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
	*ppxIdleTaskStackBuffer = xIdleTaskStack;
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
	*pulIdleTaskStackSize = 100;
}

int main( void )
{
	
#ifdef DEBUG
  debug();
#endif

	TaskHandle_t xHandleTask1;		//任务句柄，用于后续对该任务进行操作,

	prvSetupHardware();
	
	LED_Init();
	
	xTaskCreate(Task1Function, "Task1", 100, NULL, 1, &xHandleTask1);
	xTaskCreate(Task2Function, "Task2", 1000, NULL, 1, NULL);
	
	
	xTaskCreateStatic(Task3Funtion,"Task3",100,NULL,1,xTask3Stack, &xHandleTask3);
	
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* 只有在没有足够的堆空间来创建空闲任务时才会到达这里 */
	return 0;
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Start with the clocks in their expected state. */
	RCC_DeInit();

	/* Enable HSE (high speed external clock). */
	RCC_HSEConfig( RCC_HSE_ON );

	/* Wait till HSE is ready. */
	while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
	{
	}

	/* 2 wait states required on the flash. */
	*( ( unsigned long * ) 0x40022000 ) = 0x02;

	/* HCLK = SYSCLK */
	RCC_HCLKConfig( RCC_SYSCLK_Div1 );

	/* PCLK2 = HCLK */
	RCC_PCLK2Config( RCC_HCLK_Div1 );

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config( RCC_HCLK_Div2 );

	/* PLLCLK = 8MHz * 9 = 72 MHz. */
	RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );

	/* Enable PLL. */
	RCC_PLLCmd( ENABLE );

	/* Wait till PLL is ready. */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );

	/* Wait till PLL is used as system clock source. */
	while( RCC_GetSYSCLKSource() != 0x08 )
	{
	}

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

	/* SPI2 Periph clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );


	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

	SerialPortInit();
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

#ifdef  DEBUG
/* Keep the linker happy. */
void assert_failed( unsigned char* pcFile, unsigned long ulLine )
{
	for( ;; )
	{
	}
}
#endif
