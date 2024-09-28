/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"		//信号量处理
#include "timers.h"

/* Library includes. */
#include "stm32f10x_it.h"
#include "serial.h"

/*hardware includes*/
#include "key.h"

static void prvSetupHardware( void );
int fputc( int ch, FILE *f );
extern void vSetupTimerTest( void );
extern void vApplicationIdleHook( void );

/*全局变量信号*/
int flag = 0;

/*Timer config 定时器*/
xTimerHandle MyTimer_Handle_1;

/*Semaphore Handle---信号量句柄定义区*/


/*Task stack size config 	栈大小*/
#define TASK1_STK_SIZE 100		
#define TASK2_STK_SIZE 100		
#define TASK3_STK_SIZE 100		


/*Task uxPriority config 	优先级*/
#define TASK1_TASK_PRIO 1		
#define TASK2_TASK_PRIO 1		
#define TASK3_TASK_PRIO 1		

/*Task Handle config		任务句柄定义*/
TaskHandle_t	xTaskTCB1;
TaskHandle_t	xTaskTCB2;
TaskHandle_t	xTaskTCB3;

/*Task funtion				任务函数定义*/
void Task1Function(void * param);
void Task2Function(void * param);
void Task3Function(void * param);



void vApplicationIdleHook( void )
{}

void mydelay(u16 num)
{
  u16 i,j;
  for(i=0;i<num;i++)
    for(j=0;j<0x800;j++);
}

void Task1Function(void * param)
{
	xTimerStart(MyTimer_Handle_1, 0);
	while (1)
	{

		vTaskDelay(10);
	}
}

void Task2Function(void * param)
{

	while (1)
	{

		vTaskDelay(1);
	}
}

void Task3Function(void * param)
{
	
	while(1)
	{
		vTaskDelay(1);
	}
}

void Timer1CallbackFunction()
{
	static int sum = 0;
	printf("Timer has run : %d \r\n", sum++);
	flag = !flag;
}

int main( void )
{
	prvSetupHardware();

	Key_Init();
	Led_Init();
	
	MyTimer_Handle_1 = xTimerCreate((const char * 			) "MyTimer", 
									(TickType_t				) 100,
									(UBaseType_t			) pdFALSE,
									(void * 				) NULL,
									(TimerCallbackFunction_t) Timer1CallbackFunction);
	 
	//创建三个任务
	xTaskCreate(( TaskFunction_t) Task1Function,
                (const char *	) "Task1Funtion", 
                (uint16_t		) TASK1_STK_SIZE,
                (void * 		) NULL,
                (UBaseType_t	) TASK1_TASK_PRIO,
                (TaskHandle_t *	) &xTaskTCB1 );
	xTaskCreate(( TaskFunction_t) Task2Function,
                (const char *	) "Task2Funtion", 
                (uint16_t		) TASK2_STK_SIZE,
                (void * 		) NULL,
                (UBaseType_t	) TASK2_TASK_PRIO,
                (TaskHandle_t *	) &xTaskTCB2 );					

	xTaskCreate(( TaskFunction_t) Task3Function,
                (const char *	) "Task3Funtion", 
                (uint16_t		) TASK3_STK_SIZE,
                (void * 		) NULL,
                (UBaseType_t	) TASK3_TASK_PRIO,
                (TaskHandle_t *	) &xTaskTCB3 );
				
				
				
	vTaskStartScheduler();
	return 0;
}

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

StackType_t xIdleTaskStack[100];
StaticTask_t xIdleTaskTCB;
/*静态分配任务*/
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
	*ppxIdleTaskStackBuffer = xIdleTaskStack;
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
	*pulIdleTaskStackSize = 100;
}

StackType_t xIdleTimerStack[100];
StaticTask_t xTimerTask;
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
	*ppxTimerTaskTCBBuffer = &xTimerTask;
	*ppxTimerTaskStackBuffer = xIdleTimerStack;
	*pulTimerTaskStackSize = 100;
}

void EXTI0_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	if(EXTI_GetITStatus(EXTI_Line0))
	{
		
		xTimerResetFromISR(MyTimer_Handle_1, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		
		EXTI_ClearFlag(EXTI_Line0);
	}
}
