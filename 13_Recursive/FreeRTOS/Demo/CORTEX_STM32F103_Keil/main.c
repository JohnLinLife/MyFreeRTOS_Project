/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"		//信号量处理

/* Library includes. */
#include "stm32f10x_it.h"
#include "serial.h"


static void prvSetupHardware( void );
int fputc( int ch, FILE *f );
extern void vSetupTimerTest( void );
extern void vApplicationIdleHook( void );

/*全局变量信号*/
int flag = 0;

/*Semaphore Handle---信号量句柄定义区*/
SemaphoreHandle_t Semaphore_Handle_1;
SemaphoreHandle_t Semaphore_Handle_USART1;

/*Task1 static create config*/
#define TASK1_STK_SIZE 100		//栈大小
#define TASK1_TASK_PRIO 1		//优先级
StackType_t	xTask1Stack[TASK1_STK_SIZE];
StaticTask_t	xTaskHandle1;
TaskHandle_t	xTaskTCB1;
void Task1Function(void * param);

/*Task2 static create config*/
#define TASK2_STK_SIZE 100		//栈大小
#define TASK2_TASK_PRIO 1		//优先级
StackType_t	xTask2Stack[TASK2_STK_SIZE];
StaticTask_t	xTaskHandle2;
TaskHandle_t	xTaskTCB2;
void Task2Function(void * param);

/*Task3 static create config*/
#define TASK3_STK_SIZE 100		//栈大小
#define TASK3_TASK_PRIO 1		//优先级
StackType_t	xTask3Stack[TASK3_STK_SIZE];
StaticTask_t	xTaskHandle3;
TaskHandle_t	xTaskTCB3;
void Task3Function(void * param);

/*Task4 static create config*/
#define TASK4_STK_SIZE 100		//栈大小
#define TASK4_TASK_PRIO 1		//优先级
StackType_t	xTask4Stack[TASK4_STK_SIZE];
StaticTask_t	xTaskHandle4;
TaskHandle_t	xTaskTCB4;
void Task4Function(void * param);

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
	int sum, i;
	while (1)
	{
		for(i = 0; i < 1000000; i++)
			sum++;				//计算
		xSemaphoreGive(Semaphore_Handle_1);
	}
}

void Task2Function(void * param)
{
	while (1)
	{
		xSemaphoreTakeRecursive(Semaphore_Handle_USART1, portMAX_DELAY);
		printf("task2 is running\r\n");
		xSemaphoreGiveRecursive(Semaphore_Handle_USART1);
		vTaskDelay(1);
	}
}

void Task3Function(void * param)
{
	int i;
	while(1)
	{
		xSemaphoreTakeRecursive(Semaphore_Handle_USART1, portMAX_DELAY);
		printf("task3 is running\r\n");
		for(i = 0; i < 10; i++)
		{
			xSemaphoreTakeRecursive(Semaphore_Handle_USART1, portMAX_DELAY);
			printf("task3 is running loop times:%d\r\n", i);
			xSemaphoreGiveRecursive(Semaphore_Handle_USART1);
		}
		xSemaphoreGiveRecursive(Semaphore_Handle_USART1);
		vTaskDelay(1);
	}
}

void Task4Function(void * param)
{
	while(1)
	{
		flag = 1;
		while(1)
		{
			if(xSemaphoreTakeRecursive(Semaphore_Handle_USART1, portMAX_DELAY) != pdTRUE)
			{
				xSemaphoreGiveRecursive(Semaphore_Handle_USART1);
			}
			else
			{
				break;
			}
		}
		printf("task4 is running\r\n");
		
		xSemaphoreGiveRecursive(Semaphore_Handle_USART1);
		flag = 0;
		vTaskDelay(1);
	}
}

int main( void )
{
	prvSetupHardware();
	Semaphore_Handle_1 = xSemaphoreCreateCounting(10, 0);
	Semaphore_Handle_USART1 = xSemaphoreCreateRecursiveMutex();

	//创建三个任务
	xTaskTCB1 = xTaskCreateStatic((TaskFunction_t )Task1Function,            //任务函数
									(const char*    )"Task1Funtion",          //任务名称
									(uint16_t       )TASK1_STK_SIZE,        //任务堆栈大小
									(void*          )NULL,                  //传递给任务函数的参数
									(UBaseType_t    )TASK1_TASK_PRIO,       //任务优先级
									(StackType_t *  )xTask1Stack,
                                    (StaticTask_t * )&xTaskHandle1 );
	xTaskTCB2 = xTaskCreateStatic((TaskFunction_t )Task2Function,            //任务函数
									(const char*    )"Task2Funtion",          //任务名称
									(uint16_t       )TASK2_STK_SIZE,        //任务堆栈大小
									(void*          )NULL,                  //传递给任务函数的参数
									(UBaseType_t    )TASK2_TASK_PRIO,       //任务优先级
									(StackType_t *  )xTask2Stack,
                                    (StaticTask_t * )&xTaskHandle2 );
	xTaskTCB3 = xTaskCreateStatic((TaskFunction_t )Task3Function,            //任务函数
									(const char*    )"Task3Funtion",          //任务名称
									(uint16_t       )TASK3_STK_SIZE,        //任务堆栈大小
									(void*          )NULL,                  //传递给任务函数的参数
									(UBaseType_t    )TASK3_TASK_PRIO,       //任务优先级
									(StackType_t *  )xTask3Stack,
                                    (StaticTask_t * )&xTaskHandle3 );
	xTaskTCB4 = xTaskCreateStatic((TaskFunction_t )Task4Function,            //任务函数
									(const char*    )"Task4Funtion",          //任务名称
									(uint16_t       )TASK4_STK_SIZE,        //任务堆栈大小
									(void*          )NULL,                  //传递给任务函数的参数
									(UBaseType_t    )TASK4_TASK_PRIO,       //任务优先级
									(StackType_t *  )xTask4Stack,
                                    (StaticTask_t * )&xTaskHandle4 );
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
