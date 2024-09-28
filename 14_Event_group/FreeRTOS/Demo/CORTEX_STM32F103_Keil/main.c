/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"		//信号量处理
#include "event_groups.h"

/* Library includes. */
#include "stm32f10x_it.h"
#include "serial.h"


static void prvSetupHardware( void );
int fputc( int ch, FILE *f );
extern void vSetupTimerTest( void );
extern void vApplicationIdleHook( void );

/*全局变量信号*/
int flag = 0;

/*EventGroup 事件组句柄定义区*/
EventGroupHandle_t EventGroup_Handle_1;
EventGroupHandle_t EventGroup_Handle_2;

/*Queue队列句柄定义区*/
QueueHandle_t Queue_Handle_1;

/*Semaphore Handle---信号量句柄定义区*/
SemaphoreHandle_t USART_Lock;

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

/*Task5 create config*/
TaskHandle_t	xTaskHandle5;
void Task5(void * param);
/*Task6 create config*/
TaskHandle_t	xTaskHandle6;
void Task6(void * param);
/*Task7 create config*/
TaskHandle_t	xTaskHandle7;
void Task7(void * param);


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
	int i = 0,sum = 0;
	while (1)
	{
		for(i = 0; i < 100000; i++)
		{
			sum++;
		}
		xQueueSend(Queue_Handle_1, &sum, 0);
		xEventGroupSetBits(EventGroup_Handle_1, (1<<0));
		vTaskDelay(1);
	}
}

void Task2Function(void * param)
{
	int i = 0,sum = 0;
	while (1)
	{
		for(i = 0; i < 100000; i++)
		{
			sum--;
		}
		xQueueSend(Queue_Handle_1, &sum, 0);
		xEventGroupSetBits(EventGroup_Handle_1, (1<<1));		//对1号事件写
		vTaskDelay(1);
	}
}

void Task3Function(void * param)
{
	int sum, dsc;
	while(1)
	{
		xEventGroupWaitBits(EventGroup_Handle_1,((1<<0)|(1<<1)), pdTRUE, pdTRUE, portMAX_DELAY);	//读取事件组，配置为：0位和1位、读取后清理、两条见相与触发、一直等待
		xQueueReceive(Queue_Handle_1, &sum, 0);		
		xQueueReceive(Queue_Handle_1, &dsc, 0);
		flag = sum - dsc;
		//printf("val1 = %d, val2 = %d\r\n", sum, dsc);
	}
}

#define COOK	(1<<0)
#define BUY		(1<<1)
#define WASH	(1<<2)
#define ALL		(1<<0)|(1<<1)|(1<<2)

void Task4Function(void * param)
{

	while(1)
	{
		
		vTaskDelay(1);
	}
}

void Task5(void * param)
{
	int i = 0;
	while(1)
	{
		xSemaphoreTake(USART_Lock, portMAX_DELAY);
		printf("COOK waitting...\r\n");
		xSemaphoreGive(USART_Lock);
		
		xEventGroupWaitBits(EventGroup_Handle_2, BUY, pdFALSE, pdTRUE, portMAX_DELAY);	//等待BUY完成，且不清理
		
		for(i = 0 ; i < 50000; i++);
		xSemaphoreTake(USART_Lock, portMAX_DELAY);
		printf("COOK finish...\r\n");
		xSemaphoreGive(USART_Lock);
		xEventGroupSync(EventGroup_Handle_2, COOK, ALL, portMAX_DELAY);
		vTaskDelay(1);
	}
}
void Task6(void * param)
{
	int i = 0;
	while(1)
	{
		for(i = 0 ; i < 50000; i++);
		
		xSemaphoreTake(USART_Lock, portMAX_DELAY);
		printf("BUY finish...\r\n");
		xSemaphoreGive(USART_Lock);
		
		xEventGroupSync(EventGroup_Handle_2, BUY, ALL, portMAX_DELAY);
		
		xSemaphoreTake(USART_Lock, portMAX_DELAY);
		printf("eat!\r\n");
		xSemaphoreGive(USART_Lock);
		vTaskDelay(1);
	}
}
void Task7(void * param)
{
	int i = 0;
	while(1)
	{
		vTaskDelay(10);
		xSemaphoreTake(USART_Lock, portMAX_DELAY);
		printf("WASH finish...\r\n");
		xSemaphoreGive(USART_Lock);
		
		xEventGroupSync(EventGroup_Handle_2, WASH, ALL, portMAX_DELAY);
		flag = 1;
	}
}	

int main( void )
{
	prvSetupHardware();
	EventGroup_Handle_1 = xEventGroupCreate();	//创建事件组
	EventGroup_Handle_2 = xEventGroupCreate();	//创建事件组
	Queue_Handle_1 = xQueueCreate(2, sizeof(int));	//创建信号队列，大小为2，每个容量为int
	USART_Lock = xSemaphoreCreateMutex();		//串口锁
	
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
	xTaskCreate(Task5, "Task5", 100, NULL, 1, &xTaskHandle5);
	xTaskCreate(Task6, "Task6", 100, NULL, 1, &xTaskHandle6);
	xTaskCreate(Task7, "Task7", 100, NULL, 1, &xTaskHandle7);
									
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
