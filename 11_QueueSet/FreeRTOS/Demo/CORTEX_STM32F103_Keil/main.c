/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x_it.h"
#include "serial.h"


static void prvSetupHardware( void );
int fputc( int ch, FILE *f );
extern void vSetupTimerTest( void );
extern void vApplicationIdleHook( void );

/*Queue Handle*/
static QueueHandle_t Queue1_Handle;
static QueueHandle_t Queue2_Handle;
static QueueSetHandle_t QueueSet_Handle_1;

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

/*全局变量监测区*/
//int a = 0;
//int b = 0;

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
	int i = 0;
	while (1)
	{
		xQueueSend(Queue1_Handle, &i, portMAX_DELAY);
		i++;
		vTaskDelay(10);
	}
}

void Task2Function(void * param)
{
	int i = -1;
	while (1)
	{
		xQueueSend(Queue1_Handle, &i, portMAX_DELAY);
		i--;
		vTaskDelay(20);
	}
}

void Task3Function(void * param)
{
	QueueSetMemberHandle_t QueueSetMember;
	int i;
	while(1)
	{
		QueueSetMember = xQueueSelectFromSet(QueueSet_Handle_1, portMAX_DELAY);		//读取队列集的有数据的队列
		xQueueReceive(QueueSetMember, &i, 0);		//读取该队列的数据
		printf("Get data : %d\r\n", i);
		
	}
}

int main( void )
{
	prvSetupHardware();
	printf("start!");
	
	/*队列及队列集的创建*/
	Queue1_Handle = xQueueCreate(2, sizeof(int));	//创建队列1
	if(Queue1_Handle == NULL)
		printf("Queue not created\r\n");
	Queue2_Handle = xQueueCreate(2, sizeof(int));
	if(Queue2_Handle == NULL)
		printf("Queue not created\r\n");
	QueueSet_Handle_1 = xQueueCreateSet(4);			//创建队列集，大小为队列长度之和
	if(QueueSet_Handle_1 == NULL)
		printf("Queue Set not created\r\n");
	xQueueAddToSet(Queue1_Handle, QueueSet_Handle_1);		//将队列添加到队列集中
	xQueueAddToSet(Queue2_Handle, QueueSet_Handle_1);
	
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
	xTaskTCB2 = xTaskCreateStatic((TaskFunction_t )Task3Function,            //任务函数
									(const char*    )"Task3Funtion",          //任务名称
									(uint16_t       )TASK3_STK_SIZE,        //任务堆栈大小
									(void*          )NULL,                  //传递给任务函数的参数
									(UBaseType_t    )TASK3_TASK_PRIO,       //任务优先级
									(StackType_t *  )xTask3Stack,
                                    (StaticTask_t * )&xTaskHandle3 );
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
