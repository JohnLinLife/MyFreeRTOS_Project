/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x_it.h"
#include <string.h> 

/* Demo app includes. */

#include "serial.h"

#include "oled.h"
#include "keyboard.h"
#include "PWM.h"
#include "Beep.h"







//用户变量
QueueHandle_t KeyQueue;	//队列句柄
volatile char PassWord[6] = {0,0,0,0,0,0};
volatile char TruePasswd[6] = {1,0,0,0,0,0};
u8 current = 0,KeyChoose = 0, chances = 0;

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

/*----------------------------------------------------------------------*/

extern void vApplicationIdleHook( void )	//钩子函数
{}
	

/*-----------------------------------------------------------*/\
void AwaitState1()	//待机状态1，备选输入区
{
	u8 i=0;
	for(i = 0 ; i<6 ;i++)
	{
		OLED_ShowNum(i*8, 16, PassWord[i],1,16,0);
	}
	OLED_ShowChinese(64,16, 3, 16,1);	//确定
	OLED_ShowChinese(80,16, 4, 16,1);
	OLED_ShowChinese(0,0, 5, 16,1);		//密码
	OLED_ShowChinese(16,0, 6, 16,1);
	OLED_ShowChinese(64,32, 1, 16,1);	//设
	OLED_ShowChinese(80,32, 2, 16,1);	//置
}

void AwaitState2()	//待机状态2，备选确定区
{
	OLED_ShowString(0,16,"******",16,1);
	OLED_ShowChinese(64,16, 3, 16,0);	//确定
	OLED_ShowChinese(80,16, 4, 16,0);
	
	OLED_ShowChinese(0,0, 5, 16,1);	//密码
	OLED_ShowChinese(16,0, 6, 16,1);
	OLED_ShowChinese(64,32, 1, 16,1);	//设
	OLED_ShowChinese(80,32, 2, 16,1);	//置
}

void AwaitState3()	//待机状态3，备选修改区
{
	OLED_ShowString(0,16,"******",16,1);
	OLED_ShowChinese(64,16, 3, 16,1);	//确定
	OLED_ShowChinese(80,16, 4, 16,1);	
	
	OLED_ShowChinese(0,0, 5, 16,1);		//密码
	OLED_ShowChinese(16,0, 6, 16,1);
	OLED_ShowChinese(64,32, 1, 16,0);	//设
	OLED_ShowChinese(80,32, 2, 16,0);	//置
}

void PasswdInput()
{
	u8 i=0;
	bool BlackBlock = TRUE; 
	for(i = 0 ; i<6 ;i++)
	{
		if(i == KeyChoose)
			BlackBlock = FALSE;
		OLED_ShowNum(i*8, 16, PassWord[i],1,16,BlackBlock);
		BlackBlock = TRUE;
	}
	OLED_ShowString(0,0,"Choose the number",16,1);
}

void ChooseAdd()
{
	KeyChoose++;
	if(KeyChoose == 6)
		KeyChoose = 0;
	PasswdInput();
}
void ChooseLoss()
{
	if(KeyChoose == 0)
		KeyChoose = 6;
	else
		KeyChoose--;
	PasswdInput();
}


void EditPasswd()
{
	u8 i=0;
	for(i = 0 ; i<6 ;i++)
	{
		if(i == KeyChoose)
		{
			OLED_ShowChar(i*8, 16+8, '-',16,1);
		}
		OLED_ShowNum(i*8, 16, PassWord[i],1,16,1);
		OLED_ShowString(0,0,"Please enter",16,1);
	}
}
void NumberLoss()
{
	if(PassWord[KeyChoose]==0)
		PassWord[KeyChoose] = 9;
	else
		PassWord[KeyChoose]--;
	EditPasswd();
}
void NumberAdd()
{
	if(PassWord[KeyChoose]==9)
		PassWord[KeyChoose] = 0;
	else
		PassWord[KeyChoose]++;
	EditPasswd();
}

bool SamePasswd()
{
	u8 i = 0;
	for(i = 0; i < 6; i++)
	{
		if(TruePasswd[i] != PassWord[i])
			return FALSE;
	}
	return TRUE;
}
//Lock
#define LOCK_STK_SIZE		256
#define LOCK_TASK_PRIO		0
TaskHandle_t LockTask_Handler;
void LockDoor()
{
	uint16_t time;
	taskENTER_CRITICAL();	//进入临界区
	time = 10;
	OLED_Clear();
	
	while(time!=0)
	{
		
		time--;
		OLED_ShowString(16,0,"Please wait",16,1);
		OLED_ShowNum(48,16,time,2,16,1);
		OLED_ShowChar(64,16,'s',16,1);
		OLED_Refresh();
		Beep_Ring();
		main_delay_ms(125);
		Beep_Close();
		main_delay_ms(125);
		Beep_Ring();
		main_delay_ms(125);
		Beep_Close();
		main_delay_ms(125);
	}
	current = 0;
	OLED_Clear();
	OLED_ShowString(8,16,"Press any key",16,1);
	OLED_ShowString(32,32,"to back",16,1);
	OLED_Refresh();
	vTaskDelete(LockTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}

void CheckPasswd()
{
	int i = 0;
	bool Check = SamePasswd();
	OLED_Clear();
	if(Check == TRUE)
	{
		for(i = 0; i <6 ; i++)
			PassWord[i] = 0;
		OLED_ShowString(0,0,"Successful",16,1);
		chances = 0;
		OLED_Refresh();
		PWM_setCompare3(Angle_To_PWM(180));
		Beep_Ring();
		Beep_Close();
		main_delay_ms(1000);
		PWM_setCompare3(Angle_To_PWM(0));
	}
	else
	{
		for(i = 0; i <6 ; i++)
			PassWord[i] = 0;
		OLED_ShowString(0,0,"Unsuccessful",16,1);
		OLED_ShowString(0,16,"Only   chances",16,1);
		OLED_ShowNum(40,16, 3 - ++chances,1,16,1);
	}
	if(chances == 3)
	{
		chances = 0;
		OLED_ShowString(0,32,"lock",16,1);
		xTaskCreate((TaskFunction_t )LockDoor,            //任务函数
                (const char*    )"Locked",          //任务名称
                (uint16_t       )LOCK_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )LOCK_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&LockTask_Handler);   //任务句柄
	}
	OLED_Refresh();
}
void NewPasswd()
{
	u8 i = 0;
	for(i = 0; i<6; i++)
		TruePasswd[i] = PassWord[i];
	for(i=0; i<6; i++)
		PassWord[i] = 0;
	OLED_ShowString(0,0,"Change successful",16,1);
	OLED_Refresh();
}

void Setting_Passwd()
{
	chances = 0;
	OLED_ShowString(0,0,"1.Enter NewPasswd",16,1);
	OLED_ShowString(0,16,"2.Edit NewPasswd",16,1);
	OLED_ShowString(0,32,"3.Cancel",16,1);
}

void CheckSetting()
{
	u8 i = 0;
	bool Check = SamePasswd();
	if(Check == TRUE)
	{
		current = 11;
		PasswdInput();
	}
	else
	{
		for(i = 0; i <6 ; i++)
			PassWord[i] = 0;
		OLED_ShowString(0,0,"Unsuccessful",16,1);
		OLED_ShowString(0,16,"Only   chances",16,1);
		OLED_ShowNum(40,16, 3 - ++chances,1,16,1);
	}
	if(chances == 3)
	{
		chances = 0;
		OLED_ShowString(0,32,"lock",16,1);
		xTaskCreate((TaskFunction_t )LockDoor,            //任务函数
                (const char*    )"Locked",          //任务名称
                (uint16_t       )LOCK_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )LOCK_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&LockTask_Handler);   //任务句柄
	}
	OLED_Refresh();
}

typedef struct{
	u8 current; 	//当前索引
	u8 last;		//上一个
	u8 next;		//下一个
	u8 enter;		//下一级
	u8 backspace;	//上一级
	void (*current_operation)(void);		//状态执行函数
}Menu_Table;

Menu_Table table[20] = {
	{0, 0, 1, 2, 0, (*AwaitState1)},		//待机备选输入
	{1, 0, 9, 8, 1, (*AwaitState2)},		//待机备选确认
	{2, 3, 4, 5, 0, (*PasswdInput)},		//输入界面
	{3, 3, 4, 5, 0, (*ChooseLoss)},			//选位--
	{4, 3, 4, 5, 0, (*ChooseAdd)},			//选位++
	{5, 6, 7, 2, 2, (*EditPasswd)},			//密码编辑界面
	{6, 6, 7, 2, 2, (*NumberLoss)},			//密码数字--
	{7, 6, 7, 2, 2, (*NumberAdd)},			//密码数字++
	{8, 1, 1, 1, 1, (*CheckPasswd)},		//验证密码
	
	{9, 1, 9, 18, 9, (*AwaitState3)},		//待机设置项
	//验证与选项需要分离 18
	{10, 17, 11, 9, 9, (*Setting_Passwd)},	//设置新密码输入界面
	{11, 12, 13, 14, 10, (*PasswdInput)},	//输入	
	{12, 12, 13, 14, 10, (*ChooseLoss)},	//选位--
	{13, 12, 13, 14, 10, (*ChooseAdd)},		//选位++
	{14, 15, 16, 11, 11, (*EditPasswd)},			//密码编辑界面
	{15, 15, 16, 11, 11, (*NumberLoss)},			//密码数字--
	{16, 15, 16, 11, 11, (*NumberAdd)},			//密码数字++
	{17, 0, 0, 0, 0, (*NewPasswd)},
	
	{18, 0, 0, 0, 0, (*CheckSetting)},		//验证
};

void TaskPasswd(void * param)
{
	uint16_t key;
	table[current].current_operation();
	OLED_Refresh();
	while (1)
	{
		xQueueReceive(KeyQueue, &key, portMAX_DELAY);	//读取按键并保存在Key变量中
		/*********************State 1*********************/
		switch(key)
		{
			case 1: current = table[current].last; break;
			case 2: current = table[current].next; break;
			case 3: current = table[current].enter; break;
			case 4: current = table[current].backspace; break;
		}
		OLED_Clear();
		table[current].current_operation();
		
		OLED_Refresh();
		vTaskDelay(100);
	}
}

void TaskKeyboardRead(void * param)
{
	uint16_t KeyRead;
	while (1)
	{
		KeyRead = 0;
		KeyRead = KeyBoard_Read();
		vTaskDelay(1);
		if(KeyRead!=0)
			xQueueSendToBack(KeyQueue, &KeyRead, portMAX_DELAY);
		
//		OLED_Refresh();
	}
}


//开始任务初始化配置
#define START_STK_SIZE		256
#define START_TASK_PRIO		1
TaskHandle_t StartTask_Handler;

//密码任务初始化配置
#define PASSWD_STK_SIZE		1024
#define PASSWD_TASK_PRIO	2
TaskHandle_t PasswdTask_Handler;

//key
#define KEY_STK_SIZE		256
#define KEY_TASK_PRIO	1
TaskHandle_t KeyTask_Handler;

void start_task(void * param)
{
	taskENTER_CRITICAL();	//进入临界区
	KeyQueue = xQueueCreate(1, sizeof(uint16_t));	//创建消息队列
	
	xTaskCreate((TaskFunction_t )TaskPasswd,            //任务函数
                (const char*    )"Passwd",          //任务名称
                (uint16_t       )PASSWD_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )PASSWD_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&PasswdTask_Handler);   //任务句柄	
	xTaskCreate((TaskFunction_t )TaskKeyboardRead,            //任务函数
                (const char*    )"Key",          //任务名称
                (uint16_t       )KEY_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )KEY_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&KeyTask_Handler);   //任务句柄
	
	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
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

int main( void )
{
	
#ifdef DEBUG
  debug();
#endif

	prvSetupHardware();

	OLED_Init();
	Beep_Init();
	main_delay_ms(50);
	KeyBoard_Init();
	PWM_Init();
	PWM_setCompare3(Angle_To_PWM(0));

	xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄
	
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
