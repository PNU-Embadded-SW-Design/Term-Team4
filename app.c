

#include <includes.h>


#define		DARK_TRUE	(OS_FLAGS)0x0001
#define		DARK_FALSE	(OS_FLAGS)0x0002
#define		DATK_TRUE	(OS_FLAGS)0x0004

/*
*********************************************************************************************************
*                                             GLOBAL VARIABLES
*********************************************************************************************************
*/
CPU_INT16U val = 0;
CPU_BOOLEAN jodo = FALSE;
OS_FLAG_GRP	MyEventFlag;
CPU_BOOLEAN dark = FALSE;

static OS_SEM sem1;
static OS_SEM sem2;


/*
*********************************************************************************************************
*                                            LOCAL VARIABLES
*********************************************************************************************************
*/

static  OS_TCB   AppTaskStartTCB; 
static  CPU_STK  AppTaskStartStk[APP_TASK_START_STK_SIZE];

static  OS_TCB   JodoTCB; 
static  CPU_STK  JodoStk[APP_TASK_START_STK_SIZE];

static	 OS_TCB		JodoLedOnTCB;
static	 CPU_STK	JodoLedOnStk[APP_TASK_START_STK_SIZE];

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart  (void *p_arg);

static	void	JodoTask	(void *p_arg);

static	void	JodoLedOn	(void *p_arg);

static	void	Jodo_IntHandler	(CPU_DATA int_id);

void	JodoISR();
void sendDataUART1(CPU_INT16U data);
static void SendInteger(CPU_INT16U data);
static void SendInteger(CPU_INT16U data);
static void USART_Configure();
void	AdcInit(void);
void	GPIO_Configure(void);
void	RCC_Configure(void);
void NVIC_Configure(void) ;
void EXTI_Configure(void);
static CPU_INT16U readADC0(void);
void ADC_InterruptHandler();

int  main (void)
{
    
    OS_ERR      err;    
    BSP_IntDisAll();
    BSP_Init();                                                   /* Initialize BSP functions                         */
    CPU_Init();      
	NVIC_Configure();
    USART_Configure();
	EXTI_Configure();
    RCC_Configure();
    GPIO_Configure();
    //AdcInit();
	//BSP_IntVectSet(BSP_INT_ID_ADC1_2, JodoISR);

	OSSemCreate(&sem1, "ADC_OS_SEM", 0, &err);
	
	OSInit(&err);                                               /* Init uC/OS-III.                                      */
	
	OSFlagCreate(&MyEventFlag, "My Event Flag Group", (OS_FLAGS)0, &err);

    OSTaskCreate((OS_TCB     *)&AppTaskStartTCB,                /* Create the start task                                */
                 (CPU_CHAR   *)"App Task Start",
                 (OS_TASK_PTR )AppTaskStart, 
                 (void       *)0,
                 (OS_PRIO     )APP_TASK_START_PRIO,
                 (CPU_STK    *)&AppTaskStartStk[0],
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK     )0,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */

	while(DEF_ON){
	  ;
	}
}



static  void  AppTaskStart (void *p_arg)
{
  	OS_ERR err;
	CPU_TS ts;

    CPU_INT32U  cpu_clk_freq;
    CPU_INT32U  cnts;

	cpu_clk_freq = BSP_CPU_ClkFreq();
    cnts         = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;/* Determine nbr SysTick increments                         */
    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).                  */

#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running                */
#endif

    CPU_IntDisMeasMaxCurReset();
	
	/* Initialize the uC/CPU services                   */
		//BSP_IntVectSet(EXTI15_10_IRQChannel, JodoISR);
	//BSP_IntEn(EXTI15_10_IRQChannel);


	OSTaskCreate((OS_TCB     *)&JodoTCB,
                 (CPU_CHAR   *)"Jodo Task Start",
                 (OS_TASK_PTR )JodoTask, 
                 (void       *)0,
                 (OS_PRIO     )4,
                 (CPU_STK    *)&JodoStk[0],
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK     )0,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

	OSTaskCreate((OS_TCB     *)&JodoLedOnTCB,
                 (CPU_CHAR   *)"Jodo led on",
                 (OS_TASK_PTR )JodoLedOn, 
                 (void       *)0,
                 (OS_PRIO     )5,
                 (CPU_STK    *)&JodoLedOnStk[0],
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK     )0,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
	

	BSP_LED_Off(0);
    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.           */
		//OSSemPend(&sem1, 1u, OS_OPT_PEND_BLOCKING, 0u, &err);
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == 1){
			sendDataUART1((CPU_INT16U)'T');
			dark = TRUE;
			//OSFlagPost(&MyEventFlag, DARK_TRUE, (OS_OPT)OS_OPT_POST_FLAG_SET, &err);
		}
		else {
			sendDataUART1((CPU_INT16U)'F');
			dark = FALSE;
			//OSFlagPost(&MyEventFlag, DARK_FALSE, (OS_OPT)OS_OPT_POST_FLAG_SET, &err);
		}
		OSFlagPend(&MyEventFlag, DARK_FALSE + DARK_TRUE, 0u, (OS_OPT)OS_OPT_PEND_FLAG_CONSUME+ OS_OPT_PEND_FLAG_SET_ANY , &ts, &err);

		
		if( dark == TRUE ){
			GPIO_SetBits(GPIOD, BSP_GPIOD_LED1);
		}
		else {
			GPIO_ResetBits(GPIOD, BSP_GPIOD_LED1);
		}
		//OSSemPost(&sem1, OS_OPT_PEND_BLOCKING, &err);
      /*  OSTimeDlyHMSM((CPU_INT16U)0, (CPU_INT16U)0, (CPU_INT16U)1, (CPU_INT32U)0,
                    OS_OPT_TIME_HMSM_STRICT, 
                      &err);*/
		//OSTaskSuspend((OS_TCB *)0, &err); 
    }
    

    
}


static	void	JodoTask	(void *p_arg)
{
  	OS_ERR err;
	CPU_TS ts;
    (void)p_arg;

	
  	while (DEF_TRUE) {
		LedToggle(3);
        sendDataUART1((CPU_INT16U)'q');
		OSFlagPost(&MyEventFlag, DARK_TRUE, (OS_OPT)OS_OPT_POST_FLAG_SET, &err);
		GPIO_ResetBits(GPIOD, BSP_GPIOD_LED1);
	//BSP_IntDisAll();
	//
	////	while(ADC_GetFlagStatus(ADC1, 0x2)==RESET);

	//OSSemPend(&sem1, 0u, OS_OPT_PEND_BLOCKING, 0u, &err);
	                    	//sendDataUART1((CPU_INT16U)'j');

		
		////USART_SendData(USART1, (CPU_INT16U)'i');
		//if(ADC_GetITStatus(ADC1, ADC_IT_EOC)!=RESET){
			//set light sensor value for print to LCD
			//val = ADC_GetConversionValue(ADC1);
			// TODO implement7
			// clear 'Read data register not empty' flag
			//ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
		//}
//
    	//SendInteger(val);
		//sendDataUART1((CPU_INT16U)'0'+GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0));

		//BSP_IntEn(ADC1_2_IRQChannel);S
		//OSTaskSuspend((OS_TCB *)0, &err);
		//OSTaskResume(&AppTaskStartTCB,
			//		 &err);
	//OSSemPost(&sem1, OS_OPT_PEND_BLOCKING, &err);

 /* OSTimeDlyHMSM((CPU_INT16U)0, (CPU_INT16U)0, (CPU_INT16U)2, (CPU_INT32U)0,
                   OS_OPT_TIME_HMSM_STRICT, 
                       &err);
	*/	
	}
}

static	void	JodoLedOn	(void *p_arg)
{
	OS_ERR err;
	CPU_TS ts;
    (void)p_arg;

	
  	while (DEF_TRUE) {
		//LedToggle(4);
        sendDataUART1((CPU_INT16U)'s');

		OSFlagPost(&MyEventFlag, DARK_FALSE, (OS_OPT)OS_OPT_POST_FLAG_SET, &err);
		GPIO_ResetBits(GPIOD, BSP_GPIOD_LED1);
	}
}
void sendDataUART1(CPU_INT16U data) {
	while ((USART1->SR & USART_SR_TC) == 0);
	USART_SendData(USART1, data);
	
	
} 

static void SendData(CPU_INT16U data)
{
    USART1->DR = data;
   /* Wait till TC is set */
   while ((USART1->SR & USART_SR_TC) == 0);    
}
static void SendInteger(CPU_INT16U data){
  SendData(' ');
  while(data > 0){
    SendData( (data%10)+0x30 );
    data /= 10;
  }
                      	sendDataUART1((CPU_INT16U)'s');

  SendData(' ');
}

 void JodoISR(){
 	CPU_SR_ALLOC();
  	
	OS_CRITICAL_ENTER();  
  
	OSIntEnter();

  	OS_CRITICAL_EXIT();
	
	jodo = TRUE;
    sendDataUART1((CPU_INT16U)'i');

	OSIntExit ();

	/*if(ADC_GetITStatus(ADC1, ADC_IT_EOC)!=RESET){
			//set light sensor value for print to LCD
			val = ADC_GetConversionValue(ADC1);
			// TODO implement7
			// clear 'Read data register not empty' flag
			ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
		}
	    	SendInteger(val);
                    	sendDataUART1((CPU_INT16U)'i');*/
	//OSSemPend(&sem1, 0u, OS_OPT_PEND_BLOCKING, 0u, &err);

  /* Disable interrupts. */
	//OSSemPost(&sem1, OS_OPT_PEND_BLOCKING, &err);
} 

void USART_Configure() {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 14400;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

void	AdcInit(void){
	ADC_InitTypeDef ADC_InitStructure;
	
	    // ADC1 Configuration
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // interrupt enable
    ADC_Cmd(ADC1, ENABLE); // ADC1 enable
    ADC_ResetCalibration(ADC1);
   

    while(ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);

    while(ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void NVIC_Configure(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitTypeDef CStructure;

	/*NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel ;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_Init(&NVIC_InitStructure);
*/
	/*CStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
	CStructure.NVIC_IRQChannelCmd = ENABLE;
	CStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	CStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_Init(&CStructure);*/
}

void EXTI_Configure(void){
	EXTI_InitTypeDef EXTI_InitSturcture;
	EXTI_InitTypeDef ESturcture;

	/*GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource11);
	EXTI_InitSturcture.EXTI_Line = EXTI_Line11;
	EXTI_InitSturcture.EXTI_LineCmd = ENABLE;
	EXTI_InitSturcture.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitSturcture.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitSturcture);
 */
/*	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);
	ESturcture.EXTI_Line = EXTI_Line0;
	ESturcture.EXTI_LineCmd = ENABLE;
	ESturcture.EXTI_Mode = EXTI_Mode_Interrupt;
	ESturcture.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&ESturcture);*/
}

void	GPIO_Configure(void){
	GPIO_InitTypeDef GPIO_Led;
	GPIO_Led.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
	GPIO_Led.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Led.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_Led);

	GPIO_InitTypeDef GPIO_Botton;
	GPIO_Botton.GPIO_Pin = GPIO_Pin_11;
	GPIO_Botton.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Botton.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_Botton);
	
	GPIO_InitTypeDef GPIO_Jodo;
	GPIO_Jodo.GPIO_Pin = GPIO_Pin_0;
	GPIO_Jodo.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Jodo.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_Jodo);
	
}

void	RCC_Configure(void){
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
}
