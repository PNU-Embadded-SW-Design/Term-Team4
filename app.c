
#include <includes.h>


#define      DARK_TRUE   (OS_FLAGS)0x0001
#define      DARK_FALSE   (OS_FLAGS)0x0002
#define      DATK_TRUE   (OS_FLAGS)0x0004

/**
   @file tcs3200.h
   @brief Libreria Sensor de color TCS3200 
   @author Jaime Laborda, Andres Reveron, Rafa Oriol
   @date 2017/01/13
*/
/*-------------------------------------------------------------------------------*/
/*----------------------LIBRERIA SENSOR COLOR TCS3200----------------------------*/
/*------------------------Proyecto Sistemas Embebidos----------------------------*/
/*-----------------Disenada y programada por Jaime Laborda-----------------------*/
/*------------------------------Diciembre de 2016--------------------------------*/
/*----------------------------------tcs3200.h------------------------------------*/
/*-------------------------------------------------------------------------------*/

#include <stdbool.h>
#define VIRTUALIZAR_SENSOR 0
#define SystemCoreClock 72000000
/*#define MIN_RED 6000.0
#define MAX_RED 35000.0
#define MIN_GREEN 10000.0
#define MAX_GREEN 56000.0
#define MIN_BLUE 10000.0
#define MAX_BLUE 35000.0*/

#define MIN_RED 2000.0
#define MAX_RED 12000.0
#define MIN_GREEN 2000.0
#define MAX_GREEN 12000.0
#define MIN_BLUE 2000.0
#define MAX_BLUE 12000.0

 _Bool IC_ColorMode=false;
u8 calibrate_number;

int FreqColor;

enum Colors{Red, Blue, Clear, Green};
enum Scaling{Scl0, Scl2, Scl20, Scl100};

//PUBLIC FUNCTIONS
void Captura_TCS3200_Init(void);
void TCS3200_Config(void);
void Set_Filter (u8 mode); //Mode es de tipo enum Color
void Set_Scaling (u8 mode); //Mode es de tipo enum Scaling
int GetColor(int set_color); //Funcion que Devulve RGB de color Rojo (0-255)

void TIM3_IRQHandler(void); //Manejador de interrupcion

CPU_INT32U getFrequency(void);

/*
*********************************************************************************************************
*                                             GLOBAL VARIABLES
*********************************************************************************************************
*/

static OS_SEM sem1;
static OS_SEM sem2;


/*
*********************************************************************************************************
*                                            LOCAL VARIABLES
*********************************************************************************************************
*/

static  OS_TCB   AppTaskStartTCB; 
static  CPU_STK  AppTaskStartStk[APP_TASK_START_STK_SIZE];

static    OS_TCB      ColorTCB;
static    CPU_STK   ColorStk[APP_TASK_START_STK_SIZE];

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart  (void *p_arg);

static   void   ColorTask   (void *p_arg);

static void SendData(CPU_INT16U data);
void SendBuffer(char *pucBuffer);
static void USART_Configure();
void   GPIO_Configure(void);
void   RCC_Configure(void);
void Timer_Configure(void);

int  main (void)
{
    
    OS_ERR      err;    
    BSP_IntDisAll();
    BSP_Init();                                                   /* Initialize BSP functions                         */
    CPU_Init();      
    USART_Configure();
    RCC_Configure();
    GPIO_Configure();
    //AdcInit();
   //BSP_IntVectSet(BSP_INT_ID_ADC1_2, JodoISR);
//  Timer_Configure();
  
   OSSemCreate(&sem1, "ADC_OS_SEM", 0, &err);
   
   OSInit(&err);                                               /* Init uC/OS-III.                                      */

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
//   BSP_IntVectSet(TIM3_IRQChannel, TIM3_IRQHandler);
//   BSP_IntEn(TIM3_IRQChannel);

   OSTaskCreate((OS_TCB     *)&ColorTCB,
                 (CPU_CHAR   *)"Jodo led off",
                 (OS_TASK_PTR )ColorTask, 
                 (void       *)0,
                 (OS_PRIO     )2,
                 (CPU_STK    *)&ColorStk[0],
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK     )0,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
   BSP_LED_Off(0);

    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.           */

        OSTimeDlyHMSM((CPU_INT16U)0, (CPU_INT16U)0, (CPU_INT16U)1, (CPU_INT32U)0,
                    OS_OPT_TIME_HMSM_STRICT, 
                      &err);
    }   

}

static   void   ColorTask   (void *p_arg)
{
   OS_ERR err;
   CPU_TS ts;
   (void)p_arg;

   Captura_TCS3200_Init();
   
   Set_Filter(Clear);
   Set_Scaling(Scl20);
   
   int colorRed = 0;
   int colorGreen = 0;
   int colorBlue = 0;

   char buffer[80] = {'/0'};
   while (DEF_TRUE) {
      
      USART_SendData(USART1, 'T');
      colorRed = GetColor(Red);
      colorGreen = GetColor(Green);
      colorBlue = GetColor(Blue);
      
      if(VIRTUALIZAR_SENSOR){
         colorRed = colorRed + 10;
         colorGreen = colorGreen + 5;
         colorBlue = colorBlue + 15;
         
         if(colorRed >= 255) colorRed = 0;
         if(colorGreen >= 255) colorGreen = 0;
         if(colorBlue >= 255) colorBlue = 0;
      }
      
      sprintf(buffer, "Red: %d\r\n", colorRed);
      SendBuffer(buffer);
      
      sprintf(buffer, "Green: %d\r\n", colorGreen);
      SendBuffer(buffer);
      
      sprintf(buffer, "Blue: %d\r\n", colorBlue);
      SendBuffer( buffer);
        
      OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT, (OS_ERR) 0);

   }
   
}

void SendBuffer(char *pucBuffer)
{
    while (*pucBuffer)
    {
        USART_SendData(USART1, *pucBuffer++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}
static void SendData(CPU_INT16U data)
{
    USART1->DR = data;
   /* Wait till TC is set */
   while ((USART1->SR & USART_SR_TC) == 0);    
}

void USART_Configure() {
   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

   GPIO_Init(GPIOA, &GPIO_InitStructure);

   USART_InitStructure.USART_BaudRate = 115200;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_Mode = USART_Mode_Tx;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

   USART_Init(USART1, &USART_InitStructure);
   USART_Cmd(USART1, ENABLE);
}

//void   AdcInit(void){
//   ADC_InitTypeDef ADC_InitStructure;
//   
//       // ADC1 Configuration
//    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
//    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
//    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//    ADC_InitStructure.ADC_NbrOfChannel = 1;
//    ADC_Init(ADC1, &ADC_InitStructure);
//
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
//    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // interrupt enable
//    ADC_Cmd(ADC1, ENABLE); // ADC1 enable
//    ADC_ResetCalibration(ADC1);
//   
//
//    while(ADC_GetResetCalibrationStatus(ADC1));
//
//    ADC_StartCalibration(ADC1);
//
//    while(ADC_GetCalibrationStatus(ADC1));
//
//    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//}

void   GPIO_Configure(void){
   GPIO_InitTypeDef GPIO_Color;
   GPIO_Color.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
   GPIO_Color.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Color.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOB, &GPIO_Color);
}

void   RCC_Configure(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}

void Timer_Configure(void){
  
   TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
   
   /*Activo Clock para el periferico del timer*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
   
   /*Configuro la base de tiempos del timer*/
   
   TIM_TimeBaseInitStructure.TIM_Prescaler = 7200; //0.0001s ////84; //Resolucion de 0.001ms = 1us
                  // TIM_Prescaler = 720, 72 로 했을 때 동작 멈춤
   TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInitStructure.TIM_Period = 5000;// 0.0001s * 10000 = 1s          
                  //period = 100, 1000일 땐 동작이 멈춤.
                  // 5000일 땐 잘 작동 (10000보다 주기 좀 더 빨라짐)
   TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //0;
   
   TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
   
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 500000; //us // 0.5s * 1000000 (TIM3의 주기인 0.5s와 맞춰줌)
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
   
   //CHANNEL 3 -> SUBIDA
   /* TIM Input Capture Init structure definition */
   TIM_ICInitTypeDef TIM_ICInitStructure;
   TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;  // 얘를 TIM_Channel_3 에서 TIM_Channel_1 으로 바꿔주니까 TIM가 잘 동작함.
   TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
   TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
   TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
   TIM_ICInitStructure.TIM_ICFilter = 0;
   
   TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
   
   
  /* TIM_IT_CC3: TIM interrupt sources ---------------------------------------------------*/
   //Configuro interrupcion en el TIM3 CC3
   TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
   
   TIM_Cmd(TIM3, ENABLE);   
   
   NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   //Configurar interrupcion del Channel 4 (BAJADA) del TIM3 -> NVIC
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;//TIM3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
            //// NVIC_IRQChannelPreemptionPriority을 0에서 2로 조정 -> '1', '5'의 출력이 좀 더 주기에 맞게 고르게 출력이 됨??
            //// 근데 2 -> 3으로 조정하니까 또 '1', '5'의 출력 주기가 엉망이 됨????
            // Timer IRQ Handler에서 if(TIM_GetITStatus(..) != RESET) {} 문을 추가해주고
            // 이 값을 0으로 수정 -> 잘 동작함.
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   
   NVIC_Init(&NVIC_InitStructure);
}
void Captura_TCS3200_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   
   /* GPIOB clock enable */
//   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   /*-------------------------- GPIO Configuration ----------------------------*/
   /* GPIOB Configuration: PB0 como entrada para captura */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   /* Connect TIM4 pins to AF2 */
//   GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3); //TIM3 CC3 -> PB0

}
void Set_Filter (u8 mode) //Mode es de tipo enum Filtro
{
   switch (mode){
      case(Red):
         GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4);
         break;
      case(Blue):
         GPIO_ResetBits(GPIOB, GPIO_Pin_3);
         GPIO_SetBits(GPIOB, GPIO_Pin_4);
         break;
      case(Clear):
         GPIO_ResetBits(GPIOB, GPIO_Pin_4);
         GPIO_SetBits(GPIOB, GPIO_Pin_3);
         break;
      case(Green):
         GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4);
         break;
   }
}

void Set_Scaling (u8 mode) //Mode es de tipo enum Filtro
{
   switch (mode){
      case(Scl0):
         GPIO_ResetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_2);
         break;
      case(Scl2):
         GPIO_ResetBits(GPIOB, GPIO_Pin_1);
         GPIO_SetBits(GPIOB, GPIO_Pin_2);
         break;
      case(Scl20):
         GPIO_ResetBits(GPIOB, GPIO_Pin_2);
         GPIO_SetBits(GPIOB, GPIO_Pin_1);
         break;
      case(Scl100):
         GPIO_SetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_2);
         break;
   }
}
   int Output_Color;
int GetColor(int set_color) //Funcion que Devuelve RGB de color Rojo (0-255)
{
  USART_SendData(USART1, 'C');
   //char Output_Color;
   
   calibrate_number=0;


  CPU_INT32U TimeColor=0;   

   Set_Filter(set_color); //Set filter to Color

   TimeColor = getFrequency();
   Set_Filter(Clear); //Set filter to default
   char buffer[80] = {'\0'};
      sprintf(buffer, "-- TimeColor: %d\r\n", TimeColor);
      SendBuffer(buffer);
   
   FreqColor = SystemCoreClock/(TimeColor*84); //Frequency conversion by means of SystemCoreClock 
//   FreqColor= BSP_CPU_ClkFreq()/(TimeColor*168);
      sprintf(buffer, "-- FreqColor: %d\r\n", FreqColor);
      SendBuffer(buffer);
   
   //Freq to Color -> Depending of the filter
   switch (set_color){
      case Red:
         Output_Color = (255.0/(MAX_RED-MIN_RED))*(FreqColor-MIN_RED); //MAPEAR FUNCION
         break;
      
      case Green:
         Output_Color = (255.0/(MAX_GREEN-MIN_GREEN))*(FreqColor-MIN_GREEN);  //MAPEAR FUNCION
         break;
      
      case Blue: 
         Output_Color = (255.0/(MAX_RED-MIN_BLUE))*(FreqColor-MIN_BLUE);  //MAPEAR FUNCION
         break;
   }
   
   //Constrain Value to MaxRange
   if (Output_Color > 255) Output_Color = 255;
   if (Output_Color < 0) Output_Color = 0;
//   
   return Output_Color   ; //Mapeo y retorno valor
}


CPU_INT32U getFrequency() {
  USART_SendData(USART1, 'F');
OS_ERR err;
 
  CPU_INT32U TimeColor_H=0;
  CPU_INT32U TimeColor_L=0;
  CPU_INT32U local_time=0;
 // read the time for which the pin is high

 OSTimeDlyHMSM((CPU_INT16U)0, (CPU_INT16U)0, (CPU_INT16U)0, (CPU_INT32U)100,
                    OS_OPT_TIME_HMSM_STRICT, 
                      &err);  
 while (!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)));  // wait for the ECHO pin to go high
 
 while ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)))    // while the pin is high 
  {
  USART_SendData(USART1, 'h');
  local_time++;   // measure time for which the pin is high
// OSTimeDlyHMSM((CPU_INT16U)0, (CPU_INT16U)0, (CPU_INT16U)0, (CPU_INT32U)0.001,
//                    OS_OPT_TIME_HMSM_STRICT, 
//                      &err);  
  }
 return local_time;
} 
