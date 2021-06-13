
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

//#define MIN_RED 35000.0
//#define MAX_RED 80000.0
//#define MIN_GREEN 1000.0
//#define MAX_GREEN 4000.0
//#define MIN_BLUE 1000.0
//#define MAX_BLUE 4000.0
#define MIN_YELLOW 350
#define MAX_YELLOW 800

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
   Captura_TCS3200_Init();
  
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

   
   Set_Filter(Clear);
   Set_Scaling(Scl20);
   
   int colorRed = 0;
   int colorGreen = 0;
   int colorBlue = 0;

   GPIO_SetBits(GPIOB, GPIO_Pin_5);
   
   char buffer[80] = {'/0'};
   while (DEF_TRUE) {
      
      USART_SendData(USART1, 'T');
      colorRed = GetColor(Red);
//      colorGreen = GetColor(Green);
//      colorBlue = GetColor(Blue);
      
      if(VIRTUALIZAR_SENSOR){
         colorRed = colorRed + 10;
         colorGreen = colorGreen + 5;
         colorBlue = colorBlue + 15;
         
         if(colorRed >= 255) colorRed = 0;
         if(colorGreen >= 255) colorGreen = 0;
         if(colorBlue >= 255) colorBlue = 0;
      }
      
      sprintf(buffer, "Color: %d\r\n", colorRed);
      SendBuffer(buffer);
      
//      sprintf(buffer, "Green: %d\r\n", colorGreen);
//      SendBuffer(buffer);
//      
//      sprintf(buffer, "Blue: %d\r\n", colorBlue);
//      SendBuffer( buffer);
        
      OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT, &err);

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
}

void Captura_TCS3200_Init(void)
{   
   /* GPIOB clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   /*-------------------------- GPIO Configuration ----------------------------*/
   /* GPIOB Configuration: PB0 como entrada para captura */
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
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
int GetColor(int set_color) //Funcion que Devuelve RGB de color Rojo (0-255)
{
    OS_ERR err;
    USART_SendData(USART1, 'C');
    
    char buffer[80] = {'\0'};
//    sprintf(buffer, "-- Color Type: %d\r\n", set_color);
//    SendBuffer(buffer);

    CPU_INT32U TimeColor=0;       
 
   Set_Filter(Clear); //Set filter to Color
   Set_Filter(set_color); //Set filter to Color
 
   OSTimeDlyHMSM((CPU_INT16U)0, (CPU_INT16U)0, (CPU_INT16U)0, (CPU_INT32U)100,
                    OS_OPT_TIME_HMSM_STRICT, 
                      &err);  
 
    TimeColor = getFrequency();
    Set_Filter(Clear); //Set filter to default
    sprintf(buffer, "-- TimeColor: %d\r\n", TimeColor);
    SendBuffer(buffer);
   
//    FreqColor = SystemCoreClock/(TimeColor); //Frequency conversion by means of SystemCoreClock 
//    sprintf(buffer, "-- FreqColor: %d\r\n", FreqColor);
//    SendBuffer(buffer);
   
//    int Output_Color;
    //Freq to Color -> Depending of the filter
//    switch (set_color){
//      case Red:
//         Output_Color = (255.0/(MAX_RED-MIN_RED))*(FreqColor-MIN_RED); //MAPEAR FUNCION
//         break;
//      
//      case Green:
//         Output_Color = (255.0/(MAX_GREEN-MIN_GREEN))*(FreqColor-MIN_GREEN);  //MAPEAR FUNCION
//         break;
//      
//      case Blue: 
//         Output_Color = (255.0/(MAX_RED-MIN_BLUE))*(FreqColor-MIN_BLUE);  //MAPEAR FUNCION
//         break;
//    }

    //Constrain Value to MaxRange
//    if (Output_Color > 255) Output_Color = 255;
//    if (Output_Color < 0) Output_Color = 0;
    int Output_Color = 0;
    if(MIN_YELLOW < TimeColor && TimeColor < MAX_YELLOW) Output_Color = 1;
    else Output_Color = 0;
    
    return Output_Color   ; //Mapeo y retorno valor
}


CPU_INT32U getFrequency() {
    USART_SendData(USART1, 'F');
    OS_ERR err;
 
    CPU_INT32U l_time=0;
    CPU_INT32U h_time=0;
 // read the time for which the pin is high

   char buffer[80] = {'\0'};
   
   while ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)));
   while (!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)));
   while ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)))  // wait for the ECHO pin to go high
   {
     l_time++;
   }
   while (!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)))    // while the pin is high 
    {
      h_time++;

    }
        sprintf(buffer, "  -- l_time: %d\r\n", l_time);
        SendBuffer(buffer);
        sprintf(buffer, "  -- h_time: %d\r\n", h_time);
        SendBuffer(buffer);
   return h_time;
} 
