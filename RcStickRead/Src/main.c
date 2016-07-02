/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "data_type.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define RECEIVE_OUTTIME 100
#define TRANSMIT_OUTTIME 100

/* ADC定义 */
#define ADCCONVERTEDVALUES_BUFFER_SIZE 4
#define ADC_Uref    3.3

/* 摇杆定义 */
#define   RC_STICK_ROLL_U_MAX   2.70
#define   RC_STICK_ROLL_U_MID   1.72
#define   RC_STICK_ROLL_U_MIN   0.74

#define   RC_STICK_PITCH_U_MAX   2.67
#define   RC_STICK_PITCH_U_MID   1.63
#define   RC_STICK_PITCH_U_MIN   0.54

#define   RC_STICK_THROTTLE_U_MAX   2.70
#define   RC_STICK_THROTTLE_U_MID   0.72
#define   RC_STICK_THROTTLE_U_MIN   0.00

/* 定义最大仿真范围 */
#define RADIUS_MAX    300
#define HIGHT_MAX    500

/* 方向偏移量设定 */
#define   X_OFFSET  80
#define   Y_OFFSET  10
#define   Z_OFFSET  -80

/* ADC采集值保存 */
uint32_t adc_value[ADCCONVERTEDVALUES_BUFFER_SIZE];

/* 低报警电压 */
#define V_BAT_LOW 3.4

/* 延时时间，可通过串口更新 */
uint8_t   gDelayTime=10;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*************************************************************
* @brief 开启/关闭蜂鸣器 
* @param value: 音量等级 0-1000
* @retval none
**************************************************************/
void BuzzerControl(uint16_t value)
{
  if( value)
  {
    /* Timer Output Compare Configuration Structure declaration */
    TIM_OC_InitTypeDef sConfig;
      
    sConfig.OCMode       = TIM_OCMODE_PWM1;
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

    /* Set the pulse value for channel 1 */
    sConfig.Pulse = value;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfig, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
    
    /* Start channel 1 */
    if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }
  }
  else
  {
        /* Stop channel 1 */
    if(HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

  }
}



/*************************************************************
* @brief 开启/关闭LED等
* @param value: 0-1000，调节LED 亮度
* @retval none
**************************************************************/
void LEDControl(uint16_t value)
{
  if( value)
  {
    /* Timer Output Compare Configuration Structure declaration */
    TIM_OC_InitTypeDef sConfig;
      
    sConfig.OCMode       = TIM_OCMODE_PWM1;
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

    /* Set the pulse value for channel 1 */
    sConfig.Pulse = value;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
    
    /* Start channel 1 */
    if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }
  }
  else
  {
        /* Stop channel 1 */
    if(HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

  }
}



/*************************************************************
* @brief BSP_USART_SendData 串口发送字符
* @param ch:待发送字符
* @arg
* @retval 发送成功标志位
**************************************************************/
HAL_StatusTypeDef BSP_USART_SendData(UART_HandleTypeDef *huart, uint8_t ch)
{
 
  /*等待上次发送完成*/
  while( (HAL_UART_STATE_BUSY_TX == HAL_UART_GetState(huart) )||(HAL_UART_STATE_BUSY_TX_RX == HAL_UART_GetState(huart)) );  

  /*开始发送数据-阻塞方式*/
  if( HAL_UART_Transmit(huart, &ch, 1,TRANSMIT_OUTTIME) == HAL_UART_STATE_TIMEOUT )
    return HAL_TIMEOUT; /*发送超时*/

  return HAL_OK;
}


/*************************************************************
* @brief BSP_USART_SendArray 串口发送字符串
* @param ptr:待发送数组，size：发送的字节数
* @arg
* @retval 发送状态标识
**************************************************************/
HAL_StatusTypeDef BSP_USART_SendArray(UART_HandleTypeDef *huart,  uint8_t* ptr,uint16_t size)
{
  if(NULL == ptr)
    return HAL_ERROR;
  

  if(size)
  {
    /*等待上次发送完成*/
    while( (HAL_UART_STATE_BUSY_TX == HAL_UART_GetState(huart) )||(HAL_UART_STATE_BUSY_TX_RX == HAL_UART_GetState(huart)) );  

    
    /*开始发送数据-阻塞方式*/
    if( HAL_UART_Transmit(huart, ptr, size,TRANSMIT_OUTTIME) == HAL_UART_STATE_TIMEOUT )
      return HAL_TIMEOUT; /*发送超时*/
  
  }

  return HAL_OK;  
}



/*************************************************************
* @brief BSP_USART_SendData 串口接收一个字符
* @param 
* @arg
* @retval ch: 返回接收到的字符
**************************************************************/
uint8_t BSP_USART_ReceiveData(UART_HandleTypeDef *huart )
{
  uint8_t ch;
 
  /*等待上次接收完成*/
  while( (HAL_UART_STATE_BUSY_RX == HAL_UART_GetState(huart) )||(HAL_UART_STATE_BUSY_TX_RX  == HAL_UART_GetState(huart)) );  

  /*开始接收数据*/
  if( HAL_UART_STATE_TIMEOUT  ==  HAL_UART_Receive(huart, &ch, 1,RECEIVE_OUTTIME) )
    return 0;//HAL_TIMEOUT; /*接收超时，返回0*/
 
  return ch;
}



/*************************************************************
* @brief BSP_USART_ReceiveArray 串口发送字符
* @param huart：句柄，ptr:待存放数组，size：发送的字节数
* @arg
* @retval 接收状态标识
**************************************************************/
HAL_StatusTypeDef BSP_USART_ReceiveArray(UART_HandleTypeDef *huart,  uint8_t* ptr,uint16_t size)
{
  if(NULL == ptr)
    return HAL_ERROR;
  
  if(size)
  {
    /*等待上次接收完成*/
    while( (HAL_UART_STATE_BUSY_RX == HAL_UART_GetState(huart) )||(HAL_UART_STATE_BUSY_TX_RX  == HAL_UART_GetState(huart)) );  

    /*开始接收数据*/
    if(HAL_UART_STATE_TIMEOUT  ==  HAL_UART_Receive(huart, ptr, size ,RECEIVE_OUTTIME) )
      return HAL_TIMEOUT;//; /*接收超时，返回0*/
    
  }

  return HAL_OK;  
}



/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
 
  /* 将Printf内容发往串口 */
  BSP_USART_SendData( &huart1, ch );

	return (ch);
}


/**
  * @brief  对数据进行校验，获取校验值
  * @param  ptr: 待校验数据指针
  * @param  len: 待校验数据长度
  * 
  * @retval uint8_t: 校验值
  */
uint8_t doSumCheck(uint8_t* ptr, uint8_t len )
{
  uint8_t sum;
  uint8_t i;
  
  sum = 0;
  for ( i=0;i<len;i++ )
  {
    sum += *ptr++;
  }
  
  return sum;
}


/**
  * @brief  将数据 SensorData_t 发送出去
  * @param  SensorData:  待发送的数据内容
  * 
  * @retval 无（死循环）
  */
void DataParse_PutData(SensorData_t* SensorData)
{
  Protocol_SensorPack_t Protocol_SensorPack;
  
  /* 填写帧文件头 */
  Protocol_SensorPack.header0 = SENSOR_PROTOCOL_PACK_HEADER0;
  Protocol_SensorPack.header1 = SENSOR_PROTOCOL_PACK_HEADER1;
  
  /* 填写帧数据区长度字段*/
  Protocol_SensorPack.length      = SENSOR_PROTOCOL_PACK_MAX_DATA_LEN ;
  
  /* 将要发送的数据复制到数据区 */
  memcpy( &Protocol_SensorPack.sensor_data, SensorData, Protocol_SensorPack.length );
  
  /* 计算并填写校验字 */
  Protocol_SensorPack.sum = doSumCheck( (uint8_t*)&Protocol_SensorPack.sensor_data, Protocol_SensorPack.length );
  
  
  /* 数据发送 */
  BSP_USART_SendArray( &huart2,  (uint8_t*)&Protocol_SensorPack, SENSOR_PROTOCOL_PACK_MAX_SIZE );
  
}




/*显示信息*/
void  LCD_DispData( SensorData_t* SensorData)
{
  BACK_COLOR  = LCD_COLOR_WHITE;
  POINT_COLOR = LCD_COLOR_DARKBLUE;	

  LCD_ShowString(10,70,"X:");
  LCD_ShowString(10,90,"Y:");
  LCD_ShowString(10,110,"Z:");

  if( SensorData->x < 0 )
  {
    LCD_ShowChar(40,70,'-',0); 
    SensorData->x = -SensorData->x;
  }else
  {
    LCD_ShowChar(40,70,' ',0);
  }
  
  if( SensorData->y < 0 )
  {
    LCD_ShowChar(40,90,'-',0);
    SensorData->y = -SensorData->y;
  }
  else
  {
    LCD_ShowChar(40,90,' ',0);
  }
  
  if( SensorData->z < 0 )  
  {
    LCD_ShowChar(40,110,'-',0);
    SensorData->z = -SensorData->z;
  }
  else 
  {
    LCD_ShowChar(40,110,' ',0);
  }
  LCD_ShowNum(50, 70, SensorData->x ,3 );
  LCD_ShowNum(50, 90, SensorData->y ,3 );
  LCD_ShowNum(50, 110, SensorData->z ,3 );
}

/*  */

/*************************************************************
* @brief 读取摇杆状态，输出模拟坐标值 
* @param pSensorData: 数据保存位置
* @retval float 返回电池电压值，如果该值不为0，可以证明本次执行正常
**************************************************************/
float  ReadRcStick( SensorData_t* pSensorData )
{
    uint32_t adc_value_roll;
    uint32_t adc_value_pitch;
    uint32_t adc_value_throttle;
    uint32_t adc_value_vbat;
    
    float u_roll;
    float u_pitch;
    float u_throttle;
    float u_vbat;

      /* 连续采集 NUM_SAMPLES 次数据，计算平均值 */
#define NUM_SAMPLES   10
    adc_value_roll     = 0;
    adc_value_pitch    = 0;
    adc_value_throttle = 0;
    adc_value_vbat = 0;
  
    uint16_t i;
    for(i=0;i<NUM_SAMPLES;i++)
    {
      /* 读ADC值 */
      HAL_ADC_Start_DMA( &hadc1,adc_value,ADCCONVERTEDVALUES_BUFFER_SIZE);
      adc_value_roll     += adc_value[0];
      adc_value_pitch    += adc_value[1];
      adc_value_throttle += adc_value[2];
      adc_value_vbat += adc_value[3];
//      HAL_Delay(1);
    }
    adc_value_roll     /= NUM_SAMPLES;
    adc_value_pitch    /= NUM_SAMPLES;
    adc_value_throttle /= NUM_SAMPLES;
    adc_value_vbat     /= NUM_SAMPLES;

  
    /* 数据转换 */
    u_roll      = ( (adc_value_roll*ADC_Uref/4096    ) ) ;
    u_pitch     = ( (adc_value_pitch*ADC_Uref/4096   ) ) ;
    u_throttle  = ( (adc_value_throttle*ADC_Uref/4096) ) ;    //0.45为固定偏移量
    u_vbat  = ( 2*(adc_value_vbat*ADC_Uref/4096) ) ;          //采集前经过分压
    
    /* 打印结果 */
//    printf("AdcData=(%f,%f,%f)\n", u_roll, u_pitch, u_throttle);
    
    /* 数据转换 */
    if( u_roll > RC_STICK_ROLL_U_MID) 
    {
      pSensorData->x  = (int16_t)(RADIUS_MAX*(u_roll    -  RC_STICK_ROLL_U_MID    )/( RC_STICK_ROLL_U_MAX  -  RC_STICK_ROLL_U_MID      ) );
    }else
    {
      pSensorData->x  = (int16_t)(RADIUS_MAX*(u_roll    -  RC_STICK_ROLL_U_MID    )/( RC_STICK_ROLL_U_MID  - RC_STICK_ROLL_U_MIN       ) );
    }
    
    if( u_pitch > RC_STICK_PITCH_U_MID) 
    {
      pSensorData->y  = (int16_t)(RADIUS_MAX*(u_pitch   -  RC_STICK_PITCH_U_MID    )/( RC_STICK_PITCH_U_MAX  -  RC_STICK_PITCH_U_MID  ) );
    }else
    {
      pSensorData->y  = (int16_t)(RADIUS_MAX*(u_pitch   -  RC_STICK_PITCH_U_MID    )/( RC_STICK_PITCH_U_MID  - RC_STICK_PITCH_U_MIN    ) );
    }
    
    if( u_throttle > RC_STICK_THROTTLE_U_MIN) 
    {
      pSensorData->z  = (int16_t)( HIGHT_MAX*(u_throttle )/( RC_STICK_THROTTLE_U_MAX ) );
    }
    
    
    /* TODO::根据设定的平移量调整 */
    pSensorData->x += X_OFFSET ;
    pSensorData->y += Y_OFFSET ;
    pSensorData->z += Z_OFFSET ;
    
    return u_vbat;
}

/* 上电检查摇杆状态，如果摇杆默认偏移较大，不予启动 */
void CheckRcStick(SensorData_t* pSensorData)
{
  uint16_t counter;
  uint8_t flg;
  
  counter = 0;
  flg = 0;
  while(1)
  {
    /* 读ADC值 */
    ReadRcStick( pSensorData );
    if( 0==counter%500 )
    {
      LCD_DispData( pSensorData);
    }
    else  if( 0==counter%800 ) 
    {
      if(flg)
      {
        POINT_COLOR = LCD_COLOR_RED;
        LCD_ShowString(0,50,"Adjust Stick!!");
        flg = 0;
      }else
      {
        POINT_COLOR = LCD_COLOR_RED;
        LCD_ShowString(0,50,"              ");
        flg = 1;
      }
      
    }

    /* 判断输出位置是否合适 */
    if( (pSensorData->z < 150) || (pSensorData->z > HIGHT_MAX) )
    {
      LEDControl( ((counter%500)/250)*10 );
      BuzzerControl(counter%999+1);
    }else if( ( pSensorData->y > 50)||( pSensorData->y <(-50)))
    {
      LEDControl( ((counter%500)/250)*10 );
      BuzzerControl(counter%999+1);
    }else if( (pSensorData->x > 50)||(pSensorData->x <-50))
    {
      LEDControl( ((counter%500)/250)*10 );
      BuzzerControl(counter%999+1);
    }
    else
    {
      LEDControl( 0 );
      BuzzerControl(0);
      
      POINT_COLOR = LCD_COLOR_RED;
      LCD_ShowString(0,50,"              ");
      break;
    }


    counter++;

  }
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  SensorData_t last_SensorData;
  SensorData_t SensorData;
  uint32_t tickstart = 0;
  uint32_t tickStickChanged = 0;
  uint8_t  StateIndicateFlag;
  float vbat;               /* 电池电压 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1,&gDelayTime,1);
  
  
  Lcd_Init();   //tft初始化
  LCD_Clear( LCD_COLOR_WHITE ); //清屏
  BACK_COLOR  = LCD_COLOR_BLACK;
  POINT_COLOR = LCD_COLOR_WHITE; 
  LCD_ShowString(8,0,"COORDINATE");
  LCD_ShowString(40,16,"SIMULATION");
  BACK_COLOR  = LCD_COLOR_WHITE;
  POINT_COLOR = LCD_COLOR_BLACK; 
  LCD_ShowString(0,32,"===============");


  /* 检查摇杆范围 */
  CheckRcStick( &SensorData );
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
 
    
    /* 读ADC值 */
    vbat = ReadRcStick( &SensorData );

    
    /* TODO::此处仅为了测试处理 */
    SensorData.x = -SensorData.x;
    SensorData.y = -SensorData.y;
    SensorData.z =  SensorData.z;
    
    /* 发送数据 */
    DataParse_PutData(&SensorData);
    printf("SensorData=(%d,%d,%d)\n",SensorData.x,SensorData.y,SensorData.z);
    LCD_DispData( &SensorData);
    
    /* 摇杆相对位移动作较小，超过60s，自动关闭显示器 */
    if( (HAL_GetTick() - tickStickChanged) > 60000 )
    {    
      LCD_ControlLED(0);      
    }
    
    if( ((SensorData.x*SensorData.x + SensorData.y*SensorData.y ) \
      - (last_SensorData.x*last_SensorData.x + last_SensorData.y*last_SensorData.y )) > 80 )
    {
      tickStickChanged = HAL_GetTick();
      LCD_ControlLED(1); 
    }
    
    /* 保存本次测量结果 */
    memcpy( &last_SensorData,&SensorData, sizeof(SensorData_t) );
    
    
    /* 运行状态指示 */
    if( (HAL_GetTick() - tickstart) > 2500 )
    {   
      tickstart = HAL_GetTick();
      
      /* 检查系统电量 */
      if( vbat < V_BAT_LOW )
      {
        
        if(StateIndicateFlag)
        {
          LEDControl( 20 );
          if( vbat < 3.32) BuzzerControl(400);
          POINT_COLOR = LCD_COLOR_RED;
          LCD_ShowString(0,50,"Low Battery!!");
          
        }else
        {
          LEDControl( 0 );
          if( vbat < 3.4) BuzzerControl(0);
          POINT_COLOR = LCD_COLOR_RED;
          LCD_ShowString(0,50,"             ");
        }
      }
      else
      {
        LEDControl( 10 );
        BuzzerControl(0);
        LCD_ShowString(0,50,"             ");
        
      }
      
      StateIndicateFlag = !StateIndicateFlag;
    }

    /* 延时控制 */
    HAL_Delay( gDelayTime*10 );
    
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 300;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_AO_GPIO_Port, LCD_AO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RESET_Pin|LCD_CS_Pin|LCD_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LCD_AO_Pin LCD_RESET_Pin LCD_LED_Pin */
  GPIO_InitStruct.Pin = LCD_AO_Pin|LCD_RESET_Pin|LCD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  接收完成回调函数
  * @param  huart: 指向串口实例.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(  USART1 == huart->Instance )
  {
      HAL_UART_Receive_IT(&huart1,&gDelayTime,1);
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
