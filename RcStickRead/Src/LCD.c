#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "lcd.h"
#include "font.h"

uint16_t BACK_COLOR, POINT_COLOR;   //����ɫ������ɫ	  


#define  StartDataWriting()   HAL_GPIO_WritePin(LCD_AO_GPIO_Port,LCD_AO_Pin,GPIO_PIN_SET)  
#define  StartRegWriting()   HAL_GPIO_WritePin(LCD_AO_GPIO_Port,LCD_AO_Pin,GPIO_PIN_RESET) 

#define  LCD_CS_HIGH()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_SET)  
#define  LCD_CS_LOW()    HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_RESET)  

#define  LCD_RST_RESET()      HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_RESET)  
#define  LCD_RST_RELEASE()    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_SET)  

#define  LCD_LED_ON()      HAL_GPIO_WritePin(LCD_LED_GPIO_Port,LCD_LED_Pin,GPIO_PIN_SET)  
#define  LCD_LED_OFF()    HAL_GPIO_WritePin(LCD_LED_GPIO_Port,LCD_LED_Pin,GPIO_PIN_RESET)  

/*****************************���غ���******************************************/

/**
  * @brief  SPI ��������ȡ 4 bytes
  * @retval ����������
*/
//static uint32_t SPIx_Read(void)
//{
//  uint32_t readvalue = 0;
//  uint32_t writevalue = 0xFFFFFFFF;
//  
//  HAL_SPI_TransmitReceive(&BSP_SPI_HANDLE, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 1, SPIx_TIMEOUT_MAX);

//  return readvalue;
//}



/**
  * @brief  SPI д1byte������
  * @param  Value: д���ֵ
  * @retval ��
  */
static void SPIx_Write(uint8_t Value)
{

  HAL_SPI_Transmit(&BSP_SPI_HANDLE, (uint8_t*) &Value, 1, SPIx_TIMEOUT_MAX);

}
 

static void LCD_Writ_Bus(char value)   //��������д��
{	
  SPIx_Write( value );
} 




static void LCD_WR_DATA8(char da) //��������-8λ����
{
  StartDataWriting();
	LCD_Writ_Bus(da);
}  



static void LCD_WR_DATA(uint16_t da)
{
  StartDataWriting();
	LCD_Writ_Bus(da>>8);
	LCD_Writ_Bus(da);
}	  


static void LCD_WR_REG(uint8_t da)	 
{
  StartRegWriting();
	LCD_Writ_Bus(da);
}


//static void LCD_WR_REG_DATA(uint8_t reg,uint16_t da)
//{
//  LCD_WR_REG(reg);
//	LCD_WR_DATA(da);
//}



/*****************************��������******************************************/
void Address_set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{ 
  LCD_WR_REG(0x2a);
  LCD_WR_DATA8(x1>>8);
  LCD_WR_DATA8(x1);
  LCD_WR_DATA8(x2>>8);
  LCD_WR_DATA8(x2);

  LCD_WR_REG(0x2b);
  LCD_WR_DATA8(y1>>8);
  LCD_WR_DATA8(y1);
  LCD_WR_DATA8(y2>>8);
  LCD_WR_DATA8(y2);

  LCD_WR_REG(0x2C);
  
}


/* LCD������� */
void LCD_ControlLED(uint8_t state)
{
  if( state )
    LCD_LED_ON() ;
  else
    LCD_LED_OFF() ;
}




void Lcd_Init(void)
{

  LCD_CS_LOW();
  LCD_RST_RESET();
  HAL_Delay(100);	
	LCD_RST_RELEASE() ;
  LCD_CS_HIGH();
  
  HAL_Delay(100);	
  
  
	/* ��Ƭѡʹ�� */
  LCD_CS_LOW();
  
  /* ��ʼ���Ĵ����б�  */
	LCD_WR_REG(0x11); //Sleep out
  HAL_Delay(120); //Delay 120ms
  
	//------------------------------------ST7735S Frame Rate-----------------------------------------//
	LCD_WR_REG(0xB1);
	LCD_WR_DATA8(0x05);


  LCD_WR_REG(0x11);//Sleep exit 
  HAL_Delay(120);

  //ST7735R Frame Rate
  LCD_WR_REG(0xB1); 
  LCD_WR_DATA8(0x01); 

  LCD_WR_DATA8(0x2C); LCD_WR_DATA8(0x2D); 
  LCD_WR_REG(0xB2); 
  LCD_WR_DATA8(0x01); LCD_WR_DATA8(0x2C); LCD_WR_DATA8(0x2D); 
  LCD_WR_REG(0xB3); 
  LCD_WR_DATA8(0x01); LCD_WR_DATA8(0x2C); LCD_WR_DATA8(0x2D); 
  LCD_WR_DATA8(0x01); LCD_WR_DATA8(0x2C); LCD_WR_DATA8(0x2D); 

  LCD_WR_REG(0xB4); //Column inversion 
  LCD_WR_DATA8(0x07); 

  //ST7735R Power Sequence
  LCD_WR_REG(0xC0); 
  LCD_WR_DATA8(0xA2); LCD_WR_DATA8(0x02); LCD_WR_DATA8(0x84); 
  
  LCD_WR_REG(0xC1); LCD_WR_DATA8(0xC5); 
  
  LCD_WR_REG(0xC2); 
  LCD_WR_DATA8(0x0A); LCD_WR_DATA8(0x00); 
  LCD_WR_REG(0xC3); 
  LCD_WR_DATA8(0x8A); LCD_WR_DATA8(0x2A); 
  LCD_WR_REG(0xC4); 
  LCD_WR_DATA8(0x8A); LCD_WR_DATA8(0xEE); 

  LCD_WR_REG(0xC5); //VCOM 
  LCD_WR_DATA8(0x0E); 

  LCD_WR_REG(0x36); //MX, MY, RGB mode 
  LCD_WR_DATA8(0x68);     //0xC8-����,0x68����

  //ST7735R Gamma Sequence
  LCD_WR_REG(0xe0); 
  LCD_WR_DATA8(0x0f); LCD_WR_DATA8(0x1a); 
  LCD_WR_DATA8(0x0f); LCD_WR_DATA8(0x18); 
  LCD_WR_DATA8(0x2f); LCD_WR_DATA8(0x28); 
  LCD_WR_DATA8(0x20); LCD_WR_DATA8(0x22); 
  LCD_WR_DATA8(0x1f); LCD_WR_DATA8(0x1b); 
  LCD_WR_DATA8(0x23); LCD_WR_DATA8(0x37); LCD_WR_DATA8(0x00); 
  LCD_WR_DATA8(0x07); 
  LCD_WR_DATA8(0x02); LCD_WR_DATA8(0x10); 
  
  LCD_WR_REG(0xe1); 
  LCD_WR_DATA8(0x0f); LCD_WR_DATA8(0x1b); 
  LCD_WR_DATA8(0x0f); LCD_WR_DATA8(0x17); 
  LCD_WR_DATA8(0x33); LCD_WR_DATA8(0x2c); 
  LCD_WR_DATA8(0x29); LCD_WR_DATA8(0x2e); 
  LCD_WR_DATA8(0x30); LCD_WR_DATA8(0x30); 
  LCD_WR_DATA8(0x39); LCD_WR_DATA8(0x3f); 
  LCD_WR_DATA8(0x00); LCD_WR_DATA8(0x07); 
  LCD_WR_DATA8(0x03); LCD_WR_DATA8(0x10);  

  LCD_WR_REG(0x2a);
  LCD_WR_DATA8(0x00);LCD_WR_DATA8(0x00);
  LCD_WR_DATA8(0x00);LCD_WR_DATA8(0x7f);
  
  LCD_WR_REG(0x2b);
  LCD_WR_DATA8(0x00);LCD_WR_DATA8(0x00);
  LCD_WR_DATA8(0x00);LCD_WR_DATA8(0x7f);

  LCD_WR_REG(0xF0); //Enable test command  
  LCD_WR_DATA8(0x01); 
  
  LCD_WR_REG(0xF6); //Disable ram power save mode 
  LCD_WR_DATA8(0x00); 

  LCD_WR_REG(0x3A); //65k mode 
  LCD_WR_DATA8(0x05); 
  LCD_WR_REG(0x29); //Display on
  LCD_WR_REG(0x2C);


//  /* �ͷ�Ƭѡ */
//  LCD_CS_HIGH();
}




//��������
//Color:Ҫ���������ɫ
void LCD_Clear(uint16_t Color)
{
	uint8_t VH,VL;
	uint16_t i,j;
	VH=Color>>8;
	VL=Color;	
  
  
  /* д���� */
	Address_set(0,0,LCD_W-1,LCD_H-1);
  for(i=0;i<LCD_W;i++)
  {
    for (j=0;j<LCD_H;j++)
    {
      LCD_WR_DATA8(VH);
      LCD_WR_DATA8(VL);	
    }
  }
   


}
//��ָ��λ����ʾһ������(32*33��С)
//dcolorΪ������ɫ��gbcolorΪ������ɫ
void showhanzi(uint16_t x,uint16_t y,uint8_t index)	
{  
	uint8_t i,j;
	uint8_t* temp = (uint8_t *)hanzi;  

  Address_set(x,y,x+31,y+31); //��������      
  temp+=index*128;	
  for(j=0;j<128;j++)
  {
    for(i=0;i<8;i++)
    { 		     
      if((*temp&(1<<i))!=0)
      {
        LCD_WR_DATA(POINT_COLOR);
      } 
      else
      {
        LCD_WR_DATA(BACK_COLOR);
      }   
    }
    temp++;
  }
  
}




//����
//POINT_COLOR:�˵����ɫ
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	Address_set(x,y,x,y);//���ù��λ�� 
	LCD_WR_DATA(POINT_COLOR); 	    
} 	 




//��һ�����
//POINT_COLOR:�˵����ɫ
void LCD_DrawPoint_big(uint16_t x,uint16_t y)
{
	LCD_Fill(x-1,y-1,x+1,y+1,POINT_COLOR);
} 



//��ָ�����������ָ����ɫ
//�����С:
//  (xend-xsta)*(yend-ysta)
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint16_t i,j; 
	Address_set(xsta,ysta,xend,yend);      //���ù��λ�� 
	for(i=ysta;i<=yend;i++)
	{													   	 	
		for(j=xsta;j<=xend;j++)LCD_WR_DATA(color);//���ù��λ�� 	    
	} 					  	    
}  



//����
//x1,y1:�������
//x2,y2:�յ�����  
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1; //������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //���õ������� 
	else if(delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//ˮƽ�� 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//������� 
	{  
		LCD_DrawPoint(uRow,uCol);//���� 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}    



//������
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}



//��ָ��λ�û�һ��ָ����С��Բ
//(x,y):���ĵ�
//r    :�뾶
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //�ж��¸���λ�õı�־
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a);             //3           
		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0-a,y0+b);             //1       
		LCD_DrawPoint(x0-b,y0-a);             //7           
		LCD_DrawPoint(x0-a,y0-b);             //2             
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0-b);             //5
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-b,y0+a);             
		a++;
		//ʹ��Bresenham�㷨��Բ     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 
		LCD_DrawPoint(x0+a,y0+b);
	}
} 



//��ָ��λ����ʾһ���ַ�

//num:Ҫ��ʾ���ַ�:" "--->"~"
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
//��ָ��λ����ʾһ���ַ�

//num:Ҫ��ʾ���ַ�:" "--->"~"

//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t mode)
{
    uint8_t temp;
    uint8_t pos,t;
	uint16_t x0=x;
	uint16_t colortemp=POINT_COLOR;      
    if(x>LCD_W-16||y>LCD_H-16)return;	    
	//���ô���		   
	num=num-' ';//�õ�ƫ�ƺ��ֵ
	Address_set(x,y,x+8-1,y+16-1);      //���ù��λ�� 
	if(!mode) //�ǵ��ӷ�ʽ
	{
		for(pos=0;pos<16;pos++)
		{ 
			temp=asc2_1608[(uint16_t)num*16+pos];		 //����1608����
			for(t=0;t<8;t++)
		    {                 
		        if(temp&0x01)POINT_COLOR=colortemp;
				else POINT_COLOR=BACK_COLOR;
				LCD_WR_DATA(POINT_COLOR);	
				temp>>=1; 
				x++;
		    }
			x=x0;
			y++;
		}	
	}else//���ӷ�ʽ
	{
		for(pos=0;pos<16;pos++)
		{
		    temp=asc2_1608[(uint16_t)num*16+pos];		 //����1608����
			for(t=0;t<8;t++)
		    {                 
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos);//��һ����     
		        temp>>=1; 
		    }
		}
	}
	POINT_COLOR=colortemp;	    	   	 	  
}   



//m^n����
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}			 
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//color:��ɫ
//num:��ֵ(0~4294967295);	
void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;

	num=(uint16_t)num;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+8*t,y,' ',0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+8*t,y,temp+48,0); 
	}
} 
//��ʾ2������
//x,y:�������
//num:��ֵ(0~99);	 
void LCD_Show2Num(uint16_t x,uint16_t y,uint16_t num,uint8_t len)
{         	
	uint8_t t,temp;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
	 	LCD_ShowChar(x+8*t,y,temp+'0',0); 
	}
} 
//��ʾ�ַ���
//x,y:�������  
//*p:�ַ�����ʼ��ַ
//��16����
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p)
{         
    while(*p!='\0')
    {       
        if(x>LCD_W-16){x=0;y+=16;}
        if(y>LCD_H-16){y=x=0;}
        LCD_ShowChar(x,y,*p,0);
        x+=8;
        p++;
    }  
}


/*
 * ��������itoa
 * ����  ������������ת�����ַ���
 * ����  ��-radix =10 ��ʾ10���ƣ��������Ϊ0
 *         -value Ҫת����������
 *         -buf ת������ַ���
 *         -radix = 10
 * ���  ����
 * ����  ����
 * ����  ����USART1_printf()����
 */
static char *itoa(int value, char *string, int radix)
{
	int     i, d;
	int     flag = 0;
	char    *ptr = string;
	
	/* This implementation only works for decimal numbers. */
	if (radix != 10)
	{
	    *ptr = 0;
	    return string;
	}
	
	if (!value)
	{
	    *ptr++ = 0x30;
	    *ptr = 0;
	    return string;
	}
	
	/* if this is a negative value insert the minus sign. */
	if (value < 0)
	{
	    *ptr++ = '-';
	
	    /* Make the value positive. */
	    value *= -1;
	}
	
	for (i = 10000; i > 0; i /= 10)
	{
	    d = value / i;
	
	    if (d || flag)
	    {
	        *ptr++ = (char)(d + 0x30);
	        value -= (d * i);
	        flag = 1;
	    }
	}
	
	/* Null terminate the string. */
	*ptr = 0;
	
	return string;

} /* NCL_Itoa */



/*
 * ��������USART1_printf
 * ����  ��
 */
void LCD_Printf(uint16_t x, uint16_t y, char *Data,...)
{
	const char *s;
	int d;   
	char buf[16];
  
#define LCD_STRING_HIGHT 16	
#define LCD_STRING_WIDTH 8	
  
	va_list ap;
	va_start(ap, Data);
  
//  uint16_t line = y;

	while ( *Data != 0)     // �ж��Ƿ񵽴��ַ���������
	{				                          
        if ( *Data == 0x5c )  //'\'
        {									  
            switch ( *++Data )
            {
                case 'r':							          //�س���
                case 'n':							          //���з�
                    y = y + LCD_STRING_HIGHT ;
                    x = 0;
                    Data ++;
                break;

                default:
                    Data ++;
                break;
            }			 
        }
        else if ( *Data == '%')
        {									  //
            switch ( *++Data )
            {				
                case 's':										  //�ַ���
                    s = va_arg(ap, const char *);
                    for ( ; *s; s++) 
                    {
                      LCD_ShowChar(x,y,*s,0);
                      if( (x + LCD_STRING_WIDTH) <=  LCD_W)
                      {
                        x = x + LCD_STRING_WIDTH ;
                      }
                      else
                      {
                        y = y + LCD_STRING_HIGHT ;
                        x = 0;
                      }
                    }
                    Data++;
                    break;

                case 'd':										//ʮ����
                    d = va_arg(ap, int);
                    itoa(d, buf, 10);
                    for (s = buf; *s; s++) 
                    {
                        LCD_ShowChar(x,y,*s,0);
                        if( (x + LCD_STRING_WIDTH) <=  LCD_W)
                        {
                          x = x + LCD_STRING_WIDTH ;
                        }
                        else
                        {
                          y = y + LCD_STRING_HIGHT ;
                          x = 0;
                        }
                      
                    }
                    Data++;
                    break;
                default:
                    Data++;
                    break;
            }		 
      } /* end of else if */
      else 
      {
        LCD_ShowChar(x,y,*s++,0);
        if( (x + LCD_STRING_WIDTH) <=  LCD_W)
        {
          x = x + LCD_STRING_WIDTH ;
        }
        else
        {
          y = y + LCD_STRING_HIGHT ;
          x = 0;
        }
      }
	}

}








