#ifndef __LCD_H
#define __LCD_H		

#include "stm32f1xx_hal.h"


/**
  * @brief SPI���
  */
#define BSP_SPI_HANDLE       hspi1              /* ָ�������� */
extern  SPI_HandleTypeDef    BSP_SPI_HANDLE;    /* ���� */
#define SPIx_TIMEOUT_MAX     1000              /* SPIͨ�������ʱ */



//����LCD�ĳߴ�	
#define LCD_W 128
#define LCD_H 128

//IO����  


extern  uint16_t BACK_COLOR, POINT_COLOR;   //����ɫ������ɫ


void LCD_ControlLED(uint8_t state);
void Lcd_Init(void); 
void LCD_Clear(uint16_t Color);
void Address_set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);

void LCD_DrawPoint(uint16_t x,uint16_t y);//����
void LCD_DrawPoint_big(uint16_t x,uint16_t y);//��һ�����
uint16_t  LCD_ReadPoint(uint16_t x,uint16_t y); //����
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);		   
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t mode);  //��ʾһ���ַ�
void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len);    //��ʾ����
void LCD_Show2Num(uint16_t x,uint16_t y,uint16_t num,uint8_t len);  //��ʾ2������
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p);		    //��ʾһ���ַ���,16����
 
void showhanzi(uint16_t x,uint16_t y,uint8_t index);
void LCD_Printf(uint16_t x, uint16_t y, char *Data,...);



//������ɫ
#define LCD_COLOR_WHITE         	0xFFFF
#define LCD_COLOR_BLACK         	0x0000	  
#define LCD_COLOR_BLUE         	  0x001F  
#define LCD_COLOR_BRED            0XF81F
#define LCD_COLOR_GRED 			      0XFFE0
#define LCD_COLOR_GBLUE			      0X07FF
#define LCD_COLOR_RED           	0xF800
#define LCD_COLOR_MAGENTA       	0xF81F
#define LCD_COLOR_GREEN         	0x07E0
#define LCD_COLOR_CYAN          	0x7FFF
#define LCD_COLOR_YELLOW        	0xFFE0
#define LCD_COLOR_BROWN 			    0XBC40 //��ɫ
#define LCD_COLOR_BRRED 			    0XFC07 //�غ�ɫ
#define LCD_COLOR_GRAY  			    0X8430 //��ɫ
//GUI��ɫ

#define LCD_COLOR_DARKBLUE      	0X01CF	//����ɫ
#define LCD_COLOR_LIGHTBLUE      	0X7D7C	//ǳ��ɫ  
#define LCD_COLOR_GRAYBLUE       	0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LCD_COLOR_LIGHTGREEN     	0X841F //ǳ��ɫ
#define LCD_COLOR_LGRAY 			    0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LCD_COLOR_LGRAYBLUE       0XA651 //ǳ����ɫ(�м����ɫ)
#define LCD_COLOR_LBBLUE          0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)


					  		 
#endif  
	 
	 



