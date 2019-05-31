#include "stm32f0xx.h"        // Device header
#include "math.h"							// ABS, SQRT, COS, SIN  etc..
#include "stdio.h"						//Sscanf

//PA1 - LCD_RES
//PF0 - SDA	,	PF1 - SCL

#define T191_Addr					0x78
#define LCD_ON()					GPIOA->BSRR |=1<<1
#define LCD_RES()					GPIOA->BRR  |=1<<1

struct GPS_RX {
	uint8_t	time_mm;
	uint8_t time_ss;
	uint8_t time_hh;
	uint32_t 	lat;
	uint8_t		lat_sign; // N=0,S=1
	uint32_t 	lon;
	uint8_t		lon_sign;	//E=0,W=1
	uint16_t 	speed;
	uint16_t	course;
} GPS_RMC;


uint8_t i,act_GPS=6,word_flag;
uint8_t flag_update_lcd_gps=0;
char usart_rx=0,USART_BUF[82],temp_buf=0,TMP_BUF[82];

const  uint32_t R_Earth = 6371000; //Earth radius
const float Pi=3.14159265358979;

//---------------Timer settings-------------
uint16_t TimingDelay,led_count,ms1000;
uint8_t ms200=0,msec200=0,sec_tic=0;

void TimingDelayDec(void) 																													{
 if (TimingDelay			!=0x00) TimingDelay--;
 if (!led_count) {led_count=500; GPIOB->ODR ^=1;}
 if (!ms1000) {ms1000=1000;sec_tic=1;}
 if (!ms200) {ms200=200;msec200=1;}
 led_count--;ms1000--;ms200--;
 }

void TIM17_IRQHandler(void)																													{
		if (TIM17->SR & TIM_SR_UIF) {
					TimingDelayDec();
  				TIM17->SR &=(~TIM_SR_UIF);
		}
}	
void delay_ms (uint16_t DelTime) 																										{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}

//------------------------------------T191 LCD ------------------------------
const unsigned char T191_font[]= 																										{// Font 8*5 
	0x00, 0x00, 0x00, 0x00, 0x00 ,   // sp  32
     0x00, 0x00, 0x2f, 0x00, 0x00 ,   // !   33
     0x00, 0x07, 0x00, 0x07, 0x00 ,   // "   34
     0x14, 0x7f, 0x14, 0x7f, 0x14 ,   // #   35
     0x24, 0x2a, 0x7f, 0x2a, 0x12 ,   // $   36
     0xc4, 0xc8, 0x10, 0x26, 0x46 ,   // %   37
     0x36, 0x49, 0x55, 0x22, 0x50 ,   // &   38
     0x00, 0x05, 0x03, 0x00, 0x00 ,   // '   39
     0x00, 0x1c, 0x22, 0x41, 0x00 ,   // (   40
     0x00, 0x41, 0x22, 0x1c, 0x00 ,   // )   41
     0x14, 0x08, 0x3E, 0x08, 0x14 ,   // *   42
     0x08, 0x08, 0x3E, 0x08, 0x08 ,   // +   43
     0x00, 0x00, 0x50, 0x30, 0x00 ,   // ,   44
     0x10, 0x10, 0x10, 0x10, 0x10 ,   // -   45
     0x00, 0x60, 0x60, 0x00, 0x00 ,   // .   46
     0x20, 0x10, 0x08, 0x04, 0x02 ,   // /   47
     0x3E, 0x51, 0x49, 0x45, 0x3E ,   // 0   48
     0x00, 0x42, 0x7F, 0x40, 0x00 ,   // 1   49
     0x42, 0x61, 0x51, 0x49, 0x46 ,   // 2   50
     0x21, 0x41, 0x45, 0x4B, 0x31 ,   // 3   51
     0x18, 0x14, 0x12, 0x7F, 0x10 ,   // 4   52
     0x27, 0x45, 0x45, 0x45, 0x39 ,   // 5   53
     0x3C, 0x4A, 0x49, 0x49, 0x30 ,   // 6   54
     0x01, 0x71, 0x09, 0x05, 0x03 ,   // 7   55
     0x36, 0x49, 0x49, 0x49, 0x36 ,   // 8   56
     0x06, 0x49, 0x49, 0x29, 0x1E ,   // 9   57
     0x00, 0x36, 0x36, 0x00, 0x00 ,   // :   58
     0x00, 0x56, 0x36, 0x00, 0x00 ,   // ;   59
     0x08, 0x14, 0x22, 0x41, 0x00 ,   // <   60
     0x14, 0x14, 0x14, 0x14, 0x14 ,   // =   61
     0x00, 0x41, 0x22, 0x14, 0x08 ,   // >   62
     0x02, 0x01, 0x51, 0x09, 0x06 ,   // ?   63
     0x32, 0x49, 0x59, 0x51, 0x3E ,   // @   64
     0x7E, 0x11, 0x11, 0x11, 0x7E ,   // A   65
     0x7F, 0x49, 0x49, 0x49, 0x36 ,   // B   66
     0x3E, 0x41, 0x41, 0x41, 0x22 ,   // C   67
     0x7F, 0x41, 0x41, 0x22, 0x1C ,   // D   68
     0x7F, 0x49, 0x49, 0x49, 0x41 ,   // E   69
     0x7F, 0x09, 0x09, 0x09, 0x01 ,   // F   70
     0x3E, 0x41, 0x49, 0x49, 0x7A ,   // G   71
     0x7F, 0x08, 0x08, 0x08, 0x7F ,   // H   72
     0x00, 0x41, 0x7F, 0x41, 0x00 ,   // I   73
     0x20, 0x40, 0x41, 0x3F, 0x01 ,   // J   74
     0x7F, 0x08, 0x14, 0x22, 0x41 ,   // K   75
     0x7F, 0x40, 0x40, 0x40, 0x40 ,   // L   76
     0x7F, 0x02, 0x0C, 0x02, 0x7F ,   // M   77
     0x7F, 0x04, 0x08, 0x10, 0x7F ,   // N   78
     0x3E, 0x41, 0x41, 0x41, 0x3E ,   // O   79
     0x7F, 0x09, 0x09, 0x09, 0x06 ,   // P   80
     0x3E, 0x41, 0x51, 0x21, 0x5E ,   // Q   81
     0x7F, 0x09, 0x19, 0x29, 0x46 ,   // R   82
     0x46, 0x49, 0x49, 0x49, 0x31 ,   // S   83
     0x01, 0x01, 0x7F, 0x01, 0x01 ,   // T   84
     0x3F, 0x40, 0x40, 0x40, 0x3F ,   // U   85
     0x1F, 0x20, 0x40, 0x20, 0x1F ,   // V   86
     0x3F, 0x40, 0x38, 0x40, 0x3F ,   // W   87
     0x63, 0x14, 0x08, 0x14, 0x63 ,   // X   88
     0x07, 0x08, 0x70, 0x08, 0x07 ,   // Y   89
     0x61, 0x51, 0x49, 0x45, 0x43 ,   // Z   90
     0x00, 0x7F, 0x41, 0x41, 0x00 ,   // [   91
     0x55, 0x2A, 0x55, 0x2A, 0x55 ,   // 55  92
     0x00, 0x41, 0x41, 0x7F, 0x00 ,   // ]   93
     0x04, 0x02, 0x01, 0x02, 0x04 ,   // ^   94
     0x40, 0x40, 0x40, 0x40, 0x40 ,   // _   95
     0x00, 0x01, 0x02, 0x04, 0x00 ,   // '   96
     0x20, 0x54, 0x54, 0x54, 0x78 ,   // a   97
     0x7F, 0x48, 0x44, 0x44, 0x38 ,   // b   98
     0x38, 0x44, 0x44, 0x44, 0x20 ,   // c   99
     0x38, 0x44, 0x44, 0x48, 0x7F ,   // d   100
     0x38, 0x54, 0x54, 0x54, 0x18 ,   // e   101
     0x08, 0x7E, 0x09, 0x01, 0x02 ,   // f   102
     0x0C, 0x52, 0x52, 0x52, 0x3E ,   // g   103
     0x7F, 0x08, 0x04, 0x04, 0x78 ,   // h   104
     0x00, 0x44, 0x7D, 0x40, 0x00 ,   // i   105
     0x20, 0x40, 0x44, 0x3D, 0x00 ,   // j   106
     0x7F, 0x10, 0x28, 0x44, 0x00 ,   // k   107
     0x00, 0x41, 0x7F, 0x40, 0x00 ,   // l   108
     0x7C, 0x04, 0x18, 0x04, 0x78 ,   // m   109
     0x7C, 0x08, 0x04, 0x04, 0x78 ,   // n   110
     0x38, 0x44, 0x44, 0x44, 0x38 ,   // o   111
     0x7C, 0x14, 0x14, 0x14, 0x08 ,   // p   112
     0x08, 0x14, 0x14, 0x18, 0x7C ,   // q   113
     0x7C, 0x08, 0x04, 0x04, 0x08 ,   // r   114
     0x48, 0x54, 0x54, 0x54, 0x20 ,   // s   115
     0x04, 0x3F, 0x44, 0x40, 0x20 ,   // t   116
     0x3C, 0x40, 0x40, 0x20, 0x7C ,   // u   117
     0x1C, 0x20, 0x40, 0x20, 0x1C ,   // v   118
     0x3C, 0x40, 0x30, 0x40, 0x3C ,   // w   119
     0x44, 0x28, 0x10, 0x28, 0x44 ,   // x   120
     0x0C, 0x50, 0x50, 0x50, 0x3C ,   // y   121
     0x44, 0x64, 0x54, 0x4C, 0x44 ,   // z   122
    //
     0x00, 0x08, 0x36, 0x41, 0x00 ,   //7B- {
     0x00, 0x00, 0x7f, 0x00, 0x00 ,   //7C- |
     0x00, 0x41, 0x36, 0x08, 0x00 ,   //7D- }
     0x04, 0x02, 0x04, 0x08, 0x04 ,   //7E- ~
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //7F- 
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //80- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //81- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //82- ‚
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //83- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //84- „
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //85- …
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //86- †
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //87- ‡
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //88- €
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //89- ‰
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8A- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8B- ‹
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8C- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8D- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8E- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8F- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //90- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //91- ‘
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //92- ’
     0x00, 0x00, 0x00, 0x00, 0x00 ,   // 93- “
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //94- ”
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //95- •
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //96- –
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //97- —
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //98- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //99- ™ 
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9A- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9B- ›
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9C- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9D- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9E- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9F- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A0-  
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A1- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A2- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A3- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A4- ¤
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A5- ?
     0x00, 0x00, 0x36, 0x00, 0x00 ,   //A6- ¦
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A7- §
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A8- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A9- ©
     0x3E, 0x49, 0x49, 0x49, 0x22 ,   //AA- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //AB- «
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //AC- ¬
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //AD- ­
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //AE- ®
     0x44, 0x45, 0x7C, 0x45, 0x44 ,   //AF- ?
     0x06, 0x09, 0x09, 0x06, 0x00 ,   //B0- ° - Degree Symbol
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B1- ±
     0x00, 0x41, 0x7F, 0x41, 0x00 ,   //B2- ?
     0x00, 0x44, 0x7D, 0x40, 0x00 ,   //B3- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B4- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B5- µ
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B6- ¶
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B7- ·
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B8- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B9- ?
     0x38, 0x54, 0x44, 0x28, 0x00 ,   //BA- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //BB- »
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //BC- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //BD- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //BE- ?
     0x4A, 0x48, 0x7A, 0x40, 0x40 ,   //BF- ?                             
    //???????
 0x7E, 0x11, 0x11, 0x11, 0x7E , // A        /*192*/
 0x7F, 0x49, 0x49, 0x49, 0x31 , // ?
 0x7F, 0x49, 0x49, 0x49, 0x36 , // B
 0x7F, 0x01, 0x01, 0x01, 0x03 , // ?
 0x60, 0x3E, 0x21, 0x21, 0x7F , // ?
 0x7F, 0x49, 0x49, 0x49, 0x41 , // E
 0x63, 0x14, 0x7F, 0x14, 0x63 , // ?
 0x22, 0x49, 0x49, 0x49, 0x36 , // ?
 0x7F, 0x10, 0x08, 0x04, 0x7F , // ?
 0x7F, 0x10, 0x09, 0x04, 0x7F , // ?
 0x7F, 0x08, 0x14, 0x22, 0x41 , // K
 0x7C, 0x02, 0x01, 0x01, 0x7F , // ?
 0x7F, 0x02, 0x0C, 0x02, 0x7F , // M
 0x7F, 0x08, 0x08, 0x08, 0x7F , // H
 0x3E, 0x41, 0x41, 0x41, 0x3E , // O
 0x7F, 0x01, 0x01, 0x01, 0x7F , // ?
 0x7F, 0x09, 0x09, 0x09, 0x06 , // P
 0x3E, 0x41, 0x41, 0x41, 0x22 , // C
 0x01, 0x01, 0x7F, 0x01, 0x01 , // T
 0x07, 0x48, 0x48, 0x48, 0x3F , // ?
 0x1C, 0x22, 0x7F, 0x22, 0x1C , // ?
 0x63, 0x14, 0x08, 0x14, 0x63 , // X
 0x3F, 0x20, 0x20, 0x3F, 0x60 , // ?
 0x07, 0x08, 0x08, 0x08, 0x7F , // ?
 0x7F, 0x40, 0x7F, 0x40, 0x7F , // ?
 0x3F, 0x20, 0x3F, 0x20, 0x7F , // ?
 0x01, 0x7F, 0x48, 0x48, 0x30 , // ?
 0x7F, 0x48, 0x48, 0x30, 0x7F , // ?
 0x7F, 0x48, 0x48, 0x48, 0x30 , // ?
 0x22, 0x49, 0x49, 0x49, 0x3E , // ?
 0x7F, 0x08, 0x3E, 0x41, 0x3E , // ?
 0x46, 0x29, 0x19, 0x09, 0x7F , // ?
 0x20, 0x54, 0x54, 0x54, 0x78 , // a
 0x78, 0x54, 0x54, 0x54, 0x20 , // b
 0x7C, 0x54, 0x54, 0x54, 0x28 , // ?
 0x7C, 0x04, 0x04, 0x04, 0x00 , // ?
 0x60, 0x38, 0x24, 0x38, 0x60 , // ?
 0x38, 0x54, 0x54, 0x54, 0x18 , // e
 0x6C, 0x10, 0x7C, 0x10, 0x6C , // ?
 0x28, 0x44, 0x54, 0x54, 0x28 , // ?
 0x3C, 0x40, 0x40, 0x20, 0x7C , // ?
 0x3C, 0x40, 0x42, 0x20, 0x7C , // ?
 0x7C, 0x10, 0x10, 0x28, 0x44 , // ?
 0x60, 0x10, 0x08, 0x04, 0x7C , // ?
 0x7C, 0x08, 0x10, 0x08, 0x7C , // ?
 0x7C, 0x10, 0x10, 0x10, 0x7C , // ?
 0x38, 0x44, 0x44, 0x44, 0x38 , // o
 0x7C, 0x04, 0x04, 0x04, 0x7C , // ?
 0x7C, 0x14, 0x14, 0x14, 0x08 , // p
 0x38, 0x44, 0x44, 0x44, 0x20 , // c
 0x04, 0x04, 0x7C, 0x04, 0x04 , // ?
 0x0C, 0x50, 0x50, 0x50, 0x3C , // y
 0x18, 0x24, 0x7C, 0x24, 0x18 , // ?
 0x44, 0x28, 0x10, 0x28, 0x44 , // x
 0x3C, 0x20, 0x20, 0x3C, 0x60 , // ?
 0x0C, 0x10, 0x10, 0x10, 0x7C , // ?
 0x7C, 0x40, 0x7C, 0x40, 0x7C , // ?
 0x3C, 0x20, 0x3C, 0x20, 0x7C , // ?
 0x04, 0x7C, 0x48, 0x48, 0x30 , // ?
 0x7C, 0x48, 0x30, 0x00, 0x7C , // ?
 0x7C, 0x48, 0x48, 0x48, 0x30 , // ?
 0x28, 0x44, 0x54, 0x54, 0x38 , // ?
 0x7C, 0x10, 0x38, 0x44, 0x38 , // ?
 0x58, 0x24, 0x24, 0x24, 0x7C  //  ?
 };


void I2C_write(unsigned char byte_send)																			    		{//WRITE
	 while(!(I2C1->ISR & I2C_ISR_TXE)){}; 
	 I2C1->TXDR = byte_send;
	}
void I2C_Start(void)    																														{//Start
	 while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	 I2C1->CR2 |= (T191_Addr << I2C_CR2_SADD_Pos) | (0xff << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	 while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	}
void I2C_Stop(void) 																																{//Stop
	 while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	 I2C1->CR2 |= I2C_CR2_STOP;
	 while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	 I2C1->ICR |= I2C_ICR_STOPCF;		
 }
void LCD_Init(void) 																																{// INIT LCD T191
    
     LCD_RES();delay_ms(10);
  	 LCD_ON();delay_ms(10);

     I2C_Start();
     I2C_write(0x00);I2C_write(0x31);
     I2C_write(0x14);I2C_write(0x06); 																			// Initial Settings
     I2C_write(0x30);I2C_write(0x11);
     I2C_write(0x05);I2C_write(0x31);
     I2C_write(0x9A); 																											// Contrast
     I2C_write(0x0C); 																											//0x0C -Normal mode, 00 - Mirror mode
     I2C_write(0x30);I2C_write(0x0C); 																			// ?????????? ?????
     I2C_write(0x40);I2C_write(0x80); 																			// GoTo x=0, y=0
     I2C_Stop();
    }
void LCD_Gotoxy ( uint8_t x, uint8_t y )														    						{//78.00.30.80|x.40|y
       I2C_Start();
       I2C_write(0x00);
       I2C_write(0x30);
       I2C_write(0x80 | 1+x*6);  																//(0...15 Colls
       I2C_write(0x40 | y);  																		//(0...7 - Rows)
       I2C_Stop();
    }

void LCD_Write_data (unsigned char dat_byte)																	      {//78.40.XX
       I2C_Start();
       I2C_write(0x40);
       I2C_write(dat_byte);
       I2C_Stop();
      }

void LCD_mode(uint8_t mode)																													{//78.00.30.0C - Normal; 78.00.30.0D - Inverse 
      
       I2C_Start();
       if(mode > 0)
       {I2C_write(0x00);I2C_write(0x30);I2C_write(0x0C);}
       else
       {I2C_write(0x00);I2C_write(0x30);I2C_write(0x0D);}
       I2C_Stop();
      }
void LCD_Char(char flash_o, uint8_t mode)															        			{// Print Symbol
      	 unsigned char i,ch; 
         I2C_Start();
         I2C_write(0x40);
          for(i=0; i<5; i++)
      	  {
      	    ch = T191_font[(flash_o - 32)*5+i];
      	     if(mode) { I2C_write(~ch); }
      			 else  	 { I2C_write(ch);}
      		 if(i == 4)
      			  {
      			     if(mode) { I2C_write(0xff); }
      				  else   {	 I2C_write(0x00); }
      			  }
      	   }
         I2C_Stop();
          }
void LCD_PrintStr(char *s, uint8_t mode)																	    			{// Print String
          while (*s)
        {
          LCD_Char(*s, mode);
          s++;
        }
      }
void LCD_Clear(void)      																													{// Clear LCD
    	 unsigned char i;
    	for(i = 0; i < 10; i++) LCD_PrintStr("                ",0);
      }
       
void LCD_ClearStr(uint8_t y,uint8_t qn)      																				{// Clear String
 uint8_t temp_i; 	 
	for (temp_i=0;temp_i<qn;temp_i++)
	 {
		 LCD_Gotoxy(0,y+temp_i);    	
		 LCD_PrintStr("                ",0);
   }
}
void LCD_PrintDec(long value,uint8_t mode) 																					{// Print Dec
	
	char i=1,d=0;
	unsigned char text[10];
	do 
  { 
    if (value >=10)  {
				d = value % 10; 																				// ??????? ?? ???????
				text[i] = d + '0'; 																			// ????? ASCII ?????? ? ???????? ? ?????
				value /= 10; 																						// "?????????? ????? ??????" -- ??????? ?? 10
			}
		else 
			{	text[i] = value + '0';
				value=0;
			}
 		i++;
  }
	while(value); 
	i--;			
	do
	{	LCD_Char(text[i], mode);
		i--;
	}
	while(i);
}
			
void LCD_PrintHex(long value,uint8_t mode) 																					{// Print Hex
	
	char i=1,d=0;
	unsigned char text[10];
	do 
  { 
    if (value >=0x10)  {
				d = value % 0x10; 																				
				if(d<0x0A) text[i] = d + '0'; 																			
				else 			 text[i] = d + 0x37;
				value /= 0x10; 																						
			}
		else 
			{	
				if(value < 0x0A)	text[i] = value + '0';			//0..9
				else 							text[i] = value + 0x37;			//A...F
				value=0;
			}
 		i++;
  }
	while(value); 
	i--;			
	do
	{	LCD_Char(text[i], mode);
		i--;
	}
	while(i);
}

void LCD_PrintBin(uint8_t value,uint8_t mode) 																			{// Print Bin
	
	uint8_t i=1,d=0;
	uint8_t text[8];
	do 
  { 
    if (value >=2)  {
				d = value % 2; 																				
				text[i] = d + '0'; 																			
				value /= 0x2; 																						
			}
		else 
			{	
				text[i] = value + '0';			//0..9
				value=0;
			}
 		i++;
  }
	while(i<9); 
	i--;			
	do
	{	LCD_Char(text[i], mode);
		i--;
	}
	while(i);
}
//---------------------------------End of LCD INIT----------------------------
void USART1_IRQHandler(void) 																				{ //USART1 IRQ HANDLER
	if (USART1->ISR & USART_ISR_RXNE)																	{	// Got Data in RX - Buffer 
	 
		USART_BUF[usart_rx]=(uint8_t)(USART1->RDR);			//PutChar(UART1->DR,0);
		if((USART_BUF[usart_rx]==0x24) || (USART_BUF[usart_rx]==0x21)) 
				{USART_BUF[0]=USART_BUF[usart_rx]; usart_rx=0;} 							//Preambule detected 
		usart_rx++;
		if((USART_BUF[usart_rx-1]==0x0D)&&(USART_BUF[usart_rx]==0x0A)) 	{	//Frame Received
			if ((USART_BUF[3]=='R')&& (USART_BUF[17]=='A'))								{	//There is good RMC frame received
					for (i=0;i<82;i++) {TMP_BUF[i]=USART_BUF[i];}
					word_flag=1; usart_rx--;temp_buf=usart_rx;
			}
			else 																													{	//Not RMC frame
						usart_rx=0; 
						for (i=0;i<82;i++) 
							{USART_BUF[i]=0x00;} //Flush not RMC Frame
					}	
		}
			if (usart_rx > 82) usart_rx =0;
		}
	if (USART1->ISR & USART_ISR_ORE)																	{	//
	}
	}


	
void BUF_DATA_new(void)		{
int hh,mm,ss,ms;
float t_lat,t_lon,speed,course;
		//Driving-  $GPRMC,112717.00,A,4845.41395,N,04449.16781,E,4.169,52.00,270519,,,A*5B<CR><LF>
//--------------------------Time----------------------------------
sscanf(TMP_BUF,"$GPRMC,%2d%2d%2d.%2d,A,%f,%c,%f,%c,%f,%f", 
	&hh, &mm, &ss, &ms, &t_lat,&GPS_RMC.lat_sign, &t_lon,	&GPS_RMC.lon_sign, &speed, &course);

	GPS_RMC.lat=(uint32_t)(t_lat*100000);
	GPS_RMC.lon=(uint32_t)(t_lon*100000);
	
	GPS_RMC.speed = (uint16_t)(speed*1.852);
	GPS_RMC.course= (uint16_t)course;
	GPS_RMC.time_hh=(uint8_t)hh;
	GPS_RMC.time_mm=(uint8_t)mm;
	GPS_RMC.time_ss=(uint8_t)ss;
	/*
	GPS_RMC.time_hh= 	((TMP_BUF[7]-0x30)*10 	+ TMP_BUF[8]-0x30);
	GPS_RMC.time_mm= 	((TMP_BUF[9]-0x30)*10 	+ TMP_BUF[10]-0x30);
	GPS_RMC.time_ss=	((TMP_BUF[11]-0x30)*10 	+	TMP_BUF[12]-0x30);
//-------------------------Latitude-------------------------------	
	GPS_RMC.lat=							 ((TMP_BUF[19]-0x30)*10	+ TMP_BUF[20]-0x30)*10000000;
	GPS_RMC.lat =	GPS_RMC.lat +((TMP_BUF[21]-0x30)*1000 + (TMP_BUF[22]-0x30)*100	+ (TMP_BUF[24]-0x30)*10 + (TMP_BUF[25]-0x30))*1000;
	GPS_RMC.lat = GPS_RMC.lat +((TMP_BUF[26]-0x30)*100	 + (TMP_BUF[27]-0x30)*10 	+ (TMP_BUF[28]-0x30));
	if (TMP_BUF[30]=='N') {GPS_RMC.lat_sign=0;} 	// N=0,S=1
	else										{GPS_RMC.lat_sign=1;}	
//-------------------------Longitude------------------------------	
	GPS_RMC.lon= 								((TMP_BUF[32]-0x30)*100	 + (TMP_BUF[33]-0x30)*10 	+ (TMP_BUF[34]-0x30))*10000000;
	GPS_RMC.lon=	GPS_RMC.lon	+	((TMP_BUF[35]-0x30)*1000 + (TMP_BUF[36]-0x30)*100	+ (TMP_BUF[38]-0x30)*10 + (TMP_BUF[39]-0x30))*1000;
	GPS_RMC.lon=	GPS_RMC.lon +	((TMP_BUF[40]-0x30)*100	 + (TMP_BUF[41]-0x30)*10 	+ (TMP_BUF[42]-0x30));
	if (USART_BUF[44]=='E') {GPS_RMC.lon_sign=0;} 	//E=0,W=1
	else										{GPS_RMC.lon_sign=1;}	
//-------------------------Speed in knots---------------------------------
//	for convert into km/h need multiply by 1.852
	for (i=0;i<7;i++) 							{//[46] - first Speed Symbol
		SPD_BUF[i]= TMP_BUF[i+46];
		sp_buf=i;	
		if (SPD_BUF[i+47]==',') {j=sp_buf+47;i=8;} 	
	}
	
//------------------------Course----------------------------------	
	for (i=0;i<7;i++) 							{//j is ptr. on first Speed Symbol
		CRS_BUF[i]= TMP_BUF[i+j];
		cr_buf=i;	
		if (CRS_BUF[i+j]==',') {i=8;} 	
	}
//----------------------------------------------------------------
*/
}

void initial (void)		{
//---------------TIM17------------------
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    																			//HSI 8 MHz - 1 msek
  TIM17->PSC = 8000-1;
  TIM17->ARR = 1;
  TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CEN; 											// 
	TIM17->DIER |=TIM_DIER_UIE;
	NVIC_EnableIRQ (TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn,0x05);	

//-------------------GPIOB-Blinking Led		
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; 																					//
	GPIOB->MODER |= GPIO_MODER_MODER0_0;																					//Pb0-Out 
																				
//------------I2C1---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOFEN;
	GPIOF->MODER 		|=GPIO_MODER_MODER0_1 		| GPIO_MODER_MODER1_1; 							//Alt -mode /Pf0 - SDA, Pf1- SCL
	GPIOF->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR0 	| GPIO_OSPEEDER_OSPEEDR1;						//50 MHz max speed
	GPIOF->AFR[0] 	|=(1<<GPIO_AFRL_AFRL0_Pos) |(1<<GPIO_AFRL_AFRL1_Pos);  				//I2C - Alternative
	
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN; 
	GPIOA->MODER |= GPIO_MODER_MODER1_0;																					//LCD_Res_pin

	RCC->APB1ENR |=RCC_APB1ENR_I2C1EN;
	I2C1->TIMINGR |=(0x5<< I2C_TIMINGR_PRESC_Pos);						// If HSI = 8MHz,  Presc =0x05 - 380kHz Freq
	I2C1->CR2 &=(~I2C_CR2_HEAD10R) & (~I2C_CR2_ADD10);
	I2C1->CR1 |=I2C_CR1_PE;
	
//--------------------------USART--------------------------------	
	/*PB6, PB7 - Alternative function USART1*/
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;; 								// AFR=0 - UART TX, UART RX
		
	RCC->APB2ENR |=RCC_APB2ENR_USART1EN;
	USART1->BRR = 0x341; // 9600  ((8000000 /9600)) & 0x0f
	USART1->CR1 |= USART_CR1_RE | USART_CR1_UE  | USART_CR1_RXNEIE;// 
	NVIC_EnableIRQ (USART1_IRQn); // 	
	NVIC_SetPriority(USART1_IRQn,0x02);

	__enable_irq ();	
} 


int main(void) 									{	//main

initial();
LCD_Init();
LCD_Clear();
LCD_Gotoxy (0,0);LCD_PrintStr(" STM32F042+GPS ",1);
//------------------Main Loop--------------------------------------------
while (1)  											{// Main loop
	
	if(word_flag) 		{//Has got received Data from GPS
			BUF_DATA_new(); 
			flag_update_lcd_gps=1;
			act_GPS=0;
			word_flag=0;
	}
	else 							{//No Data occurred from GPS
	if (sec_tic) { act_GPS++;}	
	}
	
	

	if (msec200) 			{//Print new data on LCD every 200 msek
		if(flag_update_lcd_gps)							{//Has data for update gps

			//-------------------------------TIME-----------------------------
			LCD_Gotoxy (0,2);	LCD_PrintStr("Time GMT:",0);if(GPS_RMC.time_hh<10)LCD_PrintStr(" ",0);LCD_PrintDec(GPS_RMC.time_hh,0); 
			LCD_PrintStr(":",0);if(GPS_RMC.time_mm<10)LCD_PrintStr("0",0);LCD_PrintDec(GPS_RMC.time_mm,0);
			//-------------------------------TIME_MSK--------------------------
			GPS_RMC.time_hh +=3; if (GPS_RMC.time_hh>23) GPS_RMC.time_hh-=24;
			LCD_Gotoxy (0,3);	LCD_PrintStr("Time MSK:",0);if(GPS_RMC.time_hh<10)LCD_PrintStr(" ",0);LCD_PrintDec(GPS_RMC.time_hh,0); 
			LCD_PrintStr(":",0);if(GPS_RMC.time_mm<10)LCD_PrintStr("0",0);LCD_PrintDec(GPS_RMC.time_mm,0);
			//------------------------------Latitude--------------------------
			LCD_Gotoxy (0,4);	LCD_PrintStr("Lat ",0);LCD_PrintDec(GPS_RMC.lat/10000000,0);LCD_Char(0xb0,0);
			LCD_PrintDec((GPS_RMC.lat/100000)%100,0);LCD_Char(0x27,0);
			LCD_PrintDec(GPS_RMC.lat%100000,0);if(GPS_RMC.lat_sign==0){ LCD_PrintStr("N ",0);}
																					else { LCD_PrintStr("S ",0);}
			//------------------------------Longitude-------------------------																	
			LCD_Gotoxy (0,5);	LCD_PrintStr("Lon ",0);LCD_PrintDec(GPS_RMC.lon/10000000,0);LCD_Char(0xb0,0);
			LCD_PrintDec((GPS_RMC.lon/100000)%100,0);LCD_Char(0x27,0);																	
			LCD_PrintDec(GPS_RMC.lon%100000,0);if(GPS_RMC.lon_sign==0){ LCD_PrintStr("E ",0);}
																					else { LCD_PrintStr("W ",0);}
		//------------------------------Speed-----------------------------																	
			LCD_Gotoxy (0,6);	LCD_PrintStr("Speed ",0);LCD_PrintDec(GPS_RMC.speed,0);LCD_PrintStr("km./h.  ",0);
			LCD_Gotoxy (0,7);	LCD_PrintStr("Course ",0);LCD_PrintDec(GPS_RMC.course,0);LCD_Char(0xb0,0);LCD_PrintStr("  ",0);
		
			flag_update_lcd_gps=0;

		}
		if (act_GPS>5)											{// 5sec - Clean LCD if no new data from GPS
					LCD_ClearStr(4,4);
					LCD_Gotoxy (0,5);	LCD_PrintStr("No signal",0);
					act_GPS=0;
			}
	msec200=0;
	}
	sec_tic=0;	

} // end - main loop 
} // end - Main  
	
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
