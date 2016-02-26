/******************************************************/
/* I2C Slave example code
/* Wait for data, increase the first byte by 1,
/* Send back the first byte increased when requested.
/******************************************************/

#include "nrf24le1.h"
#include <hal_w2.h>
#include "ringbuf.h"
#include <stdint.h>
#include "API.h"
#define LED0 P15
#define LED1 P16
#define signal P01
#define TIMEOUT  53333		//40ms
#define RINGBUFSIZE 32
uint16_t flag =0;
uint8_t flag2 =0;
uint8_t int_count=0;
uint8_t TX =0;
uint8_t first_byte=0;

uint16_t timeout_flag = 0; 

static struct ringbuf txbuf;
uint8_t txbuf_data[RINGBUFSIZE];
uint8_t sladdress = 0x07;

#define INTERRUPT_RFIRQ	9
#define TX_ADR_WIDTH    5   					// RF收发地址共5 bytes 
#define TX_PLOAD_WIDTH  9  					// 数据包长度为20 bytes

uint8_t const TX_ADDRESS[TX_ADR_WIDTH]  = {0x34,0x56,0x78,0x90,0x15}; // 定义RF收发地址

uint8_t data rx_buf[TX_PLOAD_WIDTH];

uint8_t bdata sta;
sbit	RX_DR	=sta^6;
sbit	TX_DS	=sta^5;
sbit	MAX_RT	=sta^4;

static uint8_t dat_mod = 0; //0=len,1=data
static uint8_t sent_len = 0; //ringbuf len

//---------------------------------------------------------
/**************************************************
功能：延时
**************************************************/
void delay(uint16_t x)
{
    uint16_t i,j;
    i=0;
    for(i=0;i<x;i++)
    {
       j=108;
           ;
       while(j--);
    }
}
/**************************************************
功能：硬件SPI读写
**************************************************/
uint8_t SPI_RW(uint8_t value)
{
  SPIRDAT = value;
  											       
  while(!(SPIRSTAT & 0x02));  					// 等待SPI传输完成

  return SPIRDAT;             					// 返回读出值
}
/**************************************************
功能：写RF寄存器，读RF状态值
**************************************************/
uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

  	RFCSN = 0;                   	
  	status = SPI_RW(reg);      					// 选择RF寄存器
  	SPI_RW(value);             					// 写入数据
  	RFCSN = 1;                   	

  	return(status);            					// 返回RF状态值
}
/**************************************************
功能：读RF寄存器
**************************************************/
uint8_t SPI_Read(uint8_t reg)
{
	uint8_t reg_val;

  	RFCSN = 0;                			
  	SPI_RW(reg);            					// 选择RF寄存器
  	reg_val = SPI_RW(0);    					// 读出数据
  	RFCSN = 1;                			

  	return(reg_val);        					// 返回RF状态值
}
/**************************************************
功能：读RF寄存器多字节数据到缓冲区
**************************************************/
uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status,byte_ctr;

  	RFCSN = 0;                    		
  	status = SPI_RW(reg);       				// 选择RF寄存器

  	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
    	pBuf[byte_ctr] = SPI_RW(0);    			// 连接读出数据

  	RFCSN = 1;                          

  	return(status);                    			// 返回RF状态值
}
/**************************************************
功能：把缓冲区的多字节数据写到RF寄存器
**************************************************/
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status,byte_ctr;

  	RFCSN = 0;                   		
  	status = SPI_RW(reg);    					// 选择RF寄存器
  	for(byte_ctr=0; byte_ctr<bytes; byte_ctr++) // 连接写入数据
    	SPI_RW(*pBuf++);
  	RFCSN = 1;                 			
  	return(status);          					// 返回RF状态值
}

/**************************************************
功能：设置为接收模式
**************************************************/
void RX_Mode(void)
{
	RFCE=0;
  	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);   	// 上电, CRC为2 bytes,接收模式,允许RX_DR产生中断
  	RFCE = 1; 									// 启动接收模式
}
/**************************************************
功能：RF初始化
**************************************************/
void rf_init(void)
{
  	RFCE = 0;                                   		// RF关闭
  	RFCKEN = 1;                                 		// 启动RF时钟
  	RF = 1;                                     		// 允许RF中断
		IP1 |= 0x02;																		//RF优先级最高：3
		IP0 |= 0x02;
	
		delay(1000);	
  	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    	// 设置发射地址长度
  	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); 	// 设置接收地址长度
  	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      			// 启动自动应答功能
  	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  			// PIPE0接收数据
  	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); 			// 自动重传10次
  	SPI_RW_Reg(WRITE_REG + RF_CH, 40);        			// RF频率2440MHz
  	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x0f);   			// 发射功率0dBm, 传输速率2Mbps,
  	SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); 	// PIPE0 接收数据包长度
}
/**************************************************
功能：RF中断服务程序
**************************************************/
void RF_IRQ(void) interrupt INTERRUPT_RFIRQ
{
	sta=SPI_Read(STATUS);								// 读出状态值

	if(RX_DR)									
	{
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// 读出FIFO的数据
		if(ringbuf_elements(&txbuf) <27) {//只用其中的24个字节的缓存，因为后面8个字节只能用其中的7个
			ringbuf_put(&txbuf,rx_buf[0]);			// 显示所有ID值
			ringbuf_put(&txbuf,rx_buf[1]);
			ringbuf_put(&txbuf,rx_buf[2]);
			ringbuf_put(&txbuf,rx_buf[3]);
			ringbuf_put(&txbuf,rx_buf[4]);
			ringbuf_put(&txbuf,rx_buf[5]);
			ringbuf_put(&txbuf,rx_buf[6]);
			ringbuf_put(&txbuf,rx_buf[7]);
			ringbuf_put(&txbuf,rx_buf[8]);			
		}
		LED0=!LED0;
	//trigger reader request from pi
	 // make sure to use only 24 bytes buf, to avoid last byte's missing in ringbuf
		
		signal = !signal;
		
//		flag++;
//		if(1 == flag){
//			signal =0;
//		}
//		if(3 == flag){
//			signal =1;
//			flag = 0;
//		}
		 
		
		SPI_RW_Reg(FLUSH_RX,0);							// 清除RX的FIFO
	}

	SPI_RW_Reg(WRITE_REG+STATUS,0x70);					// 清除所有中断标志 
}
/**************************************************
功能：I2C中断服务程序
**************************************************/
void I2C_IRQ (void) interrupt INTERRUPT_SERIAL
{

	uint8_t w2con1_value;
	w2con1_value = W2CON1;
	
				if ((w2con1_value & 0x04) )	   //Interrupt for address matched
				{
					// Check if RX or TX
					int_count++;
					if (W2DAT&0x01) //TX first tx to pi
					{
						TX=1; //TX
						//W2DAT=flag;
								if(0 == dat_mod){//first get len									
										W2DAT=ringbuf_elements(&txbuf);
									//W2DAT = 8;
									}else{//second get to get all data in ring buf
										W2DAT = ringbuf_get(&txbuf);
										//W2DAT = 0x06;
								}
						
					}
					else //RX
					{
						TX=0; //RX
						first_byte=1;
					 }
				}
				else  
				{
					if ((w2con1_value & 0x01)) //  Interrupt for byte sent or received 
					{
						if (TX==1) //next byte to send
						{
							if (!(w2con1_value & 0x02))	// if not NAK
							{
									if(0 == dat_mod){									
										
									}else{
										W2DAT = ringbuf_get(&txbuf);
										//W2DAT = 0x06;
									}
							}
						}
						else //byte to receive
						{
							dat_mod=W2DAT;
						}
							
					}
				}		
 }
/**************************************************
功能：io初始化
**************************************************/
void io_init()
{
	P0DIR = 0x00;	//OUTPUT
	P1DIR = 0x00;//OUTPUT
	P2DIR = 0xFF;	
	P3DIR = 0xFF;
}
/**************************************************
功能：Timer0初始化
**************************************************/
void time_init()
{
	TMOD |= 0x01;	//16 bit timer
//TMOD |= 0x11;	//16 bit timer
	TH0=(65536-TIMEOUT)/256;
	TL0=(65536-TIMEOUT)%256;
	ET0=1; //Enable interupt 
	TR0=1;
}
/**************************************************
功能：I2C初始化
**************************************************/
void i2c_init()
{
	hal_w2_enable(1);  
 	hal_w2_set_clk_freq(HAL_W2_400KHZ);
 	hal_w2_set_op_mode(HAL_W2_SLAVE);
	hal_w2_set_slave_address(sladdress);// Slave ADDRESS 0x07
	hal_w2_alter_clock(1);
	hal_w2_irq_adr_match_enable(1);
	hal_w2_irq_stop_cond_enable(0);
	INTEXP |= 0x04; 
	IEN1 |= 0x04;
		
}
/**************************************************
功能：main函数
**************************************************/
void main()
{	
	io_init();
	rf_init();																				
	ringbuf_init(&txbuf,txbuf_data,RINGBUFSIZE);			//******************************Init ringbuf
  i2c_init();
	time_init();
	
  EA= 1;																						//enable all interrupt

  RX_Mode();																				//进入接收模式	

	while(1)
	{
		if(RX_DR)								// 数据已收到
		{
			sta=0;
		}
	}	
	
}

void Timer0_IRQ(void) interrupt INTERRUPT_T0
{
	TH0=(65536-TIMEOUT)/256;
	TL0=(65536-TIMEOUT)%256;
	
	timeout_flag++;
	LED1 = !LED1;
	
	if((timeout_flag >= 250) && (ringbuf_elements(&txbuf) != 0))											//250*40 = 10000ms;
	{
		signal =! signal;
	}
}



