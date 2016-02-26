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
#define TX_ADR_WIDTH    5   					// RF�շ���ַ��5 bytes 
#define TX_PLOAD_WIDTH  9  					// ���ݰ�����Ϊ20 bytes

uint8_t const TX_ADDRESS[TX_ADR_WIDTH]  = {0x34,0x56,0x78,0x90,0x15}; // ����RF�շ���ַ

uint8_t data rx_buf[TX_PLOAD_WIDTH];

uint8_t bdata sta;
sbit	RX_DR	=sta^6;
sbit	TX_DS	=sta^5;
sbit	MAX_RT	=sta^4;

static uint8_t dat_mod = 0; //0=len,1=data
static uint8_t sent_len = 0; //ringbuf len

//---------------------------------------------------------
/**************************************************
���ܣ���ʱ
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
���ܣ�Ӳ��SPI��д
**************************************************/
uint8_t SPI_RW(uint8_t value)
{
  SPIRDAT = value;
  											       
  while(!(SPIRSTAT & 0x02));  					// �ȴ�SPI�������

  return SPIRDAT;             					// ���ض���ֵ
}
/**************************************************
���ܣ�дRF�Ĵ�������RF״ֵ̬
**************************************************/
uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

  	RFCSN = 0;                   	
  	status = SPI_RW(reg);      					// ѡ��RF�Ĵ���
  	SPI_RW(value);             					// д������
  	RFCSN = 1;                   	

  	return(status);            					// ����RF״ֵ̬
}
/**************************************************
���ܣ���RF�Ĵ���
**************************************************/
uint8_t SPI_Read(uint8_t reg)
{
	uint8_t reg_val;

  	RFCSN = 0;                			
  	SPI_RW(reg);            					// ѡ��RF�Ĵ���
  	reg_val = SPI_RW(0);    					// ��������
  	RFCSN = 1;                			

  	return(reg_val);        					// ����RF״ֵ̬
}
/**************************************************
���ܣ���RF�Ĵ������ֽ����ݵ�������
**************************************************/
uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status,byte_ctr;

  	RFCSN = 0;                    		
  	status = SPI_RW(reg);       				// ѡ��RF�Ĵ���

  	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
    	pBuf[byte_ctr] = SPI_RW(0);    			// ���Ӷ�������

  	RFCSN = 1;                          

  	return(status);                    			// ����RF״ֵ̬
}
/**************************************************
���ܣ��ѻ������Ķ��ֽ�����д��RF�Ĵ���
**************************************************/
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status,byte_ctr;

  	RFCSN = 0;                   		
  	status = SPI_RW(reg);    					// ѡ��RF�Ĵ���
  	for(byte_ctr=0; byte_ctr<bytes; byte_ctr++) // ����д������
    	SPI_RW(*pBuf++);
  	RFCSN = 1;                 			
  	return(status);          					// ����RF״ֵ̬
}

/**************************************************
���ܣ�����Ϊ����ģʽ
**************************************************/
void RX_Mode(void)
{
	RFCE=0;
  	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);   	// �ϵ�, CRCΪ2 bytes,����ģʽ,����RX_DR�����ж�
  	RFCE = 1; 									// ��������ģʽ
}
/**************************************************
���ܣ�RF��ʼ��
**************************************************/
void rf_init(void)
{
  	RFCE = 0;                                   		// RF�ر�
  	RFCKEN = 1;                                 		// ����RFʱ��
  	RF = 1;                                     		// ����RF�ж�
		IP1 |= 0x02;																		//RF���ȼ���ߣ�3
		IP0 |= 0x02;
	
		delay(1000);	
  	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    	// ���÷����ַ����
  	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); 	// ���ý��յ�ַ����
  	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      			// �����Զ�Ӧ����
  	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  			// PIPE0��������
  	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); 			// �Զ��ش�10��
  	SPI_RW_Reg(WRITE_REG + RF_CH, 40);        			// RFƵ��2440MHz
  	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x0f);   			// ���书��0dBm, ��������2Mbps,
  	SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); 	// PIPE0 �������ݰ�����
}
/**************************************************
���ܣ�RF�жϷ������
**************************************************/
void RF_IRQ(void) interrupt INTERRUPT_RFIRQ
{
	sta=SPI_Read(STATUS);								// ����״ֵ̬

	if(RX_DR)									
	{
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// ����FIFO������
		if(ringbuf_elements(&txbuf) <27) {//ֻ�����е�24���ֽڵĻ��棬��Ϊ����8���ֽ�ֻ�������е�7��
			ringbuf_put(&txbuf,rx_buf[0]);			// ��ʾ����IDֵ
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
		 
		
		SPI_RW_Reg(FLUSH_RX,0);							// ���RX��FIFO
	}

	SPI_RW_Reg(WRITE_REG+STATUS,0x70);					// ��������жϱ�־ 
}
/**************************************************
���ܣ�I2C�жϷ������
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
���ܣ�io��ʼ��
**************************************************/
void io_init()
{
	P0DIR = 0x00;	//OUTPUT
	P1DIR = 0x00;//OUTPUT
	P2DIR = 0xFF;	
	P3DIR = 0xFF;
}
/**************************************************
���ܣ�Timer0��ʼ��
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
���ܣ�I2C��ʼ��
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
���ܣ�main����
**************************************************/
void main()
{	
	io_init();
	rf_init();																				
	ringbuf_init(&txbuf,txbuf_data,RINGBUFSIZE);			//******************************Init ringbuf
  i2c_init();
	time_init();
	
  EA= 1;																						//enable all interrupt

  RX_Mode();																				//�������ģʽ	

	while(1)
	{
		if(RX_DR)								// �������յ�
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



