/*!
    \file  main.c
    \brief SPI flash demo

    \version 2016-08-15, V1.0.0, demo for GD32F4xx
    \version 2018-12-12, V2.0.0, demo for GD32F4xx
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include <stdio.h>
#include "gd32f4xx.h"
#include "systick.h"
#include "gd32f450i_eval.h"
//#include "gd5f1gxx.h"
#include "string.h"
#include "gdnftl.h"
#include "test.h"

#define BUFFER_SIZE              (2048)
#define countof(a)               (sizeof(a) / sizeof(*(a)))

#define SFLASH_ID                0xC891
#define DBT_SIZE    200
//#define RBT_SIZE    BLOCK_NUM_FOR_USER
#define ENV_SIZE    3
#define TYPE_DBT        0
#define TYPE_RBT        1
#define TYPE_ENV        2
#define TYPE_DBT_RBT    1
//#define DEBUG
#define NFTL_TEST

uint8_t count;
__IO uint32_t TimingDelay = 0;

uint8_t tx_buffer[BUFFER_SIZE];
uint8_t tx_buffer1[BUFFER_SIZE];
uint8_t tx_buffer2[BUFFER_SIZE];
uint8_t rx_buffer[BUFFER_SIZE];
uint8_t erase_buf[BUFFER_SIZE];
uint32_t nandflash_id = 0;
uint16_t i,j = 0;
uint8_t  is_successful = 0;
const char *testData = "Test Data";

int rv = -1;

void turn_on_led(uint8_t led_num);
ErrStatus memory_compare(uint8_t* src, uint8_t* dst, uint16_t length);
void test_status_led_init(void);
void test_case1(void);

// global variable
extern uint16_t ABT[RBT_SIZE];
extern uint64_t PST[RBT_SIZE];
extern uint16_t L2P[RBT_SIZE];
extern uint16_t DBT[DBT_SIZE]; 
extern uint16_t RBT[RBT_SIZE]; 
extern uint16_t ENV[ENV_SIZE]; 
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

#ifdef NFTL_TEST

int main(void)
{
		uint32_t i = 0;
    uint8_t j = 0;
    uint32_t count = 10000;
    for(i = 0; i < BUFFER_SIZE; i++){
            tx_buffer1[i] = 0x55;
			tx_buffer2[i] = 0xAA;
        }
		/* systick configuration */
    systick_config();
    delay_1ms(10);

    /* configure the led GPIO */
    test_status_led_init();

    /* USART parameter configuration */
    gd_eval_com_init(EVAL_COM0);
		
		/* systick configuration */
    systick_config();
    delay_1ms(10);

    /* configure the led GPIO */
    test_status_led_init();

    /* USART parameter configuration */
    gd_eval_com_init(EVAL_COM0);

    /* configure SPI5 GPIO and parameter */
	printf("Initializing..........\n");
    nandflash_init();
	/*PRINT BAD BLOCK INFO*/
	printf("Initialiation Success!\n Bad Block Info:\n Bad block number: %d LastValidBlock: %d \n", DBT[0],DBT[1]);
    for (i = 0; i < DBT[0]; i++){
        printf("%d ", DBT[i+2]);
    }
    #ifdef DEBUG
			//print L2P
			printf("*******L2P******* \n");
			for(i = 0; i < 800; i++){
				printf("logical: %d physical: %d erase_cnt: %d \n", i, L2P[i], ABT[L2P[i]]);
			}
	#endif
    
    /* read SPI NAND ID */
    nandflash_id = spi_nandflash_read_id();
    printf("\n\rThe Flash_ID:0x%X\n\r\n\r",nandflash_id);
	printf("*****************CERTAIN BLOCK ERS PGM TEST******************\n");
    //test_case1();   
	//nandflash_erase();
    for ( i = 0; i < count; i++)
    {		
			
		printf("CYCLE: %d\n", i);
        nandflash_page_program_certain_blocks(tx_buffer1, tx_buffer2, SPI_NAND_PAGE_SIZE);
    }
    
    
}

#else
int main(void)
{
    /* systick configuration */
    systick_config();
    delay_1ms(10);

    /* configure the led GPIO */
    test_status_led_init();

    /* USART parameter configuration */
    gd_eval_com_init(EVAL_COM0);

    /* configure SPI5 GPIO and parameter */
    spi_nandflash_init();
		/*PRINT BAD BLOCK INFO*/
	  printf("Initialiation Success!\n Bad Block Info:\n Bad block number: %d LastValidBlock: %d", DBT[0],DBT[1]);
    /* read SPI NAND ID */
    flash_id = spi_nandflash_read_id();
    printf("\n\rThe Flash_ID:0x%X\n\r\n\r",flash_id);

    /* flash id is correct */
    if(SFLASH_ID == flash_id){
        for(i = 0; i < BUFFER_SIZE; i++){
            tx_buffer[i] = i+50;
        }
        memset(erase_buf, 0xFF, 2048);
    #ifdef DEBUG
            if(SPI_NAND_FAIL == nandflash_block_erase(339)){//try erase bad block check DBT & L2P
    #else
            if(SPI_NAND_FAIL == nandflash_block_erase(600)){
    #endif
            printf("\n\rErase block failed!\n\r");
            while(1);
        }
        
				#ifdef DEBUG
            if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, 339, 0, 500)){//
				#else
        if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, 600, 0, 500)){
				#endif
            /* if failed, retry to program */
            if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, 339, 0, 500)){
                printf("\n\rWrite page failed!\n\r");
                while(1);
            }
        }
        
        if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, 600, 1, 500)){
            /* if failed, retry to program */
            if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, 600, 1, 500)){
                printf("\n\rWrite page failed!\n\r");
                while(1);
            }
        }
        if(SPI_NAND_FAIL == nandflash_page_read(rx_buffer, 600, 1, 0, 500)){
            printf("\n\rRead page failed!\n\r");
            while(1);
        }
        if(ERROR == memory_compare(tx_buffer,rx_buffer,500)){
            printf("\n\rErr:Data Read and Write aren't Matching.\n\r");
            is_successful = 1;
            while(1);
        }
        
        if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, 600, 2, 500)){
            /* if failed, retry to program */
            if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, 600, 2, 500)){
                printf("\n\rWrite page failed!\n\r");
                while(1);
            }
        }
        
        if(SPI_NAND_FAIL == nandflash_page_read(rx_buffer, 600, 0, 10, 500)){
            printf("\n\rRead page failed!\n\r");
            while(1);
        }
        if(ERROR == memory_compare(&tx_buffer[10], rx_buffer, 500-10)){
            printf("\n\rErr:Data Read and Write aren't Matching.\n\r");
            is_successful = 1;
            while(1);
        }
        
        delay_1ms(1);
        
        if(SPI_NAND_FAIL == nandflash_block_erase(751)){
            printf("\n\rErase block failed!\n\r");
            while(1);
        }
        if(SPI_NAND_FAIL == nandflash_page_read(rx_buffer, 751, 0, 0, BUFFER_SIZE)){
            printf("\n\rRead page failed!\n\r");
            while(1);
        }
        if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, 751, 0, BUFFER_SIZE)){
            /* if failed, retry to program */
            if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, 751, 0, BUFFER_SIZE)){
                printf("\n\rWrite page failed!\n\r");
                while(1);
            }
        }
        if(SPI_NAND_FAIL == nandflash_page_read(rx_buffer, 751, 0, 0, BUFFER_SIZE)){
            printf("\n\rRead page failed!\n\r");
            while(1);
        }
        if(ERROR == memory_compare(tx_buffer,rx_buffer,BUFFER_SIZE)){
            printf("\n\rErr:Data Read and Write aren't Matching.\n\r");
            is_successful = 1;
            while(1);
        }

        /* SPI test passed */
        if(0 == is_successful){
            printf("\n\rSPI-GD NAND Test Passed!\n\r");
        }
    #if 0
        /* all pages and blocks test */
        for(i=0; i<=USER_AREA_END; i++){
            /* erase first */
            nandflash_block_erase(i);
            for(j=0; j<SPI_NAND_BLOCK_SIZE; j++){
                nandflash_page_read(rx_buffer, i, j, 0, BUFFER_SIZE);
                if(0 != memcmp(rx_buffer, erase_buf, BUFFER_SIZE)){
                    printf("\n\rErr:Data Read and Write aren't Matching.\n\r");
                    is_successful = 1;
                    while(1);
                }
                if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, i, j, BUFFER_SIZE)){
                    /* if failed, retry to program */
                    if(SPI_NAND_FAIL == nandflash_page_program(tx_buffer, i, j, BUFFER_SIZE)){
                        printf("\n\rWrite page failed!\n\r");
                        while(1);
                    }
                }
                nandflash_page_read(rx_buffer, i, j, 0, BUFFER_SIZE);
                
                delay_1ms(1);

                if(ERROR == memory_compare(tx_buffer,rx_buffer,BUFFER_SIZE)){
                    printf("\n\rErr:Data Read and Write aren't Matching.\n\r");
                    is_successful = 1;
                    while(1);
                }
            }
        }
    #endif
    }else{
        /*  read ID fail */
        printf("\n\rSPI NAND: Read ID Fail!\n\r");
    }

    while(1){
        /* turn off all leds */
        gd_eval_led_off(LED1);
        gd_eval_led_off(LED2);
        gd_eval_led_off(LED3);

        /* turn on a led */
        turn_on_led(count % 3);
        count ++;
        if(3 <= count)
           count = 0;

        delay_1ms(500);
    }
}
#endif

/*!
    \brief      test status led initialize
    \param[in]  none
    \param[out] none
    \retval     none
*/
void test_status_led_init(void)
{
    /* initialize the leds */
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
    gd_eval_led_init(LED3);

    /* turn off all leds */
    gd_eval_led_off(LED1);
    gd_eval_led_off(LED2);
    gd_eval_led_off(LED3);
}

/*!
    \brief      turn on led
    \param[in]  led_num: led number
    \param[out] none
    \retval     none
*/
void turn_on_led(uint8_t led_num)
{
    switch(led_num){
    case 0:
        /* turn on LED1 */
        gd_eval_led_on(LED1);
        break;
    case 1:
        /* turn on LED2 */
        gd_eval_led_on(LED2);
        break;
    case 2:
        /* turn on LED3 */
        gd_eval_led_on(LED3);
        break;
    default:
        /* turn on all leds */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        gd_eval_led_on(LED3);
        break;
    }
}

/*!
    \brief      memory compare function
    \param[in]  src: source data pointer
    \param[in]  dst: destination data pointer
    \param[in]  length: the compare data length
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus memory_compare(uint8_t* src, uint8_t* dst, uint16_t length) 
{
    while(length --){
        if(*src++ != *dst++)
            return ERROR;
    }
    return SUCCESS;
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));
    return ch;
}
/*test case: 1. erase a bad block, check 
            2. Keep erasing same bad block check ABT
*/
void test_case1(void){
    uint16_t logical_block = 200;
    //uint16_t physical_block = L2P[logical_block];
    //uint16_t Erase_CNT = get_ABT(physical_block);
    uint32_t i = 0;
    uint8_t j = 0;
    uint32_t count = 1000000;
    for(i = 0; i < BUFFER_SIZE; i++){
            tx_buffer1[i] = 0x55;
						tx_buffer2[i] = 0xAA;
        }
		/*
    for ( i = 0; i < count; i++)
    {   
				if(i%2){
					printf("CLCYE:%d Pattern:0x55 \t\n", i);
					nandflash_block_erase(logical_block);
					for ( j = 0; j < 64; j++)
					{   
            nandflash_page_program(tx_buffer1, logical_block, j, BUFFER_SIZE);
						nandflash_page_read(rx_buffer, logical_block, j, 0, BUFFER_SIZE);
					}
				}
				else{
					printf("CLCYE:%d Pattern:0xAA \t\n", i);
					nandflash_block_erase(logical_block);
					for ( j = 0; j < 64; j++)
					{   
            nandflash_page_program(tx_buffer2, logical_block, j, BUFFER_SIZE);
						nandflash_page_read(rx_buffer, logical_block, j, 0, BUFFER_SIZE);
					}
				}
        
        
    }
		*/
    
    //nandflash_block_erase(logical_block);
		//nandflash_page_program(tx_buffer, logical_block, 0, BUFFER_SIZE);
    //nandflash_page_program(tx_buffer, logical_block, 1, BUFFER_SIZE);	
    //nandflash_page_program(tx_buffer, logical_block, 2, BUFFER_SIZE);		
    /*READ TEST TO TRIGGER ECC DATA MOVE*/
		
    for ( i = 0; i < count; i++)
    {
			printf("Read Cycle %d\t\t", i);
			for ( j = 0; j < 64; j++)
					{   
            //nandflash_page_program(tx_buffer1, logical_block, j, BUFFER_SIZE);
						nandflash_page_read(rx_buffer, logical_block, j, 0, BUFFER_SIZE);
					}
    }
		
    
/*
    for(i = 0; i < 300; i++){
				//SET ENV TO BACK UP 
        ENV[0] = 0x00;
        ENV[1] = logical_block;
        ENV[2] = select_unmapped_block();
        update_DBTRBT_to_nand(TYPE_ENV);
        nandflash_block_erase(logical_block);
        printf("Erasing.. Logical Block: %d Physical Block: %d Block Erase CNT: %d \n", logical_block, L2P[logical_block],ABT[L2P[logical_block]]);
        //program
        nandflash_page_program(tx_buffer, logical_block, 0, BUFFER_SIZE);
        
        //SET ENV TO DONE 
        ENV[0] = 0xFF;
        ENV[1] = 0XFF;
        ENV[2] = 0xFF;
				update_DBTRBT_to_nand(TYPE_ENV);
    }
		logical_block = 364;
		for(i = 0; i < 100; i++){
        //program
        nandflash_page_program(tx_buffer, logical_block, 0, BUFFER_SIZE);
        printf("Programme: Logical Block: %d Physical Block: %d Block Erase CNT: %d \n", logical_block, L2P[logical_block], ABT[L2P[logical_block]]);
        
        //printf("Erase %d \n", i );
        //printf("Before Erasing \n Logical Block: %d Physical Block: %d Block Erase CNT: %d \n", logical_block, L2P[logical_block], ABT[L2P[logical_block]]);
        nandflash_block_erase(logical_block);
        printf("After Erasing \n Logical Block: %d Physical Block: %d Block Erase CNT: %d \n", logical_block, L2P[logical_block],ABT[L2P[logical_block]]);
    }
		logical_block = 339;
		for(i = 0; i < 100; i++){
        //program
        nandflash_page_program(tx_buffer, logical_block, 0, BUFFER_SIZE);
        printf("Programme: Logical Block: %d Physical Block: %d Block Erase CNT: %d \n", logical_block, L2P[logical_block], ABT[L2P[logical_block]]);
        
        //printf("Erase %d \n", i );
        //printf("Before Erasing \n Logical Block: %d Physical Block: %d Block Erase CNT: %d \n", logical_block, L2P[logical_block], ABT[L2P[logical_block]]);
        nandflash_block_erase(logical_block);
        printf("After Erasing \n Logical Block: %d Physical Block: %d Block Erase CNT: %d \n", logical_block, L2P[logical_block],ABT[L2P[logical_block]]);
    }
*/
		
		#ifdef DEBUG
			//print L2P
			printf("*******L2P******* \n");
			for(i = 0; i < 800; i++){
				printf("logical: %d physical: %d erase_cnt: %d \n", i, L2P[i], ABT[L2P[i]]);
			}
		#endif
}
