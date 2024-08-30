/*!
    \file  test.c
    \brief SPI flash test items

    \version 2016-08-15, V1.0.0, demo for GD32F4xx
    \version 2018-12-12, V2.0.0, demo for GD32F4xx
*/

#include <stdio.h>
#include "systick.h"
#include "time.h"
#include "string.h"
#include "gdnftl.h"
#include "stdbool.h"
/*pre define*/
#define SEQUENTIAL   0
#define RANDOM       1

#define TOTAL_PAGES  TOTAL_BLOCK * BLOCK_SIZE

void shuffle(uint16_t *array, uint16_t n);
/*
    function: Full Nand Program
    description: Fully write data to Nand flash in sequential order or random order
    parameter: buffer---data to be programmed buf_len---data size type---sequential or random
*/
uint8_t nandflash_page_program_fully(uint8_t *buffer, uint16_t buf_len, bool type){
    uint16_t block_no, p_block_no;
	uint8_t  page_no;
    uint16_t i;
	uint8_t j;
    uint8_t result;

    if (type == SEQUENTIAL)
    {
        for(block_no = 0; block_no < USER_AREA_END; block_no++){
            p_block_no = get_mapped_physical_block(block_no);
            if(get_BST(p_block_no) == NOT_EMPTY){result = nandflash_block_erase(block_no);}	
            
            for ( page_no = 0; page_no < BLOCK_SIZE; )
            {
                result = nandflash_page_program(buffer, block_no, page_no, buf_len);
								if (result != SPI_NAND_FAIL) {
									page_no = page_no + 1;
								} else {
									printf("Error programming block: %d, page: %d\n", block_no, page_no);
									break; 
								}
            }
            
        }
    }
    return result;
    
}

/*
    function: Partially Nand Program
    description: Fully write data to Nand flash in sequential order or random order
    parameter: buffer---data to be programmed buf_len---data size type---sequential or random
*/
uint8_t nandflash_page_program_partially(uint8_t *buffer, uint16_t buf_len){
    uint16_t block_no, p_block_no;
	uint16_t Block_list[20] = {1, 42, 36, 100, 120, 342, 453, 523, 653, 741, 785, 232, 267, 312, 477, 564, 602, 88, 177, 442};
	uint8_t  page_no;
    uint16_t i;
	uint8_t j;
    uint8_t result;

        for(i = 0; i < 20; i++){
						block_no = Block_list[i];
            p_block_no = get_mapped_physical_block(block_no);
            if(get_BST(p_block_no) == NOT_EMPTY){result = nandflash_block_erase(block_no);}	
            
            for ( page_no = 0; page_no < BLOCK_SIZE; )
            {
                result = nandflash_page_program(buffer, block_no, page_no, buf_len);
								if (result != SPI_NAND_FAIL) {
									page_no = page_no + 1;
								} else {
									printf("Error programming block: %d, page: %d\n", block_no, page_no);
									break; 
								}
            }
            
    }
    return result;
    
}

/*
    function: Whole Nand Erase
    description: Fully erase data to Nand flash
    parameter: 
*/
uint8_t nandflash_erase(void){
    uint16_t block_no;
    uint8_t result;
    for ( block_no = 0; block_no < USER_AREA_END; block_no++)
    {
        result = nandflash_block_erase(block_no);
    }
    return result;
    
}



