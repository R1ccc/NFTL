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

extern uint16_t L2P[RBT_SIZE];
extern uint16_t ENV[ENV_SIZE];
extern uint8_t move_page_data(uint16_t des_block_No,uint16_t src_block_No,uint8_t page_No);
void shuffle(uint16_t *array, uint16_t n);
bool check_duplicates_in_L2P(int size);
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
    function: CERTAIN BLOCKS Nand Program
    description: Fully write data to Nand flash in sequential order or random order
    parameter: buffer---data to be programmed buf_len---data size type---sequential or random
*/
uint8_t nandflash_page_program_certain_blocks(uint8_t *buffer1, uint8_t *buffer2, uint16_t buf_len){
    uint16_t block_no, p_block_no;
	uint16_t Block_list[9] = {1, 42, 100, 342, 453, 523, 653, 785, 232};
    uint8_t L2P_err_msg[50] = "L2P ERROR!";
	uint8_t  page_no;
    uint16_t i;
	uint8_t j;
    uint8_t result;
    uint8_t count = 10;

    for(i = 200; i < 230; i++){
		block_no = i;
        p_block_no = get_mapped_physical_block(block_no);
        for ( j = 0; j < count; j++)
        {
            //check the L2P map table every 50 cycles
            if( (j % 50 == 0) && (j != 0)){
                if(check_duplicates_in_L2P(800)){
                    spi_nandflash_block_erase(0);
                    spi_nandflash_write_data(L2P_err_msg, 0, 0, 50);
                };
            }
            
            //SET OP FLAG
            ENV[0] = 0x00;
            ENV[1] = block_no;
            ENV[2] = select_unmapped_block();
            //back up data to BLOCK ENV[2]
						printf("Backing Up Data from %d to %d \n", ENV[1], ENV[2]);
            for ( page_no = 0; page_no < 64; page_no++)
            {   
                move_page_data(ENV[2], ENV[1], page_no);   
            }

            //SAVE OP FLAG TO NAND
            update_DBTRBT_to_nand(TYPE_ENV);

            //erase & update data
            nandflash_block_erase(block_no);
            for ( page_no = 0; page_no < 64; page_no++)
            {   
                if(page_no % 2) {nandflash_page_program(buffer1, block_no, page_no, SPI_NAND_PAGE_SIZE);}
                else {nandflash_page_program(buffer2, block_no, page_no, SPI_NAND_PAGE_SIZE);}    
            }

            //CLEAR OP FLAG
            //SET OP FLAG
						printf("Operation Done! Erasing back-up block %d \n", ENV[2]);
            ENV[0] = 0xFF;
						nandflash_block_erase(ENV[2]);
            
            //SAVE OP FLAG TO NAND
            update_DBTRBT_to_nand(TYPE_ENV);
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

/*
    function: Check L2P
    description: Check if block mapping is correct 
    parameter: 
*/
bool check_duplicates_in_L2P(int size) {
    bool seen[ReplaceBlock_AREA_END] = {false}; // record seen value
		uint16_t i;
		
    for (i = 0; i < size; i++) {
        if (L2P[i] >= 0 && L2P[i] < ReplaceBlock_AREA_END) { 
            if (seen[L2P[i]]) {  
                return true;     
            }
            seen[L2P[i]] = true;  
        } else {
            printf("Error: L2P value %d is out of valid range.\n", L2P[i]);
            return false;
        }
    }

    return false; 
}
