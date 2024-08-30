/*!
    \file  gdnftl.c
    \brief NAND FLASH TRANSLATION LAYER
           BAD BLOCK MANAGEMENT
           WEAR LEVELING
           POWER LOSS RECOVER
*/

/*
    Copyright (C) 2016 GigaDevice

    2016-10-19, V1.0.0, demo for GD32F4xx
*/

#include "gdnftl.h"
#include "string.h"
#include "systick.h"
#include <stdio.h>
#include <stdbool.h>

//#define  DEBUG
//#define  TEST
#define NFTL_TEST
#define LOG
#define ABT_ENABLE
//#define DEMO//ONLY TEST CODE, ERASE ALL WHEN INIT
//#define TEST_LOAD_UPDATE_ABTL2P
static uint8_t tem_buffer[SPI_NAND_PAGE_TOTAL_SIZE];   /* the buffer of read page data */

#define true    1
#define false   0

#define TYPE_DBT        0
#define TYPE_RBT        1
#define TYPE_ENV        2
#define TYPE_ABT        3
#define TYPE_L2PBST     4
#define WL_THRESHOLD 5
#define DBT_SIZE    200
#define RBT_SIZE    BLOCK_NUM_FOR_USER
#define ENV_SIZE    3
#define TOTAL_BLOCK 1024
#define BST_COMPRESSED_SIZE TOTAL_BLOCK/8
#define EMPTY       0
#define NOT_EMPTY   1
#define ABT_THRESHOLD 10
#define L2P_THRESHOLD 10
#define BST_THRESHOLD 10

uint32_t flash_id;

uint16_t DBT[DBT_SIZE]; 
uint16_t RBT[RBT_SIZE]; 
uint16_t ENV[ENV_SIZE] = {0xFFFF, 0xFFFF, 0xFFFF}; 

uint16_t g16DBT_Block[2] = {0xFFFF, 0xFFFF};
uint8_t  g8DBT_CRT_Block_Idx;   //0 or 1
uint8_t  g8DBT_CRT_Page;

uint16_t g16RBT_Block[2] = {0xFFFF, 0xFFFF};
uint8_t  g8RBT_CRT_Block_Idx;
uint8_t  g8RBT_CRT_Page;

uint16_t g16ABT_Block[2] = {0xFFFF, 0xFFFF};
uint8_t  g8ABT_CRT_Block_Idx;   //0 or 1
uint8_t  g8ABT_CRT_Page;

uint16_t g16L2PBST_Block[2] = {0xFFFF, 0xFFFF};
uint8_t  g8DL2PBST_CRT_Block_Idx;   //0 or 1
uint8_t  g8DL2PBST_CRT_Page;

uint16_t g16ENV_Block[2] = {0xFFFF, 0xFFFF};
uint8_t  g8ENV_CRT_Block_Idx;
uint8_t  g8ENV_CRT_Page;

uint8_t BST_compressed[BST_COMPRESSED_SIZE];

uint16_t g16LastValidBlock;
uint16_t g16BadBlockNum;
uint16_t table_area_lastvalid = Table_AREA_END;

volatile bool     BST[TOTAL_BLOCK];//mark if the physical block is empty
uint16_t ABT[TOTAL_BLOCK];//PHYSICAL BLOCK ERASE COUNT
uint16_t L2P[RBT_SIZE];//LOGICAL BLOCK ADDRESS TO PHYSICAL BLOCK ADDRESS
uint8_t  ABT_CNT = 0;//COUNTER RECORD HOW MANY TIMES ABT MODIFIED 
uint8_t  L2P_CNT = 0;//COUNTER RECORD HOW MANY TIMES L2P MODIFIED 
uint8_t  BST_CNT = 0;//COUNTER RECORD HOW MANY TIMES BST MODIFIED 
uint8_t  DBT_BLOCK_1;
/* detect the nandflash block is or not bad */


bool FLAG_ABT_UPDATE = 0;
bool FLAG_L2P_UPDATE = 0;
bool FLAG_BST_UPDATE = 0;

const uint16_t supported_flash_ids[SUPPORTED_FLASH_IDS_COUNT] = {
    0xC891,//GD5F1GM7UE
    0xC881,//GD5F1GM7RE
    0xC851,//GD5F1GQ5UE
    0xC841,//GD5F1GQ5RE
    0xC892,//GD5F2GM7UE
    0xC882,//GD5F2GM7RE
    0xC852,//GD5F2GQ5UE
    0xC842,//GD5F2GQ5RE
    0xC895,//GD5F4GM8UE
    0xC885,//GD5F4GM8RE
    0xC855,//GD5F4GQ6UE
    0xC845,//GD5F4GQ6RE
	0xC831
};

static void load_DBTRBT_from_nand(uint8_t type);
static uint8_t check_whether_in_DBT_array(uint16_t BlockNo);
uint8_t update_DBTRBT_to_nand(uint8_t type);
static void add_bad_Block_to_DBTRBT_ram(uint16_t BlockNo, uint16_t PhysicalBlockNo);
static uint16_t re_mapping_RBT(uint16_t ori_BlockNo,uint16_t old_replace_BlockNo);
static uint8_t move_page_data(uint16_t des_block_No,uint16_t src_block_No,uint8_t page_No);
static uint16_t get_replace_block_from_ram(uint16_t BlockNo);
static uint8_t update_DBTRBT_array(uint16_t BlockNo, uint16_t PhysicalBlockNo);
static void alloc_DBTRBT_block_addr(void);
static void init_build_DBTRBT(void);
static uint8_t rebuild_DBTRBT_array(void);
static void set_mapped_physical_block(uint16_t block_No, uint16_t physical_block); 
 uint8_t update_L2PBST_to_nand(void);
static uint8_t update_ABT_to_nand(void);
static uint8_t mark_bad_block(uint16_t physical_block_no);
static void load_ABT_from_nand(void);
 void load_L2PBST_from_nand(void);
static void compress_BST(volatile bool *BST, uint8_t *BST_compressed, uint16_t total_blocks);
static void decompress_BST(const uint8_t *BST_compressed, volatile bool *BST, uint16_t total_blocks);
static uint8_t table_block_replacement(uint8_t type);

uint16_t get_mapped_physical_block(uint16_t block_No);
static void set_ABT(uint16_t block_No, uint16_t value);
static uint16_t get_ABT(uint16_t block_No);
static void set_BST(uint16_t block_No, uint16_t value);
bool get_BST(uint16_t block_No);

static uint16_t get_empty_block(bool *array, size_t size, uint16_t ori_block);
static uint16_t get_min_erase_block(uint16_t ori_pb);
static uint16_t calculate_average(uint16_t *array, uint16_t size);
static uint16_t select_proper_block(uint16_t physical_block_No);
uint16_t select_unmapped_block(void);
static void env_check(void);
static bool is_supported_flash_id(uint16_t id);

//visable to user
/*nandflash initialization*/
void nandflash_init(void);
/* write the page data to SPI nandflash, block_No(0~USER_AREA_END), page_No(0~SPI_NAND_BLOCK_SIZE-1), buf_len(length of buffer to be written) */
uint8_t nandflash_page_program(uint8_t *buffer, uint16_t block_No, uint8_t page_No, uint16_t buf_len);
/* read the page data from SPI nandflash, block_No(0~USER_AREA_END), page_No(0~SPI_NAND_BLOCK_SIZE-1),address_in_page(0~SPI_NAND_PAGE_SIZE-1), buf_len(length of buffer to be read) */
uint8_t nandflash_page_read(uint8_t *buffer, uint16_t block_No, uint8_t page_No, uint16_t address_in_page, uint16_t buf_len);
/* erase the block data of SPI nandflash, block_No(0~USER_AREA_END) */
uint8_t nandflash_block_erase(uint16_t block_No);


/*!
    \brief      write the page data to nandflash
    \param[in]  buffer: the data of array
    \param[in]  block_No: the address in nandflash block(0~799)
    \param[in]  page_No: the address in nandflash page(0~63) in block_No
    \param[in]  buf_len: the length of array(1~2048)
    \param[out] none
    \retval     SPI_NAND_FAIL,SPI_NAND_SUCCESS 
*/
void nandflash_init(void){
    spi_nandflash_init();
    /*ID VERIFICATION*/
    flash_id = spi_nandflash_read_id();
    //printf("\n\rThe Flash_ID:0x%X\n\r\n\r",flash_id);
    if (is_supported_flash_id(flash_id)) {
        printf("Supported NAND Flash ID 0x%X\n", flash_id);
        // Continue initialization
    } else {
        printf("Error: Unsupported NAND Flash ID 0x%X\n", flash_id);
        // Optionally halt further initialization
        while (1); // Infinite loop to halt the system
    }
    rebuild_DBTRBT_array();
    env_check();
}

/*!
    \brief      write the page data to nandflash
    \param[in]  buffer: the data of array
    \param[in]  block_No: the address in nandflash block(0~799)
    \param[in]  page_No: the address in nandflash page(0~63) in block_No
    \param[in]  buf_len: the length of array(1~2048)
    \param[out] none
    \retval     SPI_NAND_FAIL,SPI_NAND_SUCCESS 
*/

uint8_t nandflash_page_program(uint8_t *buffer, uint16_t block_No, uint8_t page_No, uint16_t buf_len)
{
    uint8_t result;
    uint8_t page_idx;
    uint16_t replace_block,new_replace_block; 
    uint16_t physical_block, logical_block;
    uint16_t proper_block;
	uint16_t average;
	uint16_t i = 0;
    //L2P find the physical block    
    logical_block = block_No;
    physical_block = get_mapped_physical_block(logical_block);


    if((block_No > USER_AREA_END)||(page_No>=SPI_NAND_BLOCK_SIZE)||(buf_len>SPI_NAND_PAGE_SIZE)){
        return SPI_NAND_FAIL;
    }
    if(check_whether_in_DBT_array(physical_block)==true){//check if the mapped physical block is bad block
        replace_block = get_replace_block_from_ram(block_No);
        #ifdef LOG
            printf("*********Bad Block**********\nBad block detected: %x Original PB: %X Updated PB: %X\n",block_No, physical_block, replace_block);
		#endif
        set_mapped_physical_block(logical_block, replace_block);//update L2P
    }else{
        replace_block = physical_block;
    }
    
    //check erase cnt
    //based on wear leveling select the proper physical block
    #ifdef ABT_ENABLE
        average = calculate_average(ABT, 1024);
        if((ABT[replace_block] - average) > WL_THRESHOLD){
        
            proper_block = select_proper_block(replace_block);
            
            #ifdef LOG
            printf("***************Wear Leveling************* \nphyblock %d ers_cnt %d average %d rpl_blk %d\n", replace_block, ABT[replace_block], average, proper_block);		
            #endif
            //swap mapping table L2P
            swap_L2P(block_No, proper_block);
            
            L2P_CNT++;//COUNTER ++ 
            //CHECK IF NEED TO UPDATE L2P
        }
    #endif
    replace_block = get_mapped_physical_block(logical_block);//ACTUAL PROGRAM PHYSICAL BLOCK
		
    memset(tem_buffer, 0x5A, SPI_NAND_PAGE_SIZE);
    memcpy(tem_buffer, buffer, buf_len);
    #ifdef LOG
				printf("*********PROGRAM LB:%d PB:%d PAGE:%d**********\n",block_No, replace_block, page_No);
	#endif
    result = spi_nandflash_write_data(tem_buffer,replace_block*SPI_NAND_BLOCK_SIZE+page_No,0,SPI_NAND_PAGE_SIZE);
    if(result == SPI_NAND_FAIL){
        #ifdef LOG
            printf("***************Program Failed, Block Replacing************* \n");		
        #endif     
        update_DBTRBT_array(block_No, replace_block);
        mark_bad_block(replace_block);
        re_mapping_RBT(block_No,replace_block);
        update_DBTRBT_to_nand(TYPE_DBT);
        update_DBTRBT_to_nand(TYPE_RBT); 
        new_replace_block = get_replace_block_from_ram(block_No);
        //copy the old block content to new alloced block
        for(page_idx = 0;page_idx < page_No;page_idx++){
            move_page_data(new_replace_block,replace_block,page_idx);
        }
        swap_L2P(block_No, new_replace_block);
        //L2P[block_No] = new_replace_block;//update the L2P table
    }
    else{
        BST[replace_block] = NOT_EMPTY;//mark this physical block as not empty
        BST_CNT++;
    }
    compress_BST(BST, BST_compressed, TOTAL_BLOCK);
    //CHECK IF NEED TO UPDATE BST
    //CHECK IF NEED TO UPDATE BST&ABT
    if(ABT_CNT > 10){
        update_ABT_to_nand();
        #ifdef LOG
        printf("***************ABT TO NAND************* \n");		
        #endif
        ABT_CNT = 0;
    }

    if(BST_CNT > 64){
        #ifdef LOG
        printf("***************L2PBST TO NAND************* \n");		
        #endif
        update_L2PBST_to_nand();
        BST_CNT = 0;
    }
    return result;
}

/*!
    \brief      read the data from nandflash
    \param[in]  *buffer:the data of array
    \param[in]  block_No: the address in nandflash block(0~799)
    \param[in]  page_No: the serial number of nandflash page(0~63) in block_No
    \param[in]  buf_len: the length of array(1~2048)
    \param[out] none
    \retval     SPI_NAND_FAIL,SPI_NAND_SUCCESS 
*/
#ifdef NFTL_TEST
uint8_t nandflash_page_read(uint8_t *buffer, uint16_t block_No, uint8_t page_No, uint16_t address_in_page, uint16_t buf_len)
{
    uint16_t p_block_no = L2P[block_No];
    uint16_t replace_block;     
    uint8_t result;
    uint16_t ori_block;
    uint16_t des_block;
		uint8_t		i;
    if((block_No > USER_AREA_END)||(page_No>=SPI_NAND_BLOCK_SIZE)||(buf_len>SPI_NAND_PAGE_SIZE)){
        return SPI_NAND_FAIL;
    }
    
    if(check_whether_in_DBT_array(p_block_no)==true){//check pba is bb or not
        replace_block = get_replace_block_from_ram(block_No);
        set_mapped_physical_block(block_No, replace_block);//update L2P TABLE
    }else{
        replace_block = p_block_no;
    }
		#ifdef LOG
			printf("*********READ PB:%X Page: %d**********\n",replace_block, page_No);
		#endif
    result = spi_nandflash_read_data(buffer,replace_block*SPI_NAND_BLOCK_SIZE+page_No,address_in_page,buf_len);
    if(result == SPI_NAND_SUCCESS){
//        memcpy(buffer, tem_buffer, buf_len);
        return SPI_NAND_SUCCESS;
    }
    else if (result == SPI_NAND_ECC5)//5bits ECC triggered, back up data
    {   
        ori_block = replace_block;
        des_block = select_unmapped_block();
        swap_L2P(block_No, des_block);
				#ifdef LOG
            printf("\n***************ECC TRIGGER. MOVE DATA FROM BLOCK: %d TO BLOCK: %d******************\n", ori_block, des_block);
            //while (1);
        #endif
        //L2P[block_No] = des_block;//update the L2P table
        for(i = 0; i < SPI_NAND_BLOCK_SIZE; i++){//move current block to a new block
			move_page_data(des_block,ori_block,i);
	    }
		/*TODO: MARK AS BAD BLOCK*/	
        //update L2P BST TO NAND
        update_L2PBST_to_nand();
        
        return SPI_NAND_ECC5;
    }
    
    else if(result == SPI_NAND_FAIL){ 
        #ifdef LOG
            printf("*********READ FAIL**********\nPB: %X\n",replace_block);
		#endif 
        return SPI_NAND_FAIL;
    }
}
#else
uint8_t nandflash_page_read(uint8_t *buffer, uint16_t block_No, uint8_t page_No, uint16_t address_in_page, uint16_t buf_len)
{
    uint16_t replace_block;     
    if((block_No > USER_AREA_END)||(page_No>=SPI_NAND_BLOCK_SIZE)||(buf_len>SPI_NAND_PAGE_SIZE)){
        return SPI_NAND_FAIL;
    }
    
    if(check_whether_in_DBT_array(block_No)==true){
        replace_block = get_replace_block_from_ram(block_No);
    }else{
        replace_block = block_No;
    }
    
    if(spi_nandflash_read_data(buffer,replace_block*SPI_NAND_BLOCK_SIZE+page_No,address_in_page,buf_len) == SPI_NAND_SUCCESS){
//        memcpy(buffer, tem_buffer, buf_len);
        return SPI_NAND_SUCCESS;
    }else{ 
        return SPI_NAND_FAIL;
    }
}
#endif

/*!
    \brief      erase the block data of SPI nandflash
    \param[in]  block_No: the address in nandflash block(0~799)
    \param[out] none
    \retval     SPI_NAND_FAIL,SPI_NAND_SUCCESS 
*/
#ifdef NFTL_TEST
uint8_t nandflash_block_erase(uint16_t block_No)
{
    uint8_t result;
    uint16_t replace_block;
    uint16_t proper_block;
    uint16_t physical_block, new_replace_block;

    physical_block = L2P[block_No];
Start_Erase:
#ifdef DEBUG
    //printf("\rnandflash_page_erase:%x\n",block_No);
#endif
    if(block_No > USER_AREA_END){
#ifdef DEBUG
        printf("\rBlock error:%x\n",block_No);
#endif
        return SPI_NAND_FAIL;
    }    
    
    if(true==check_whether_in_DBT_array(physical_block)){//if the lba is a bad block

        replace_block = get_replace_block_from_ram(block_No);
		#ifdef LOG
            printf("*********Bad Block**********\nBad block detected: %x Original PB: %X Updated PB: %X\n",block_No, physical_block, replace_block);
		#endif
        //swap_L2P(block_No, replace_block);
        set_mapped_physical_block(block_No, replace_block);//update the L2P table
    }else{
        replace_block = physical_block;
    }
		#ifdef LOG
			printf("*********ERASE LB:%d PB:%d**********\n",block_No, replace_block);
		#endif
    result = spi_nandflash_block_erase(replace_block);
    if(result == SPI_NAND_FAIL){   
        #ifdef LOG
            printf("*********ERASE FAIL**********\nPB: %X\n",replace_block);
		#endif  
        update_DBTRBT_array(block_No,replace_block);
        mark_bad_block(replace_block);
        re_mapping_RBT(block_No,replace_block);
        update_DBTRBT_to_nand(TYPE_DBT);
        update_DBTRBT_to_nand(TYPE_RBT); 
        new_replace_block = get_replace_block_from_ram(block_No);
        swap_L2P(block_No, new_replace_block);
        goto Start_Erase;
    }
#ifdef DEBUG
    printf("erase result:%x\n",result);
#endif
    //UPDATE ABT & PST IF ERASE SUCCESS
    if(result == SPI_NAND_SUCCESS){
        ABT[replace_block] = ABT[replace_block] + 1; //PHYSICAL BLOCK ERASE CNT + 1
        ABT_CNT++;//COUNTER ++
        //PST[replace_block] = 0; //CLEAR PAGE STATUS TO UNWRITTEN
        BST[replace_block] = EMPTY;//CLEAR BLOCK STATUS TO EMPTY
        BST_CNT++;
    }
    //CHECK IF NEED TO UPDATE BST&ABT
    if(ABT_CNT > 10){
        #ifdef LOG
            printf("************Upate ABT to array********\n");
		#endif
        update_ABT_to_nand();
        ABT_CNT = 0;
    }

    if(BST_CNT > 64){
        #ifdef LOG
            printf("************Upate BSTL2P to array********\n");
		#endif
        update_L2PBST_to_nand();
        BST_CNT = 0;
    }
    return result;
}

#else
uint8_t nandflash_block_erase(uint16_t block_No)
{
    uint8_t result;
    uint16_t replace_block;
Start_Erase:
#ifdef DEBUG
    //printf("\rnandflash_page_erase:%x\n",block_No);
#endif
    if(block_No > USER_AREA_END){
#ifdef DEBUG
        printf("\rBlock error:%x\n",block_No);
#endif
        return SPI_NAND_FAIL;
    }    
    
    if(true==check_whether_in_DBT_array(block_No)){
        replace_block = get_replace_block_from_ram(block_No);
    }else{
        replace_block = block_No;
    }
    result = spi_nandflash_block_erase(replace_block);
    if(result == SPI_NAND_FAIL){     
        update_DBTRBT_array(replace_block);
        re_mapping_RBT(block_No,replace_block);
        update_DBTRBT_to_nand(TYPE_DBT);
        update_DBTRBT_to_nand(TYPE_RBT); 
        goto Start_Erase;
    }
#ifdef DEBUG
    printf("erase result:%x\n",result);
#endif
    return result;
}
#endif

/*!
    \brief      load DBT and RBT block from NAND flash
    \param[in]  type:need load table type
    \param[out] none
    \retval     none
*/
static void load_DBTRBT_from_nand(uint8_t type)
{
    //printf("\r%s:%x\n",__func__,type);
    /* load DBT table */
    if(type == TYPE_DBT){
        spi_nandflash_read_data(tem_buffer,g16DBT_Block[g8DBT_CRT_Block_Idx]*SPI_NAND_BLOCK_SIZE+g8DBT_CRT_Page,0,SPI_NAND_PAGE_SIZE);
        memcpy(DBT,tem_buffer,DBT_SIZE*sizeof(DBT[0]));
    }

    /* load RBT table */
    if(type == TYPE_RBT){
        spi_nandflash_read_data(tem_buffer,g16RBT_Block[g8RBT_CRT_Block_Idx]*SPI_NAND_BLOCK_SIZE+g8RBT_CRT_Page,0,SPI_NAND_PAGE_SIZE);
        memcpy(RBT,tem_buffer,RBT_SIZE*sizeof(RBT[0]));
    }

    /* load ENV table */
    if(type == TYPE_ENV){
        spi_nandflash_read_data(tem_buffer,g16ENV_Block[g8ENV_CRT_Block_Idx]*SPI_NAND_BLOCK_SIZE+g8ENV_CRT_Page,0,SPI_NAND_PAGE_SIZE);
        memcpy(ENV,tem_buffer,ENV_SIZE*sizeof(ENV[0]));
    }
}

/*!
    \brief      load ABT block from NAND flash
    \param[in]  type:need load table type
    \param[out] none
    \retval     none
*/
static void load_ABT_from_nand()
{
    spi_nandflash_read_data(tem_buffer,g16ABT_Block[g8ABT_CRT_Block_Idx]*SPI_NAND_BLOCK_SIZE+g8ABT_CRT_Page,0,SPI_NAND_PAGE_SIZE);
		//spi_nandflash_read_data(tem_buffer,g16ABT_Block[g8ABT_CRT_Block_Idx]*SPI_NAND_BLOCK_SIZE+0,0,SPI_NAND_PAGE_SIZE);
    memcpy(ABT,tem_buffer,TOTAL_BLOCK*sizeof(ABT[0]));
}

/*!
    \brief      load L2P&BST block from NAND flash
    \param[in]  type:need load table type
    \param[out] none
    \retval     none
*/
 void load_L2PBST_from_nand()
{
    spi_nandflash_read_data(tem_buffer,g16L2PBST_Block[g8DL2PBST_CRT_Block_Idx]*SPI_NAND_BLOCK_SIZE+g8DL2PBST_CRT_Page,0,SPI_NAND_PAGE_SIZE);
    memcpy(L2P,tem_buffer,RBT_SIZE*sizeof(L2P[0]));

    //spi_nandflash_read_data(tem_buffer,g16L2PBST_Block[g8DL2PBST_CRT_Block_Idx]*SPI_NAND_BLOCK_SIZE+g8ABT_CRT_Page,0,SPI_NAND_PAGE_SIZE);
    memcpy(BST_compressed,tem_buffer + 1600,128*sizeof(BST_compressed[0]));
    decompress_BST(BST_compressed, BST, TOTAL_BLOCK);

}

/*!
    \brief      check the block if in DBT table
    \param[in]  BlockNo:block number 
    \param[out] none
    \retval     m8Result: if block in DBT
*/
static uint8_t check_whether_in_DBT_array(uint16_t BlockNo)
{
    uint8_t                 m8Result = false;
    uint16_t                m16OldDB;
    uint16_t                m16ODBNum = DBT[0];
    uint16_t                m16Idx;
#ifdef DEBUG
    //printf("\r%s:%x\n",__func__,BlockNo);
#endif
    for ( m16Idx = 0; m16Idx < m16ODBNum; m16Idx++ ) {
        m16OldDB = DBT[2 + m16Idx];
        if ( m16OldDB == BlockNo ) {
            m8Result = true;
            break;
        }
    }   
    return m8Result;
}


/*!
    \brief      update ABT block to NAND flash
    \param[in]  type:need load table type
    \param[out] none
    \retval     m8Result: update action if success or not
*/
static uint8_t update_ABT_to_nand()
{
    uint16_t  BlockNo;
    uint16_t  PageNo;           
    uint8_t   ABT_Erase_Flag;
    uint8_t   result;
#ifdef DEBUG
    //printf("\r%s:%x\n",__func__,type);
#endif
    /* update ABT table */
       
        if((g8ABT_CRT_Page >= (SPI_NAND_BLOCK_SIZE-1))&&(g8ABT_CRT_Page!=0xFF)){
            g8ABT_CRT_Block_Idx = 1 - g8ABT_CRT_Block_Idx;  //toggle block idx
            g8ABT_CRT_Page = 0;
            ABT_Erase_Flag = true;
        }
        else{
            if(g8ABT_CRT_Page==0xFF){
                g8ABT_CRT_Page=0;
            }
            else{
                g8ABT_CRT_Page++;
            }
            ABT_Erase_Flag = false;
        }   
        BlockNo = g16ABT_Block[g8ABT_CRT_Block_Idx];
        PageNo = g8ABT_CRT_Page;
        memset(tem_buffer,0xFF,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        memcpy(tem_buffer,ABT,TOTAL_BLOCK*sizeof(ABT[0]));
#ifdef DEBUG
        //printf("\rSPI_NAND_PAGE_SIZE=%x\n",SPI_NAND_PAGE_SIZE);
#endif
        tem_buffer[SPI_NAND_PAGE_SIZE+4+0]= 'A';    //0x44
        tem_buffer[SPI_NAND_PAGE_SIZE+4+1]= 'B';    //0x42
        tem_buffer[SPI_NAND_PAGE_SIZE+4+2]= 'T';    //0x54
        tem_buffer[SPI_NAND_PAGE_SIZE+4+3]= '!';    //0x21
        tem_buffer[SPI_NAND_PAGE_SIZE+4+4]= (g16ABT_Block[1-g8ABT_CRT_Block_Idx]&0xFF);         //record the pair block No
        tem_buffer[SPI_NAND_PAGE_SIZE+4+5]= (g16ABT_Block[1-g8ABT_CRT_Block_Idx]&0xFF00)>>8;        
        result = spi_nandflash_write_data(tem_buffer,BlockNo*SPI_NAND_BLOCK_SIZE+PageNo,0,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        if(result==SPI_NAND_FAIL){
            update_DBTRBT_array(BlockNo, BlockNo);
            table_block_replacement(TYPE_ABT);
            return result;
        }
        else{
            BST[BlockNo] = NOT_EMPTY;
        }
        if(ABT_Erase_Flag == true){
            spi_nandflash_block_erase(g16ABT_Block[1-g8ABT_CRT_Block_Idx]);
            BST[g16ABT_Block[1-g8ABT_CRT_Block_Idx]] = EMPTY;
        }

    return result;
}

/*!
    \brief      update L2P&BST block to NAND flash
    \param[in]  type:need load table type
    \param[out] none
    \retval     m8Result: update action if success or not
*/
 uint8_t update_L2PBST_to_nand()
{
    uint16_t  BlockNo;
    uint16_t  PageNo;           
    uint8_t   L2PBST_Erase_Flag;
    uint8_t   result;
#ifdef DEBUG
    //printf("\r%s:%x\n",__func__,type);
#endif
    /* update DBT table */
        
        if((g8DL2PBST_CRT_Page >= (SPI_NAND_BLOCK_SIZE-1))&&(g8DL2PBST_CRT_Page!=0xFF)){
            g8DL2PBST_CRT_Block_Idx = 1 - g8DL2PBST_CRT_Block_Idx;  //toggle block idx
            g8DL2PBST_CRT_Page = 0;
            L2PBST_Erase_Flag = true;
        }
        else{
            if(g8DL2PBST_CRT_Page==0xFF){
                g8DL2PBST_CRT_Page=0;
            }
            else{
                g8DL2PBST_CRT_Page++;
            }
            L2PBST_Erase_Flag = false;
        }   
        BlockNo = g16L2PBST_Block[g8DL2PBST_CRT_Block_Idx];
        PageNo = g8DL2PBST_CRT_Page;
        memset(tem_buffer,0xFF,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        memcpy(tem_buffer,L2P,800*sizeof(L2P[0]));//COPY L2P TO TEM_BUFFER
        memcpy(tem_buffer+1600,BST_compressed,128*sizeof(BST_compressed[0]));//COPY L2P TO TEM_BUFFER
#ifdef DEBUG
        //printf("\rSPI_NAND_PAGE_SIZE=%x\n",SPI_NAND_PAGE_SIZE);
#endif
        tem_buffer[SPI_NAND_PAGE_SIZE+4+0]= 'B';    //0x44
        tem_buffer[SPI_NAND_PAGE_SIZE+4+1]= 'S';    //0x42
        tem_buffer[SPI_NAND_PAGE_SIZE+4+2]= 'T';    //0x54
        tem_buffer[SPI_NAND_PAGE_SIZE+4+3]= '!';    //0x21
        tem_buffer[SPI_NAND_PAGE_SIZE+4+4]= (g16L2PBST_Block[1-g8DL2PBST_CRT_Block_Idx]&0xFF);         //record the pair block No
        tem_buffer[SPI_NAND_PAGE_SIZE+4+5]= (g16L2PBST_Block[1-g8DL2PBST_CRT_Block_Idx]&0xFF00)>>8; 
        #ifdef LOG
        printf("L2P PBLOCK: %d PAGE: %d\n", BlockNo, PageNo);		
        #endif       
        result = spi_nandflash_write_data(tem_buffer,BlockNo*SPI_NAND_BLOCK_SIZE+PageNo,0,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        if(result==SPI_NAND_FAIL){
            update_DBTRBT_array(BlockNo, BlockNo);
            table_block_replacement(TYPE_L2PBST);
            return result;
        }
        else{
            BST[BlockNo] = NOT_EMPTY;
        }
        if(L2PBST_Erase_Flag == true){
            spi_nandflash_block_erase(g16L2PBST_Block[1-g8DL2PBST_CRT_Block_Idx]);
            BST[g16L2PBST_Block[1-g8DL2PBST_CRT_Block_Idx]] = EMPTY;
        }
    

    return result;
}

/*!
    \brief      update DBT and RBT block to NAND flash
    \param[in]  type:need load table type
    \param[out] none
    \retval     m8Result: update action if success or not
*/
uint8_t update_DBTRBT_to_nand(uint8_t type)
{
    uint16_t  BlockNo;
    uint16_t  PageNo;           
    uint8_t   DBT_Erase_Flag,RBT_Erase_Flag,ENV_Erase_Flag;
    uint8_t   result;
#ifdef DEBUG
    printf("\r%s:%x\n",__func__,type);
#endif
    /* update DBT table */
    if(type==TYPE_DBT)
    {       
        if((g8DBT_CRT_Page >= (SPI_NAND_BLOCK_SIZE-1))&&(g8DBT_CRT_Page!=0xFF)){
            g8DBT_CRT_Block_Idx = 1 - g8DBT_CRT_Block_Idx;  //toggle block idx
            g8DBT_CRT_Page = 0;
            DBT_Erase_Flag = true;
        }
        else{
            if(g8DBT_CRT_Page==0xFF){
                g8DBT_CRT_Page=0;
            }
            else{
                g8DBT_CRT_Page++;
            }
            DBT_Erase_Flag = false;
        }   
        BlockNo = g16DBT_Block[g8DBT_CRT_Block_Idx];
        PageNo = g8DBT_CRT_Page;
        memset(tem_buffer,0xFF,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        memcpy(tem_buffer,DBT,DBT_SIZE*sizeof(DBT[0]));
#ifdef DEBUG
        //printf("\rSPI_NAND_PAGE_SIZE=%x\n",SPI_NAND_PAGE_SIZE);
#endif
        tem_buffer[SPI_NAND_PAGE_SIZE+4+0]= 'D';    //0x44
        tem_buffer[SPI_NAND_PAGE_SIZE+4+1]= 'B';    //0x42
        tem_buffer[SPI_NAND_PAGE_SIZE+4+2]= 'T';    //0x54
        tem_buffer[SPI_NAND_PAGE_SIZE+4+3]= '!';    //0x21
        tem_buffer[SPI_NAND_PAGE_SIZE+4+4]= (g16DBT_Block[1-g8DBT_CRT_Block_Idx]&0xFF);         //record the pair block No
        tem_buffer[SPI_NAND_PAGE_SIZE+4+5]= (g16DBT_Block[1-g8DBT_CRT_Block_Idx]&0xFF00)>>8;        
        result = spi_nandflash_write_data(tem_buffer,BlockNo*SPI_NAND_BLOCK_SIZE+PageNo,0,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        if(result==SPI_NAND_FAIL){
            update_DBTRBT_array(BlockNo, BlockNo);
            table_block_replacement(TYPE_DBT);
            return result;
        }
        else{
            BST[BlockNo] = NOT_EMPTY;
        }
        if(DBT_Erase_Flag == true){
            spi_nandflash_block_erase(g16DBT_Block[1-g8DBT_CRT_Block_Idx]);
            BST[g16DBT_Block[1-g8DBT_CRT_Block_Idx]] = EMPTY;
        }
    }

    /* update RBT table */
    if(type==TYPE_RBT)
    {
        if((g8RBT_CRT_Page >= (SPI_NAND_BLOCK_SIZE-1))&&(g8RBT_CRT_Page!=0xFF)){
            g8RBT_CRT_Block_Idx = 1 - g8RBT_CRT_Block_Idx;  //toggle block idx
            g8RBT_CRT_Page = 0;
            RBT_Erase_Flag = true;
        }
        else{
            if(g8RBT_CRT_Page==0xFF){
                g8RBT_CRT_Page=0;
            }
            else{
                g8RBT_CRT_Page++;
            }
            RBT_Erase_Flag = false;
        }
        BlockNo = g16RBT_Block[g8RBT_CRT_Block_Idx];
        PageNo = g8RBT_CRT_Page;    
        memset(tem_buffer,0xFF,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        memcpy(tem_buffer,RBT,RBT_SIZE*sizeof(RBT[0]));
        tem_buffer[SPI_NAND_PAGE_SIZE+4+0]= 'R';    //0x52
        tem_buffer[SPI_NAND_PAGE_SIZE+4+1]= 'B';    //0x42
        tem_buffer[SPI_NAND_PAGE_SIZE+4+2]= 'T';    //0x54
        tem_buffer[SPI_NAND_PAGE_SIZE+4+3]= '!';    //0x21
        tem_buffer[SPI_NAND_PAGE_SIZE+4+4]= (g16RBT_Block[1-g8RBT_CRT_Block_Idx]&0xFF);         //record the pair block No
        tem_buffer[SPI_NAND_PAGE_SIZE+4+5]= (g16RBT_Block[1-g8RBT_CRT_Block_Idx]&0xFF00)>>8;
        result = spi_nandflash_write_data(tem_buffer,BlockNo*SPI_NAND_BLOCK_SIZE+PageNo,0,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        if(result==SPI_NAND_FAIL){
            update_DBTRBT_array(BlockNo, BlockNo);
            table_block_replacement(TYPE_RBT);
            return result;
        }   
        else{
            BST[BlockNo] = NOT_EMPTY;
        }    
        if(RBT_Erase_Flag == true){
            spi_nandflash_block_erase( g16RBT_Block[1-g8RBT_CRT_Block_Idx]);
            BST[g16RBT_Block[1-g8RBT_CRT_Block_Idx]] = EMPTY;
        }
    }

    /* update ENV table */
    if(type==TYPE_ENV)
    {
        if((g8ENV_CRT_Page >= (SPI_NAND_BLOCK_SIZE-1))&&(g8ENV_CRT_Page!=0xFF)){
            g8ENV_CRT_Block_Idx = 1 - g8ENV_CRT_Block_Idx;  //toggle block idx
            g8ENV_CRT_Page = 0;
            ENV_Erase_Flag = true;
        }
        else{
            if(g8ENV_CRT_Page==0xFF){
                g8ENV_CRT_Page=0;
            }
            else{
                g8ENV_CRT_Page++;
            }
            ENV_Erase_Flag = false;
        }
        BlockNo = g16ENV_Block[g8ENV_CRT_Block_Idx];
        PageNo = g8ENV_CRT_Page;    
        memset(tem_buffer,0xFF,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        memcpy(tem_buffer,ENV,ENV_SIZE*sizeof(ENV[0]));
        tem_buffer[SPI_NAND_PAGE_SIZE+4+0]= 'E';    //0x45
        tem_buffer[SPI_NAND_PAGE_SIZE+4+1]= 'N';    //0x4E
        tem_buffer[SPI_NAND_PAGE_SIZE+4+2]= 'V';    //0x56
        tem_buffer[SPI_NAND_PAGE_SIZE+4+3]= '!';    //0x21
        tem_buffer[SPI_NAND_PAGE_SIZE+4+4]= (g16ENV_Block[1-g8ENV_CRT_Block_Idx]&0xFF);         //record the pair block No
        tem_buffer[SPI_NAND_PAGE_SIZE+4+5]= (g16ENV_Block[1-g8ENV_CRT_Block_Idx]&0xFF00)>>8;
        result = spi_nandflash_write_data(tem_buffer,BlockNo*SPI_NAND_BLOCK_SIZE+PageNo,0,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        if(result==SPI_NAND_FAIL){
            update_DBTRBT_array(BlockNo, BlockNo);
            table_block_replacement(TYPE_ENV);
            return result;
        }
        else{
            BST[BlockNo] = NOT_EMPTY;
        }       
        if(ENV_Erase_Flag == true){
            spi_nandflash_block_erase( g16ENV_Block[1-g8ENV_CRT_Block_Idx]);
            BST[g16ENV_Block[1-g8ENV_CRT_Block_Idx]] = EMPTY;
        }
    }
    return result;
}



/*!
    \brief      add bad block to ram screen
    \param[in]  BlockNo:block number
    \param[out] none
    \retval     none
*/
static void add_bad_Block_to_DBTRBT_ram(uint16_t BlockNo, uint16_t PhysicalBlockNo)
{
#ifdef DEBUG
    //printf("\r%s:%x\n",__func__,BlockNo);
#endif
//    if((USER_AREA_START<=BlockNo)&&(BlockNo<=USER_AREA_END)){
    if(BlockNo<=USER_AREA_END){
        RBT[BlockNo] = g16LastValidBlock;
        g16LastValidBlock--;
    }
    if(g16LastValidBlock <= ReplaceBlock_AREA_START){
        while(1);
    }
    g16BadBlockNum++; 
    DBT[0] = g16BadBlockNum;        //bad block number
    DBT[1] = g16LastValidBlock;     //record last valid block
    DBT[1+g16BadBlockNum] = PhysicalBlockNo;
}

/*!
    \brief      re mapping RBT Table when the replace block is bad when running.
    \param[in]  ori_BlockNo:block number before mapping;old replace BlockNo:block number before mapping
    \param[out] none
    \retval     none
*/
static uint16_t re_mapping_RBT(uint16_t ori_BlockNo,uint16_t old_replace_BlockNo)
{
#ifdef DEBUG
    //printf("\r%s\n",__func__);
#endif
    if(0==((ori_BlockNo<=USER_AREA_END)&&(ReplaceBlock_AREA_START<=old_replace_BlockNo)&&(old_replace_BlockNo<=ReplaceBlock_AREA_END))){
        return false;
    }
    if(RBT[ori_BlockNo] != old_replace_BlockNo){
        return false;
    }
	if(ori_BlockNo == old_replace_BlockNo){
        return true;
    }
    RBT[ori_BlockNo] = g16LastValidBlock;
    g16LastValidBlock--;

    if(g16LastValidBlock <= ReplaceBlock_AREA_START){
        /* infinity loop */
        while(1);
    }
    
    DBT[1] = g16LastValidBlock; 
    return true;
}

/*!
    \brief      MANAGE BLOCK REPLACEMENT WHEN ECC ERROR OR FAIL IN TABLE AREA
    \param[in]  type:need load table type
    \param[out] none
    \retval     
*/
static uint8_t table_block_replacement(uint8_t type){
    if (type == TYPE_DBT)
    {   
        if(check_whether_in_DBT_array(table_area_lastvalid) == false)
        {
            mark_bad_block(g16DBT_Block[g8DBT_CRT_Block_Idx]);// MARK AS BAD
            g16DBT_Block[g8DBT_CRT_Block_Idx] = table_area_lastvalid;
            table_area_lastvalid--;
            g8DBT_CRT_Page = 0x00;
        }
    }
    else if (type == TYPE_ABT)
    {   
        if(check_whether_in_DBT_array(table_area_lastvalid) == false)
        {
            mark_bad_block(g16ABT_Block[g8ABT_CRT_Block_Idx]);
            g16ABT_Block[g8ABT_CRT_Block_Idx] = table_area_lastvalid;
            table_area_lastvalid --;
            g8ABT_CRT_Page = 0x00;
        }
    }
    else if (type == TYPE_L2PBST)
    {   
        if(check_whether_in_DBT_array(table_area_lastvalid) == false)
        {   
            
            #ifdef LOG
                printf("L2P TABLE BLOCK FAIL. REPLACING FROM %d TO %d", g16L2PBST_Block[g8DL2PBST_CRT_Block_Idx], table_area_lastvalid);
            #endif
            mark_bad_block(g16L2PBST_Block[g8DL2PBST_CRT_Block_Idx]);
            g16L2PBST_Block[g8DL2PBST_CRT_Block_Idx] = table_area_lastvalid;
            table_area_lastvalid --;
            g8DL2PBST_CRT_Page = 0x00;
        }
    }
    else if (type == TYPE_RBT)
    {   
        if(check_whether_in_DBT_array(table_area_lastvalid) == false)
        {
            mark_bad_block(g16RBT_Block[g8RBT_CRT_Block_Idx]);
            g16RBT_Block[g8RBT_CRT_Block_Idx] = table_area_lastvalid;
            table_area_lastvalid --;
            g8RBT_CRT_Page = 0x00;
        }
    }
    else if (type == TYPE_ENV)
    {   
        if(check_whether_in_DBT_array(table_area_lastvalid) == false)
        {
            mark_bad_block(g16ENV_Block[g8ENV_CRT_Block_Idx]);
            g16ENV_Block[g8ENV_CRT_Block_Idx] = table_area_lastvalid;
            table_area_lastvalid --;
            g8ENV_CRT_Page = 0x00;
        }
    }
 
}


/*!
    \brief      move page data(include spare area) from source block to destination block.
    \param[in]  des_block_No:block number before mapping;old replace BlockNo:block number before mapping
    \param[out] none
    \retval     none
*/
static uint8_t move_page_data(uint16_t des_block_No,uint16_t src_block_No,uint8_t page_No)
{
    uint8_t result;
    result = spi_nandflash_read_data(tem_buffer,src_block_No*SPI_NAND_BLOCK_SIZE+page_No,0,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
    if(SPI_NAND_FAIL == result){
        return result;
    }
    result = spi_nandflash_write_data(tem_buffer,des_block_No*SPI_NAND_BLOCK_SIZE+page_No,0,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
    BST[des_block_No] = NOT_EMPTY;
		return result;
}

/*!
    \brief      get replace block from the tabe on the ram
    \param[in]  BlockNo:block number
    \param[out] none
    \retval     none
*/
static uint16_t get_replace_block_from_ram(uint16_t BlockNo)
{
    return RBT[BlockNo];
}

/*!
    \brief      add new block number to DBT array
    \param[in]  BlockNo:block number
    \param[out] none
    \retval     result:update success or not
*/
static uint8_t update_DBTRBT_array(uint16_t BlockNo, uint16_t PhysicalBlockNo)
{
    uint8_t   result = true;
#ifdef DEBUG
    //printf("\r%s:%x\n",__func__,BlockNo);
#endif
    result = check_whether_in_DBT_array(PhysicalBlockNo);
    if ( result==false ) {
        add_bad_Block_to_DBTRBT_ram(BlockNo, PhysicalBlockNo);
    }
    return result;
}


/*!
    \brief      allocate the blocks to save bad block mapping table
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void alloc_DBTRBTABTPST_block_addr(void)
{
    uint16_t BlockNo = Table_AREA_END;
    uint8_t  Idx;
		uint16_t i;
    g16BadBlockNum = 0;
    memset(DBT,0,DBT_SIZE);
    memset(RBT,0,RBT_SIZE);
    memset(ABT, 0, sizeof(ABT));
    //memset(PST, 0, sizeof(PST));
    //memset(BST,0,TOTAL_BLOCK);//Initialize block status as unwritten
    //init L2P TABLE L2P[1] = 1(BLOCK1)....
	#ifdef TEST
        ABT[0x3B5] = 10;
        ABT[0x02]  = 0x1234;
    #endif
    for (i = 0; i < RBT_SIZE; ++i) {
        L2P[i] = (uint16_t)i;
    };
    for (i = 0; i < TOTAL_BLOCK; ++i) {
        BST[i] = (bool) 0;
    };
#ifdef DEBUG
    printf("\r%s\n",__func__);
#endif
    for(Idx = 0;Idx < DBT_BLOCK_NUM;Idx++){     //alloc DBT address FIND 2 BLOCKS TO SAVE DBT
        for(; BlockNo > Table_AREA_START; BlockNo--){   //950~1023 find a good block from the end.
            if(spi_nandflash_badblock_detect(BlockNo)==0){           
                g16DBT_Block[Idx]=BlockNo;//DBT SAVE TO THIS BLOCK
                DBT_BLOCK_1 = BlockNo;//save the first DBT BLOCK NUM
#ifdef LOG
                printf("\rDBT_Block[%x]=%x\n",Idx,g16DBT_Block[Idx]);
#endif
                BlockNo--;
                break;//after the first good block is detected break.
            }
            else{               //bad block
                g16BadBlockNum ++;          
                DBT[1+g16BadBlockNum] = BlockNo;
            }
        }
    }
    g8DBT_CRT_Block_Idx = 0;
    g8DBT_CRT_Page = 0xFF;

    for(Idx = 0;Idx < DBT_BLOCK_NUM;Idx++){     //alloc ABT address FIND 2 BLOCKS TO SAVE ABT
        for(; BlockNo > Table_AREA_START; BlockNo--){   //950~1023 find a good block from the end.
            if(spi_nandflash_badblock_detect(BlockNo)==0){           
                g16ABT_Block[Idx]=BlockNo;//DBT SAVE TO THIS BLOCK
                //DBT_BLOCK_1 = BlockNo;//save the first DBT BLOCK NUM
#ifdef LOG
                printf("\rDBT_Block[%x]=%x\n",Idx,g16DBT_Block[Idx]);
#endif
                BlockNo--;
                break;//after the first good block is detected break.
            }
            else{               //bad block
                g16BadBlockNum ++;          
                DBT[1+g16BadBlockNum] = BlockNo;
            }
        }
    }

    g8ABT_CRT_Block_Idx = 0;
    g8ABT_CRT_Page = 0xFF;

    for(Idx = 0;Idx < DBT_BLOCK_NUM;Idx++){     //alloc L2P address FIND 2 BLOCKS TO SAVE ABT
        for(; BlockNo > Table_AREA_START; BlockNo--){   //950~1023 find a good block from the end.
            if(spi_nandflash_badblock_detect(BlockNo)==0){           
                g16L2PBST_Block[Idx]=BlockNo;//DBT SAVE TO THIS BLOCK
                //DBT_BLOCK_1 = BlockNo;//save the first DBT BLOCK NUM
#ifdef DEBUG
                printf("\rDBT_Block[%x]=%x\n",Idx,g16DBT_Block[Idx]);
#endif
                BlockNo--;
                break;//after the first good block is detected break.
            }
            else{               //bad block
                g16BadBlockNum ++;          
                DBT[1+g16BadBlockNum] = BlockNo;
            }
        }
    }

    g8DL2PBST_CRT_Block_Idx = 0;
    g8DL2PBST_CRT_Page = 0xFF;

    for(Idx = 0;Idx < RBT_BLOCK_NUM;Idx++){     //alloc RBT address
        for(; BlockNo > Table_AREA_START; BlockNo--){
            if(spi_nandflash_badblock_detect(BlockNo)==0){
                g16RBT_Block[Idx]=BlockNo;
#ifdef LOG
                printf("\rRBT_Block[%x]=%x\n",Idx,g16RBT_Block[Idx]);
#endif
                BlockNo--;
                break;
            }
            else{
                g16BadBlockNum ++;
                DBT[1+g16BadBlockNum] = BlockNo;
            }
        }
    }
    g8RBT_CRT_Block_Idx = 0;
    g8RBT_CRT_Page = 0xFF;

    for(Idx = 0;Idx < RBT_BLOCK_NUM;Idx++){     //alloc RBT address
        for(; BlockNo > Table_AREA_START; BlockNo--){
            if(spi_nandflash_badblock_detect(BlockNo)==0){
                g16ENV_Block[Idx]=BlockNo;
#ifdef LOG
                printf("\rENV_Block[%x]=%x\n",Idx,g16ENV_Block[Idx]);
#endif
                BlockNo--;
                break;
            }
            else{
                g16BadBlockNum ++;
                DBT[1+g16BadBlockNum] = BlockNo;
            }
        }
    }
    g8ENV_CRT_Block_Idx = 0;
    g8ENV_CRT_Page = 0xFF;
}

/*!
    \brief      init bad block table by scan nand flash
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void init_build_DBTRBTABTPST(void)
{
    uint16_t BlockNo = 0;
#ifdef DEBUG
    //printf("\r%s\n",__func__);    
#endif
    g16LastValidBlock = ReplaceBlock_AREA_END;
    for(BlockNo = 0;BlockNo <= USER_AREA_END;BlockNo++){
        if( g16LastValidBlock < ReplaceBlock_AREA_START ){  
            break;
        }   
        if(spi_nandflash_badblock_detect(BlockNo)==1){
            while(spi_nandflash_badblock_detect(g16LastValidBlock)==1){
                g16BadBlockNum++;
                DBT[1+g16BadBlockNum] = g16LastValidBlock;
                g16LastValidBlock--;                
            }
            add_bad_Block_to_DBTRBT_ram(BlockNo, BlockNo);
            continue;
        }
        else{
            RBT[BlockNo] = BlockNo;
        }
    }
    DBT[0] = g16BadBlockNum;
    DBT[1] = g16LastValidBlock;

    ENV[0] = 0xFF;//flag
	ENV[1] = 0x00;//ori block
	ENV[2] = 0x00;//des block
}

/*!
    \brief      rebuild bad block mapping table from nand flash
    \param[in]  none
    \param[out] none
    \retval     result: rebuild success or not
*/
static uint8_t rebuild_DBTRBT_array(void)
{
    uint16_t  BlockNo = SPI_NAND_BLOCK_COUNT - 1;       //1023
    uint16_t  PageNo = 0;                               //0~63
    uint8_t   Flag_DBT_Exist = false;
    uint8_t   Flag_ABT_Exist = false;
    uint8_t   Flag_BSTL2P_Exist = false;
    uint8_t   Flag_RBT_Exist = false;
    uint8_t   Flag_ENV_Exist = false;
    uint8_t   Spare_data[6];
    uint16_t  Pair_block_No;
    uint8_t   Spare_data2[6];
    uint8_t   result  = true;
    uint16_t cur_table_blocks[10];
    uint16_t min = Table_AREA_END;
    uint8_t i;
   

    //check DBT whether exist in nand
    for(BlockNo = Table_AREA_END; BlockNo >= Table_AREA_START; BlockNo--){                  //1023,1022
        PageNo = 0;
        spi_nandflash_read_spare(Spare_data,BlockNo * SPI_NAND_BLOCK_SIZE + PageNo,4,6);
        if((Spare_data[0]=='D')&&(Spare_data[1]=='B')&&(Spare_data[2]=='T')&&(Spare_data[3]=='!')){
            Flag_DBT_Exist = true;
            break;
        }
    }

    if(Flag_DBT_Exist == true){
        for(PageNo = SPI_NAND_BLOCK_SIZE - 1; (PageNo < SPI_NAND_BLOCK_SIZE); PageNo--){    //0~63
            spi_nandflash_read_spare(Spare_data,BlockNo * SPI_NAND_BLOCK_SIZE+PageNo,4,6);
            if((Spare_data[0]=='D')&&(Spare_data[1]=='B')&&(Spare_data[2]=='T')&&(Spare_data[3]=='!')){
                break;
            }
        }
        g8DBT_CRT_Block_Idx = 0;
        Pair_block_No = Spare_data[4] + (Spare_data[5]<<8);
        spi_nandflash_read_spare(Spare_data2,Pair_block_No * SPI_NAND_BLOCK_SIZE+0,4,6);
        //block�����һ��page��˵�����µ�Table��������һ��block��Page0�С�
        if(PageNo == SPI_NAND_BLOCK_SIZE - 1){  //��һ��block�����ݣ�˵��Eraseʱ������
            if((Spare_data2[0]=='D')&&(Spare_data2[1]=='B')&&(Spare_data2[2]=='T')&&(Spare_data2[3]=='!')){
                PageNo = 0;
                g8DBT_CRT_Block_Idx = 1;                
                spi_nandflash_block_erase(BlockNo);     //����(��������δ������)д����block
            }
        }
        g16DBT_Block[0] = BlockNo;
        g16DBT_Block[1] = Pair_block_No;
        g8DBT_CRT_Page = PageNo;
        load_DBTRBT_from_nand(TYPE_DBT);
        
        g16BadBlockNum = DBT[0];        //bad block number
        g16LastValidBlock = DBT[1];     //record last valid block   
    }else{
        alloc_DBTRBTABTPST_block_addr();
        init_build_DBTRBTABTPST();
        update_ABT_to_nand();
        update_L2PBST_to_nand();
        update_DBTRBT_to_nand(TYPE_DBT);
        update_DBTRBT_to_nand(TYPE_RBT);
        update_DBTRBT_to_nand(TYPE_ENV);
        cur_table_blocks[0] = g16DBT_Block[0];
        cur_table_blocks[1] = g16DBT_Block[1];
        cur_table_blocks[2] = g16ABT_Block[0];
        cur_table_blocks[3] = g16ABT_Block[1];
        cur_table_blocks[4] = g16L2PBST_Block[0];
        cur_table_blocks[5] = g16L2PBST_Block[1];
        cur_table_blocks[6] = g16RBT_Block[0];
        cur_table_blocks[7] = g16RBT_Block[1];
        cur_table_blocks[8] = g16ENV_Block[0];
        cur_table_blocks[9] = g16ENV_Block[1];
        for ( i = 0; i < 10; i++)
        {
            if(cur_table_blocks[i] < min){
                min = cur_table_blocks[i];
            }
            else{
                continue;
            }
        }
        table_area_lastvalid = min - 1;                        
        return result;
    }

    //check RBT whether exist
    for(BlockNo = Table_AREA_END; BlockNo >= Table_AREA_START; BlockNo--){                  //1023,1022
        PageNo = 0;
        spi_nandflash_read_spare(Spare_data,BlockNo * SPI_NAND_BLOCK_SIZE + PageNo,4,6);
        if((Spare_data[0]=='R')&&(Spare_data[1]=='B')&&(Spare_data[2]=='T')&&(Spare_data[3]=='!')){
            Flag_RBT_Exist = true;
            break;
        }
    }
    
    if(Flag_RBT_Exist == true){
        for(PageNo = SPI_NAND_BLOCK_SIZE - 1; (PageNo < SPI_NAND_BLOCK_SIZE); PageNo--){   //0~63
            spi_nandflash_read_spare(Spare_data,BlockNo * SPI_NAND_BLOCK_SIZE+PageNo,4,6);
            if((Spare_data[0]=='R')&&(Spare_data[1]=='B')&&(Spare_data[2]=='T')&&(Spare_data[3]=='!')){
                break;
            }
        }
        g8RBT_CRT_Block_Idx = 0;
        Pair_block_No = Spare_data[4] + (Spare_data[5]<<8);
        spi_nandflash_read_spare(Spare_data2,Pair_block_No * SPI_NAND_BLOCK_SIZE+0,4,6);
        //block�����һ��page��˵�����µ�Table��������һ��block��Page0�С�
        if(PageNo == SPI_NAND_BLOCK_SIZE - 1){  //��һ��block�����ݣ�˵��Eraseʱ������
            if((Spare_data2[0]=='R')&&(Spare_data2[1]=='B')&&(Spare_data2[2]=='T')&&(Spare_data2[3]=='!')){
                PageNo = 0;
                g8RBT_CRT_Block_Idx = 1;
            }
        }
        g16RBT_Block[0] = BlockNo;
        g16RBT_Block[1] = Pair_block_No;
        g8RBT_CRT_Page = PageNo;
        load_DBTRBT_from_nand(TYPE_RBT);
        //return result;
    }else{
        result = false;     //DBT exist but RBT not exist
        //return result;
    }   

    //check ABT whether exist in nand
    for(BlockNo = Table_AREA_END; BlockNo >= Table_AREA_START; BlockNo--){                  //1023,1022
        PageNo = 0;
        spi_nandflash_read_spare(Spare_data,BlockNo * SPI_NAND_BLOCK_SIZE + PageNo,4,6);
        if((Spare_data[0]=='A')&&(Spare_data[1]=='B')&&(Spare_data[2]=='T')&&(Spare_data[3]=='!')){
            Flag_ABT_Exist = true;
            break;
        }
    }

    if(Flag_ABT_Exist == true){
        for(PageNo = SPI_NAND_BLOCK_SIZE - 1; (PageNo < SPI_NAND_BLOCK_SIZE); PageNo--){   //0~63
            spi_nandflash_read_spare(Spare_data,BlockNo * SPI_NAND_BLOCK_SIZE+PageNo,4,6);
            if((Spare_data[0]=='A')&&(Spare_data[1]=='B')&&(Spare_data[2]=='T')&&(Spare_data[3]=='!')){
                break;
            }
        }
        g8ABT_CRT_Block_Idx = 0;
        Pair_block_No = Spare_data[4] + (Spare_data[5]<<8);
        spi_nandflash_read_spare(Spare_data2,Pair_block_No * SPI_NAND_BLOCK_SIZE+0,4,6);
        //block�����һ��page��˵�����µ�Table��������һ��block��Page0�С�
        if(PageNo == SPI_NAND_BLOCK_SIZE - 1){  //��һ��block�����ݣ�˵��Eraseʱ������
            if((Spare_data2[0]=='A')&&(Spare_data2[1]=='B')&&(Spare_data2[2]=='T')&&(Spare_data2[3]=='!')){
                PageNo = 0;
                g8ABT_CRT_Block_Idx = 1;
            }
        }
        g16ABT_Block[0] = BlockNo;
        g16ABT_Block[1] = Pair_block_No;
        g8ABT_CRT_Page = PageNo;
        load_ABT_from_nand();
        //return result;
    }else{
        result = false;     //DBT exist but RBT not exist
        //return result;
    }   

    //check BST&L2P whether exist in nand
    for(BlockNo = Table_AREA_END; BlockNo >= Table_AREA_START; BlockNo--){                  //1023,1022
        PageNo = 0;
        spi_nandflash_read_spare(Spare_data,BlockNo * SPI_NAND_BLOCK_SIZE + PageNo,4,6);
        if((Spare_data[0]=='B')&&(Spare_data[1]=='S')&&(Spare_data[2]=='T')&&(Spare_data[3]=='!')){
            Flag_BSTL2P_Exist = true;
            break;
        }
    }

    if(Flag_BSTL2P_Exist == true){
        for(PageNo = SPI_NAND_BLOCK_SIZE - 1; (PageNo < SPI_NAND_BLOCK_SIZE); PageNo--){   //0~63
            spi_nandflash_read_spare(Spare_data,BlockNo * SPI_NAND_BLOCK_SIZE+PageNo,4,6);
            if((Spare_data[0]=='B')&&(Spare_data[1]=='S')&&(Spare_data[2]=='T')&&(Spare_data[3]=='!')){
                break;
            }
        }
        g8DL2PBST_CRT_Block_Idx = 0;
        Pair_block_No = Spare_data[4] + (Spare_data[5]<<8);
        spi_nandflash_read_spare(Spare_data2,Pair_block_No * SPI_NAND_BLOCK_SIZE+0,4,6);
        //block�����һ��page��˵�����µ�Table��������һ��block��Page0�С�
        if(PageNo == SPI_NAND_BLOCK_SIZE - 1){  //��һ��block�����ݣ�˵��Eraseʱ������
            if((Spare_data2[0]=='B')&&(Spare_data2[1]=='S')&&(Spare_data2[2]=='T')&&(Spare_data2[3]=='!')){
                PageNo = 0;
                g8DL2PBST_CRT_Block_Idx = 1;
            }
        }
        g16L2PBST_Block[0] = BlockNo;
        g16L2PBST_Block[1] = Pair_block_No;
        g8DL2PBST_CRT_Page = PageNo;
        load_L2PBST_from_nand();
        //return result;
    }else{
        result = false;     //DBT exist but RBT not exist
        //return result;
    }   

    //check ENV whether exist
    for(BlockNo = Table_AREA_END; BlockNo >= Table_AREA_START; BlockNo--){                  //1023,1022
        PageNo = 0;
        spi_nandflash_read_spare(Spare_data,BlockNo * SPI_NAND_BLOCK_SIZE + PageNo,4,6);
        if((Spare_data[0]=='E')&&(Spare_data[1]=='N')&&(Spare_data[2]=='V')&&(Spare_data[3]=='!')){
            Flag_ENV_Exist = true;
            break;
        }
    }
    
    if(Flag_ENV_Exist == true){
        for(PageNo = SPI_NAND_BLOCK_SIZE - 1; (PageNo < SPI_NAND_BLOCK_SIZE); PageNo--){   //0~63
            spi_nandflash_read_spare(Spare_data,BlockNo * SPI_NAND_BLOCK_SIZE+PageNo,4,6);
            if((Spare_data[0]=='E')&&(Spare_data[1]=='N')&&(Spare_data[2]=='V')&&(Spare_data[3]=='!')){
                break;
            }
        }
        g8ENV_CRT_Block_Idx = 0;
        Pair_block_No = Spare_data[4] + (Spare_data[5]<<8);
        spi_nandflash_read_spare(Spare_data2,Pair_block_No * SPI_NAND_BLOCK_SIZE+0,4,6);
        //block�����һ��page��˵�����µ�Table��������һ��block��Page0�С�
        if(PageNo == SPI_NAND_BLOCK_SIZE - 1){  //��һ��block�����ݣ�˵��Eraseʱ������
            if((Spare_data2[0]=='E')&&(Spare_data2[1]=='N')&&(Spare_data2[2]=='V')&&(Spare_data2[3]=='!')){
                PageNo = 0;
                g8ENV_CRT_Block_Idx = 1;
            }
        }
        g16ENV_Block[0] = BlockNo;
        g16ENV_Block[1] = Pair_block_No;
        g8ENV_CRT_Page = PageNo;
        load_DBTRBT_from_nand(TYPE_ENV);
    }else{
		update_DBTRBT_to_nand(TYPE_ENV);
    }   

    cur_table_blocks[0] = g16DBT_Block[0];
    cur_table_blocks[1] = g16DBT_Block[1];
    cur_table_blocks[2] = g16ABT_Block[0];
    cur_table_blocks[3] = g16ABT_Block[1];
    cur_table_blocks[4] = g16L2PBST_Block[0];
    cur_table_blocks[5] = g16L2PBST_Block[1];
    cur_table_blocks[6] = g16RBT_Block[0];
    cur_table_blocks[7] = g16RBT_Block[1];
    cur_table_blocks[8] = g16ENV_Block[0];
    cur_table_blocks[9] = g16ENV_Block[1];
    for ( i = 0; i < 10; i++)
        {
            if(cur_table_blocks[i] < min){
                min = cur_table_blocks[i];
            }
            else{
                continue;
            }
        }
    table_area_lastvalid = min - 1;  
	return 0;
}

/*
    \brief      MARK AS BAD BLOCK---PROGRAM THE SPARE AREA OF THE 1ST PAGE OF THE BAD BLOCK AS 00h
    \param[in]  none
    \param[out] none
    \retval     result:  success or not
*/
static uint8_t mark_bad_block(uint16_t physical_block_no){
    uint8_t result = SPI_NAND_SUCCESS;
    uint8_t *buffer = 0x00;
    result = spi_nandflash_block_erase(physical_block_no);
    result = spi_nandflash_write_spare(buffer,physical_block_no*BLOCK_SIZE, 0, 1);
    return result;
}

/*!
    \brief      CHECK WHICH PHYSICAL BLOCK TO BE PROGRAMMED BASED ON WEAR LEVELING 
                (IF TARGET BLCOK ERASE COUNT-AVERAGE>3, SELECT AN EMPTY BLOCK AND REMAP ALL)
    \param[in]  none
    \param[out] none
    \retval     result: rebuild success or not
*/
static uint16_t select_proper_block(uint16_t physical_block_No){
    uint16_t average_erase_cnt = calculate_average(ABT, 1024);//get average erase cnt
    //uint16_t physical_block_no = L2P[physical_block_No];
    uint16_t physical_replace_block;
    uint16_t i;
    physical_replace_block = get_min_erase_block(physical_block_No);
    
    //physical_replace_block = get_empty_block(BST, RBT_SIZE, physical_block_No);
    return physical_replace_block;
}

/*!
    \brief     FIND AN UNMAPPED & EMPTY BLOCK
    \param[in]  none
    \param[out] none
    \retval     result: rebuild success or not
*/
uint16_t select_unmapped_block(void){
	uint16_t i = 0;
    uint16_t j = 0;
	uint16_t block = 0;
    uint16_t physical_replace_block = 0xFFFF;
    uint16_t *skip_blocks[5] = {g16DBT_Block, g16RBT_Block, g16ABT_Block, g16L2PBST_Block, g16ENV_Block};
		// check all block
    for (block = 0; block < g16LastValidBlock; block++) {
        bool is_bad_block = false;
        bool is_mapped = false;
        bool skip = false;
        
        // check bad block
        if(check_whether_in_DBT_array(block)) continue;

        for (i = 0; i < sizeof(skip_blocks); i++) {
            for ( j = 0; j < sizeof(g16DBT_Block); j++)
            {
                if (block == skip_blocks[i][j]) {
                skip = true;
                break;  // SKIP
                }               
            }  
            if (skip)
                {
                    break;
                } 
        }
        
        if (skip) continue;
        
        // check if it is mapped
        for (i = 0; i < RBT_SIZE; i++) {
            if (L2P[i] == block) {
                is_mapped = true;
                break;
            }
        }
        if (is_mapped) continue;

        // check if empty
        if(BST[block] == NOT_EMPTY) continue;
        else physical_replace_block = block;
    }
    return physical_replace_block;
}

/*!
    \brief      get average erase count
    \param[in]  none
    \param[out] none
    \retval     none
*/
static uint16_t calculate_average(uint16_t *array, uint16_t size) {
    uint32_t sum = 0;
		uint16_t average = 0;
		uint16_t i;
    for (i = 0; i < size; i++) {
        sum += array[i];
    }
		average = sum/size;
    return average;
}

/*!
    \brief      get an empty & good block
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/
static uint16_t get_empty_block_num(bool *array) {
    uint16_t i;//logical block no
    uint16_t n = 0;//record empty blk num
    uint16_t phy_blk_no;
	for (i = 0; i < RBT_SIZE; i++)
    {   
        phy_blk_no = L2P[i];
        if(BST[phy_blk_no] == EMPTY){
            n++;
            continue;
        }
    }
    return n;
}

/*!
    \brief      get an TOTAL empty & good block NUMBER
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/
static uint16_t get_empty_block(bool *array, size_t size, uint16_t ori_block) {
    uint16_t i;
	for (i = 0; i < size; i++)
    {   
        if (i == ori_block)
        {
            continue;
        }
        

        if(BST[i] == EMPTY){
            if (check_whether_in_DBT_array(i))
            {
                continue;//found an bad empty block continue loop
            }
            else{
                return i;//found good empty block
            }            
        }
    }
    return 0;
}

/*!
    \brief      get an min erase block
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/
static uint16_t get_min_erase_block(uint16_t ori_pb) {
    uint16_t i, block;
    uint16_t return_block = 0xFFFF;
    uint16_t erase_cnt_min = 0xFFFF;
    bool skip = false;
    
    

    for (block = 0; block < USER_AREA_END; block++) {   
        // Skip blocks present in the DBT (Defective Block Table)
        if (check_whether_in_DBT_array(block)) {
            continue;
        }

        // Find the block with the minimum erase count that is empty
        if (BST[block] == EMPTY) {
            if (ABT[block] < erase_cnt_min) {
                erase_cnt_min = ABT[block];
                return_block = block;
                
                // If the erase count is zero, break early
                if (erase_cnt_min == 0) {
                    break;
                }
            }
        }
    }
    if(return_block == 0xFFFF){
        #ifdef LOG
            printf("WEARLEVELING FAILED, NO EMPTY BLOCK IN USER AREA\n");
        #endif
        return_block = ori_pb;
    }
    return return_block;
}

/*!
    \brief      get the physical block from L2P
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/
uint16_t get_mapped_physical_block(uint16_t block_No) {
    return L2P[block_No];
}

/*!
    \brief      set the physical block from L2P
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/
static void set_mapped_physical_block(uint16_t block_No, uint16_t physical_block) {
    L2P[block_No] = physical_block;
    //return 0;
}



// Function to compress BST
void compress_BST(volatile bool *BST, uint8_t *BST_compressed, uint16_t total_blocks) {
		uint16_t i = 0;
    for (i = 0; i < total_blocks; i++) {
        uint16_t byte_index = i / 8;
        uint16_t bit_index = i % 8;
        if (BST[i]) {
            BST_compressed[byte_index] |= (1 << bit_index);
        } else {
            BST_compressed[byte_index] &= ~(1 << bit_index);
        }
    }
}

void decompress_BST(const uint8_t *BST_compressed, volatile bool *BST, uint16_t total_blocks) {
    uint16_t i = 0;
		for ( i = 0; i < total_blocks; i++) {
        uint16_t byte_index = i / 8;
        uint16_t bit_index = i % 8;
        BST[i] = (BST_compressed[byte_index] & (1 << bit_index)) != 0;
    }
}

/*!
    \brief      for test case only. Set ABT to certain value
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/

static void set_ABT(uint16_t block_No, uint16_t value){
    ABT[block_No] = value;
}

/*!
    \brief      for test case only. Get ABT to certain value
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/

static uint16_t get_ABT(uint16_t block_No){
    return ABT[block_No];
}

/*!
    \brief      for test case only. Set BST to certain value
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/

static void set_BST(uint16_t block_No, uint16_t value){
    BST[block_No] = value;
}

/*!
    \brief      for test case only. Get BST to certain value
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/

bool get_BST(uint16_t block_No){
    return BST[block_No];
}

static void swap_L2P(uint16_t LogicalBlockNo, uint16_t ReplacePB){
    // Get the current physical block number mapped to the given logical block number
    uint16_t originalPB = L2P[LogicalBlockNo];
    
    // Find the logical block number currently mapped to the replacement physical block
    uint16_t replaceLB;
    for (replaceLB = 0; replaceLB < USER_AREA_END; replaceLB++) {
        if (L2P[replaceLB] == ReplacePB) {
            break;
        }
    }
    
    // Swap the mapping relationships between the logical blocks
    L2P[LogicalBlockNo] = ReplacePB;
    L2P[replaceLB] = originalPB;
}

static void env_check(void)
{
	uint8_t i;
	load_DBTRBT_from_nand(TYPE_ENV);
#ifdef LOG
    printf("***********Unexpected Power Loss Checking************\n");
#endif
	if(ENV[0] == 0x00){
		nandflash_block_erase(ENV[1]);
		for(i = 0; i < 64; i++){
			move_page_data(ENV[1],ENV[2],i);
		}
	    #ifdef LOG
            printf("Unexpected Power Loss, Data Recovering...\n");
        #endif
		ENV[0] = 0xFF;
		//ENV[2] = ENV[1];
		update_DBTRBT_to_nand(TYPE_ENV);
		nandflash_block_erase(ENV[2]);
		return ;
	}
	else
		return ;
}

// Check if the ID is in the list of supported IDs
static bool is_supported_flash_id(uint16_t id) {
		int i = 0;
    for (i = 0; i < SUPPORTED_FLASH_IDS_COUNT; i++) {
        if (supported_flash_ids[i] == id) {
            return true;
        }
    }
    return false;
}

uint8_t test_env(uint16_t BlockNo)
{
    uint16_t i,enable_erase=1;
    uint8_t tem_buffer_test[SPI_NAND_PAGE_TOTAL_SIZE];
    uint8_t buffer[SPI_NAND_PAGE_TOTAL_SIZE];

    if(enable_erase==1){
        for(i=0;i<1024;i++){
            spi_nandflash_block_erase(i);
        }
    }

	
	printf("\rENV= %x  %x  %x: ", ENV[0],ENV[1],ENV[2]);

	ENV[0] = 0x00;
	ENV[1] = BlockNo;
	ENV[2] = select_unmapped_block();//empty block

	nandflash_block_erase(ENV[1]);
	for(i = 0; i < 64; i++){
			move_page_data(ENV[1],ENV[2],i);
	}//move block data from source to dest
	for(i = 0; i < SPI_NAND_PAGE_TOTAL_SIZE; i++){
		buffer[i] = i+50;
	}

	update_DBTRBT_to_nand(TYPE_ENV);

//	load_DBTRBT_from_nand(TYPE_ENV);
	
	if(SPI_NAND_FAIL == nandflash_block_erase(ENV[1])){
		printf("\n\rErase block failed!\n\r");
		while(1);
	}
	
	if(SPI_NAND_FAIL == nandflash_page_program(buffer, ENV[1], 0, 500)){
		/* if failed, retry to program */
		if(SPI_NAND_FAIL == nandflash_page_program(buffer, ENV[1], 0, 500)){
			printf("\n\rWrite page failed!\n\r");
			while(1);
		}
	}
	
	if(SPI_NAND_FAIL == nandflash_page_program(buffer, ENV[1], 1, 500)){
		/* if failed, retry to program */
		if(SPI_NAND_FAIL == nandflash_page_program(buffer, ENV[1], 1, 500)){
			printf("\n\rWrite page failed!\n\r");
			while(1);
		}
	}

	ENV[0] = 0xFF;
	ENV[2] = ENV[1];
	update_DBTRBT_to_nand(TYPE_ENV);

	return 1;
	
}


/*!
    \brief      for debug
    \param[in]  none
    \param[out] none
    \retval     none
*/
#if 0
static void printfData(uint32_t a32SAddr, uint32_t byteCnt)
{
    uint32_t m32i;
    for( m32i = 0; m32i < byteCnt; m32i++ ){
        if((m32i & 0xF) == 0){
            printf("\r%x: ", m32i);
        }
        if((m32i & 0x7) == 0){
            printf("\t");
        }
        printf("%x ", *(uint8_t *)( a32SAddr + m32i));
        if((m32i & 0xF) == 0xF){
            printf("\n");
        }
    }
}
#endif
