/*!
    \file  gd5f1gxx.h
    \brief the header file of SPI flash gd5f1gxx driver
*/

/*
    Copyright (C) 2019 GigaDevice

    2019-12-19, V1.0.0, demo for GD32F4xx
*/

#include "stdint.h"
#include "gd5f1gxx.h"
#include "stdbool.h"


/*************************** User Configuration start ************************/
/* select the SPI mode: QSPI or SPI */
//#define    QSPI_NANDFLASH
#define    SPI_NANDFLASH

/* select check page verify when programming */
#define    WRITE_PAGE_VERIFY_EN

#define    SPI_CS_CLK                   RCU_GPIOI
#define    SPI_CS_PORT                  GPIOI
#define    SPI_CS_PIN                   GPIO_PIN_8
/*************************** User Configuration end **************************/

/* SPI NAND memory parameters */
#define    SPI_NAND_PAGE_SIZE           ((uint16_t)0x0800) /* 2 * 1024 bytes per page w/o Spare Area */
#define    SPI_NAND_BLOCK_SIZE          ((uint16_t)0x0040) /* 64 pages per block */
#define    SPI_NAND_SPARE_AREA_SIZE     ((uint16_t)0x0040) /* last 64 bytes as spare area */

#define    SPI_NAND_BLOCK_COUNT         1024               /* the count of block */
#define    BLOCK_NUM_FOR_USER           800
#define    BLOCK_NUM_FOR_ReplaceBlock   150  //800~949
#define    BLOCK_NUM_FOR_Table          /*74*/   (SPI_NAND_BLOCK_COUNT - BLOCK_NUM_FOR_USER - BLOCK_NUM_FOR_ReplaceBlock) //950~1023

#define    USER_AREA_START              /*0*/     0   
#define    USER_AREA_END                /*799*/   (USER_AREA_START+BLOCK_NUM_FOR_USER-1)
#define    ReplaceBlock_AREA_START      /*800*/   (USER_AREA_END+1)   
#define    ReplaceBlock_AREA_END        /*949*/   (ReplaceBlock_AREA_START+BLOCK_NUM_FOR_ReplaceBlock-1)
#define    Table_AREA_START             /*950*/   (ReplaceBlock_AREA_END+1)
#define    Table_AREA_END               /*1023*/  (Table_AREA_START+BLOCK_NUM_FOR_Table-1)

#define    RBT_BLOCK_NUM                2
#define    DBT_BLOCK_NUM                2
#define    MAX_BAD_BLOCK_NUM            200
#define    SPI_NAND_DATA_BLOCK_COUNT    (SPI_NAND_BLOCK_COUNT - RBT_BLOCK_NUM - DBT_BLOCK_NUM)
#define    SPI_NAND_ZONE_SIZE           1024                  /* 1024 block per zone */
#define    SPI_NAND_MAX_ZONE            ((uint16_t)0x0001)    /* 1 zones of 1024 block */
#define    SPI_NAND_PAGE_TOTAL_SIZE     (SPI_NAND_PAGE_SIZE + SPI_NAND_SPARE_AREA_SIZE)   /* the total sizze of page(page size + spare area size) */

#define    BI_OFFSET                    0       // the first byte in the blcok first page's spare area ,use for mark bad block

#define    PROTECTION                   0xA0    // nandflash protection register
#define    FEATURE1                     0xB0    // nandflash feature1 register
#define    STATUS                       0xC0    // nandflash status register
#define    FEATURE2                     0xD0    // nandflash feature2 register
#define    STATUS2                      0xF0    // nandflash status register

#define    SPI_NAND_BAD_BLOCK_FLAG      0x00    // replace bad block 
#define    SPI_NAND_USED_BLOCK_FLAG     0xF0    // replace used block 

#define    DATA_BLOCK_PERCENT           98      // ratio of data block occupying effective block
#define    BAD_BALOK_TEST_CYCLE         2       // judge the bad block need the cycle erase 

#define    DUMMY_BYTE                   0xFF
#define    ADDRESS_ERRO                 0
#define    SPI_NAND_BUSY                0        //NAND flash is busy
#define    SPI_NAND_READY               1        //NAND flash is ready
#define    SPI_NAND_FAIL                0        //NAND flash operation fail flag
#define    SPI_NAND_SUCCESS             1        //NAND flash operation success flag
#define    OIP                          0x01     //Operation in progress bit
#define    WEL                          0x02     //Write enable latch bit 
#define    E_FAIL                       0x04     //Erase fail bit
#define    P_FAIL                       0x08     //Program fail bit

#define    ECCS0                        0x10     //ECCS0 bit
#define    ECCS1                        0x20     //ECCS1 bit

#define    ECC_OVERLIMIT                0  

#define    ERASE_FAIL                   2
#define    PROGRAM_FAIL                 3

#define    ERASE_OK                     0x40     //erase block success

#define    SPI_FLASH_CS_LOW()           gpio_bit_reset(SPI_CS_PORT, SPI_CS_PIN)
#define    SPI_FLASH_CS_HIGH()          gpio_bit_set(SPI_CS_PORT, SPI_CS_PIN)

#define true    1
#define false   0

#define TYPE_DBT        0
#define TYPE_RBT        1
#define TYPE_ENV        2
#define TYPE_DBT_RBT    1
#define WL_THRESHOLD 1
#define DBT_SIZE    200
#define RBT_SIZE    BLOCK_NUM_FOR_USER	//block number for users
#define ENV_SIZE    3
#define TOTAL_BLOCK 1024
#define BST_COMPRESSED_SIZE TOTAL_BLOCK/8
#define EMPTY       0
#define NOT_EMPTY   1
#define ABT_THRESHOLD 10
#define L2P_THRESHOLD 10
#define BST_THRESHOLD 10


static void load_DBTRBT_from_nand(uint8_t type);
static uint8_t check_whether_in_DBT_array(uint16_t BlockNo);
uint8_t update_DBTRBT_to_nand(uint8_t type);
static void add_bad_Block_to_DBTRBT_ram(uint16_t BlockNo);
static uint16_t re_mapping_RBT(uint16_t ori_BlockNo,uint16_t old_replace_BlockNo);
static uint8_t move_page_data(uint16_t des_block_No,uint16_t src_block_No,uint8_t page_No);
static uint16_t get_replace_block_from_ram(uint16_t BlockNo);
static uint8_t update_DBTRBT_array(uint16_t BlockNo);
static void alloc_DBTRBT_block_addr(void);
static void init_build_DBTRBT(void);
static uint8_t rebuild_DBTRBT_array(void);
static void set_mapped_physical_block(uint16_t block_No, uint16_t physical_block); 
static uint8_t update_L2PBST_to_nand(void);
static uint8_t update_ABT_to_nand(void);
static void load_ABT_from_nand(void);
static void load_L2PBST_from_nand(void);
//void compress_BST(const bool *BST, uint8_t *BST_compressed, uint16_t total_blocks);
//void decompress_BST(const uint8_t *BST_compressed, bool *BST, uint16_t total_blocks);


uint16_t get_mapped_physical_block(uint16_t block_No);
void set_ABT(uint16_t block_No, uint16_t value);
uint16_t get_ABT(uint16_t block_No);
void set_BST(uint16_t block_No, uint16_t value);
bool get_BST(uint16_t block_No);

//static uint16_t get_empty_block(bool *array);

static uint16_t calculate_average(uint16_t *array, uint16_t size);
static uint16_t select_proper_block(uint16_t physical_block_No);
uint16_t select_unmapped_block(void);

/*nandflash initialization*/
void nandflash_init(void);
/* write the page data to SPI nandflash, block_No(0~USER_AREA_END), page_No(0~SPI_NAND_BLOCK_SIZE-1), buf_len(length of buffer to be written) */
uint8_t nandflash_page_program(uint8_t *buffer, uint16_t block_No, uint8_t page_No, uint16_t buf_len);
/* read the page data from SPI nandflash, block_No(0~USER_AREA_END), page_No(0~SPI_NAND_BLOCK_SIZE-1),address_in_page(0~SPI_NAND_PAGE_SIZE-1), buf_len(length of buffer to be read) */
uint8_t nandflash_page_read(uint8_t *buffer, uint16_t block_No, uint8_t page_No, uint16_t address_in_page, uint16_t buf_len);
/* erase the block data of SPI nandflash, block_No(0~USER_AREA_END) */
uint8_t nandflash_block_erase(uint16_t block_No);

