/*!
    \file  gd5f1gxx.h
    \brief the header file of SPI flash gd5f1gxx driver
*/

/*
    Copyright (C) 2019 GigaDevice

    2019-12-19, V1.0.0, demo for GD32F4xx
*/

#ifndef  GD5F1GXX_H
#define  GD5F1GXX_H

#include "stdint.h"
#include "gd32f4xx.h"
#include "stdbool.h"
#include "gdnftl.h"


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


#define    SPI_NAND_READID              0x9F    //RDID (Read Identification)
#define    SPI_NAND_GET_FEATURE         0x0F    //Get features
#define    SPI_NAND_SET_FEATURE         0x1F    //Set features
#define    SPI_NAND_PAGE_READ           0x13    //Array Read
#define    SPI_NAND_READ_CACHE          0x03    //Read From Cache
#define    SPI_NAND_READ_CACHE2         0x3B    //Read From Cache*2
#define    SPI_NAND_READ_CACHE4         0x6B    //Read From Cache*4
#define    SPI_NAND_WREN                0x06    //Write Enable
#define    SPI_NAND_WRDI                0x04    //Write Disable
#define    SPI_NAND_PAGE_LOAD           0x02    //Page Program Load
#define    SPI_NAND_PAGE_RAND_LOAD      0x84    //Page Program Random Input
#define    SPI_NAND_PAGE_LOAD4          0x32    //Quad IO Page Program Load
#define    SPI_NAND_PAGE_RAND_LOAD4     0x34    //Quad IO Page Program Random Input
#define    SPI_NAND_PROGRAM_EXEC        0x10    //Program Execute
#define    SPI_NAND_BLOCK_ERASE         0xD8    //BLock Erase
#define    SPI_NAND_RESET               0xFF    //Nand Reset

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
#define    SPI_NAND_ECC4                2        //NAND flash ECC exceeds limits, need to move data
#define    SPI_NAND_ECC5                3        //NAND flash ECC exceeds limits, need to move data
#define    OIP                          0x01     //Operation in progress bit
#define    WEL                          0x02     //Write enable latch bit 
#define    E_FAIL                       0x04     //Erase fail bit
#define    P_FAIL                       0x08     //Program fail bit

#define    ECCS0                        0x10     //ECCS0 bit
#define    ECCS1                        0x20     //ECCS1 bit
#define    ECCSE0                        0x10     //ECCSE0 bit
#define    ECCSE1                        0x20     //ECCSE1 bit

#define    ECC_OVERLIMIT                0  

#define    ERASE_FAIL                   2
#define    PROGRAM_FAIL                 3

#define    ERASE_OK                     0x40     //erase block success

#define    SPI_FLASH_CS_LOW()           gpio_bit_reset(SPI_CS_PORT, SPI_CS_PIN)
#define    SPI_FLASH_CS_HIGH()          gpio_bit_set(SPI_CS_PORT, SPI_CS_PIN)




uint8_t spi_nandflash_badblock_detect(uint32_t blockNo);
/* initialize SPI5 GPIO and parameter */
void spi5_init(void);
/* read a byte from the SPI flash */
uint8_t spi_nandflash_read_byte(void);
/* send a byte through the SPI interface and return the byte received from the SPI bus */
uint8_t spi_nandflash_send_byte(uint8_t byte);
/* reset nandflash */
void spi_nandflash_reset(void);
/* set nandflash register*/
void spi_nandflash_set_register(uint8_t reg,uint8_t data);
/* enable the write access to the flash */
void spi_nandflash_write_enable(void);
/* poll the status of the write in progress (wip) flag in the flash's status register */
void spi_nandflash_get_feature(uint8_t status_reg, uint8_t *status);
/* get nandflash status register flag bit */
uint8_t spi_nandflash_get_status_flag(uint8_t status_flag);
/* sned the read page command */
void spi_nandflash_page_read(uint32_t page_No);
/* send the read cache command */
void spi_nandflash_read_cache(uint8_t *buffer,uint16_t address_in_cache,uint32_t byte_cnt);
/* send the program load command */
void spi_nandflash_program_load(uint8_t *buffer,uint16_t address_in_cache,uint32_t byte_cnt);
/* send the program excute command*/
void spi_nandflash_program_execute(uint32_t page_No);
/* send program load random data command */
void spi_nandflash_pl_random_data(uint8_t *buffer,uint16_t address_in_cache,uint32_t byte_cnt);


/* initialize SPI5 and reset SPI nandflash */
void spi_nandflash_init(void);
/* read flash identification */
uint32_t spi_nandflash_read_id(void);
/* erase the nandflash blcok */
uint8_t spi_nandflash_block_erase(uint32_t block_No);
/* read the data from nandflash */
uint8_t spi_nandflash_read_data(uint8_t *buffer,uint32_t page_No,uint32_t address_in_page,uint32_t byte_cnt);
/* write the data to nandflash */
uint8_t spi_nandflash_write_data(uint8_t *buffer,uint32_t page_No,uint16_t address_in_page,uint32_t byte_cnt);
/* nandflash internal data move*/
uint8_t spi_nandflash_copy_page(uint32_t source_page,uint32_t target_page);
/* nandflash internal data move and update new data */
uint8_t spi_nandflash_copy_page_update(uint8_t *buffer,uint32_t source_page,uint32_t target_page,uint16_t address_in_page,uint32_t byte_cnt);
/* write the data to spare area */
uint8_t spi_nandflash_write_spare(uint8_t *buffer,uint32_t page_No,uint16_t spare_address,uint16_t byte_cnt);
/* read the data from spare area */
uint8_t spi_nandflash_read_spare(uint8_t *buffer,uint32_t page_No,uint16_t spare_address,uint16_t byte_cnt);
#endif /* GD5F1GXX_H */
