/*!
    \file  gd25qxx.c
    \brief SPI flash gd25qxx driver
*/

/*
    Copyright (C) 2016 GigaDevice

    2016-10-19, V1.0.0, demo for GD32F4xx
*/

#include "gd5f1gxx.h"
#include "gd32f4xx.h"
#include "gd32f450i_eval.h"
#include "string.h"
#include "systick.h"
#include <stdio.h>
#include <stdbool.h>

//#define  DEBUG
//#define  TEST
#define NFTL_TEST
#define LOG
//#define DEMO//ONLY TEST CODE, ERASE ALL WHEN INIT
//#define TEST_LOAD_UPDATE_ABTL2P
static uint8_t tem_buffer[SPI_NAND_PAGE_TOTAL_SIZE];   /* the buffer of read page data */

#define true    1
#define false   0

#define TYPE_DBT        0
#define TYPE_RBT        1
#define TYPE_ENV        2
#define TYPE_DBT_RBT    1
#define WL_THRESHOLD 1
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

uint8_t spi_nandflash_badblock_detect(uint32_t blockNo);

/************************ Temporarily not visible to the public **************************/
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



/*!
    \brief      initialize SPI5 GPIO and parameter
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi5_init(void)
{
    spi_parameter_struct spi_init_struct;

    rcu_periph_clock_enable(RCU_GPIOG);
    rcu_periph_clock_enable(SPI_CS_CLK);
    rcu_periph_clock_enable(RCU_SPI5);

    /* SPI5_CLK(PG13), SPI5_MISO(PG12), SPI5_MOSI(PG14),SPI5_IO2(PG10) and SPI5_IO3(PG11) GPIO pin configuration */
    gpio_af_set(GPIOG, GPIO_AF_5, GPIO_PIN_10|GPIO_PIN_11| GPIO_PIN_12|GPIO_PIN_13| GPIO_PIN_14);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10|GPIO_PIN_11| GPIO_PIN_12|GPIO_PIN_13| GPIO_PIN_14);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_10|GPIO_PIN_11| GPIO_PIN_12|GPIO_PIN_13| GPIO_PIN_14);

    /* SPI5_CS(PI8) GPIO pin configuration */
    gpio_mode_set(GPIOI, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    /* chip select invalid */
    SPI_FLASH_CS_HIGH();

    /* SPI5 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_32;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI5, &spi_init_struct);

    /* set crc polynomial */
//    spi_crc_polynomial_set(SPI5,7);
#ifdef QSPI_NANDFLASH
    /* quad wire SPI_IO2 and SPI_IO3 pin output enable */
    qspi_io23_output_enable(SPI5);
#endif
    /* enable SPI5 */
    spi_enable(SPI5);
}

/*!
    \brief      read a byte from the SPI flash
    \param[in]  none
    \param[out] none
    \retval     byte read from the SPI flash
*/
uint8_t spi_nandflash_read_byte(void)
{
    return(spi_nandflash_send_byte(DUMMY_BYTE));
}

/*!
    \brief      send a byte through the SPI interface and return the byte received from the SPI bus
    \param[in]  byte: byte to send
    \param[out] none
    \retval     the value of the received byte
*/
uint8_t spi_nandflash_send_byte(uint8_t byte)
{
    /* loop while data register in not emplty */
    while(RESET == spi_i2s_flag_get(SPI5,SPI_FLAG_TBE));

    /* send byte through the SPI5 peripheral */
    spi_i2s_data_transmit(SPI5,byte);

    /* wait to receive a byte */
    while(RESET == spi_i2s_flag_get(SPI5,SPI_FLAG_RBNE));

    /* return the byte read from the SPI bus */
    return(spi_i2s_data_receive(SPI5));
}

/*!
    \brief      reset nandflash
    \param[in]  none
    \param[out] none
    \retval     flash identification
*/

void spi_nandflash_reset()
{
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
    /* send " RESET" command */
    spi_nandflash_send_byte(SPI_NAND_RESET);
    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
    while(spi_nandflash_get_status_flag(OIP)==SPI_NAND_BUSY);
}

/*!
    \brief      set nandflash register
    \param[in]  reg: the address of target register
    \param[in]  data: write parameters to target register
    \param[out] none
    \retval     flash identification
*/
void spi_nandflash_set_register(uint8_t reg,uint8_t data)
{
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
    /* send " RESET" command */
    spi_nandflash_send_byte(SPI_NAND_SET_FEATURE);
    spi_nandflash_send_byte(reg);
    spi_nandflash_send_byte(data);
    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
    while(spi_nandflash_get_status_flag(OIP)==SPI_NAND_BUSY);
}

/*!
    \brief      enable the write access to the flash
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_nandflash_write_enable(void)
{
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();

    /* send "write enable" instruction */
    spi_nandflash_send_byte(SPI_NAND_WREN);

    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
}

/*!
    \brief      poll the status of the write in progress(wip) flag in the flash's status register
    \param[in]  status_reg:the address nandflash status register
    \param[in]  *status: get the byte of status rgister
    \param[out] none
    \retval     none
*/
void spi_nandflash_get_feature(uint8_t status_reg, uint8_t *status)
{
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();

    /* send "get feature" command */
    spi_nandflash_send_byte(SPI_NAND_GET_FEATURE);
    /* send the address of status register */
    spi_nandflash_send_byte(status_reg);
    /* get nandflash status */
    *status = spi_nandflash_send_byte(DUMMY_BYTE);

    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
}
 
/*!
    \brief      get nandflash status register flag bit
    \param[in]  status_flag: the flag of nandflash status register 
    \param[out] none
    \retval     SPI_NAND_BUSY: nandflash is busy
    \retval     SPI_NAND_READY: nandflash is ready
*/
uint8_t spi_nandflash_get_status_flag(uint8_t status_flag)
{
    uint8_t status;
    /* read nandflash status register*/
    spi_nandflash_get_feature( STATUS, &status );
    if( (status & status_flag) == status_flag ){
        return SPI_NAND_BUSY;
    }
    else{
        return SPI_NAND_READY;
    }
}

/*!
    \brief      send the read page command
    \param[in]  page_No: the serial number of nandflash page
    \param[out] none
    \retval     none
*/
void spi_nandflash_page_read(uint32_t page_No)
{
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
    /* send "PAGE READ" command */
    spi_nandflash_send_byte(SPI_NAND_PAGE_READ);
    /* send the serial number of page */
    spi_nandflash_send_byte((page_No>>16)&0xFF);
    spi_nandflash_send_byte((page_No>>8)&0xFF);
    spi_nandflash_send_byte(page_No&0xFF);
    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
}

/*!
    \brief      send the read cache command
    \param[in]  buffer: a pointer to the array
    \param[in]  address_in_page: the address in nandflash page
    \param[in]  byte_cnt: the number of data
    \param[out] none
    \retval     none
*/
void spi_nandflash_read_cache(uint8_t *buffer,uint16_t address_in_page,uint32_t byte_cnt)
{
    uint32_t i=0;
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
#ifdef SPI_NANDFLASH
    /* send "PAGE READ" command */
    spi_nandflash_send_byte(SPI_NAND_READ_CACHE);
    /* send the address of page */
    spi_nandflash_send_byte((address_in_page>>8)&0xFF);
    spi_nandflash_send_byte(address_in_page&0xFF);
    spi_nandflash_send_byte(DUMMY_BYTE);
#endif

#ifdef QSPI_NANDFLASH
    /* send "PAGE READ" command */
    spi_nandflash_send_byte(SPI_NAND_READ_CACHE4);
    /* send the address of page */
    spi_nandflash_send_byte((address_in_page>>8)&0xFF);
    spi_nandflash_send_byte(address_in_page&0xFF);
    spi_nandflash_send_byte(DUMMY_BYTE);
    /* enable the qspi */
    qspi_enable(SPI5);
    qspi_read_enable(SPI5);
#endif
    for(i=0;i<byte_cnt;i++){
        *buffer++=spi_nandflash_send_byte(DUMMY_BYTE);
    }
    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
    spi_quad_disable(SPI5);
}

/*!
    \brief      send the program load command,write data to cache
    \param[in]  buffer: the data of array
    \param[in]  address_in_page: the address in nandflash page
    \param[in]  byte_cnt: the number of data
    \param[out] none
    \retval     none
*/
void spi_nandflash_program_load(uint8_t *buffer,uint16_t address_in_page,uint32_t byte_cnt)
{
    uint32_t i=0;
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
#ifdef SPI_NANDFLASH
    /* send "PAGE READ" command */
    spi_nandflash_send_byte(SPI_NAND_PAGE_LOAD);
    /* send the serial number of page */
    spi_nandflash_send_byte((address_in_page>>8)&0xFF);
    spi_nandflash_send_byte(address_in_page&0xFF);
#endif 
    
#ifdef QSPI_NANDFLASH
    /* send "PAGE READ" command */
    spi_nandflash_send_byte(SPI_NAND_PAGE_LOAD4);
    /* send the serial number of page */
    spi_nandflash_send_byte((address_in_page>>8)&0xFF);
    spi_nandflash_send_byte(address_in_page&0xFF);
    qspi_enable(SPI5);
    qspi_write_enable(SPI5);
#endif
    /* deselect the flash: chip select high */
    for(i=0;i<byte_cnt;i++){
        spi_nandflash_send_byte(*buffer++);
    }
    SPI_FLASH_CS_HIGH();
    spi_quad_disable(SPI5);
}

/*!
    \brief      send the program excute command
    \param[in]  page_No: the serial number of nandflash page
    \param[out] none
    \retval     none
*/
void spi_nandflash_program_execute(uint32_t page_No)
{
    /* enable the write access to the flash */
    spi_nandflash_write_enable();
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
    /* send "PAGE READ" command */
    spi_nandflash_send_byte(SPI_NAND_PROGRAM_EXEC);
    /* send the serial number of page */
    spi_nandflash_send_byte((page_No>>16)&0xFF);
    spi_nandflash_send_byte((page_No>>8)&0xFF);
    spi_nandflash_send_byte(page_No&0xFF);
    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
}

/*!
    \brief      send program load random data command
    \param[in]  buffer: the data of array
    \param[in]  address_in_page: the address in nandflash page
    \param[in]  byte_cnt: the number of data
    \param[out] none
    \retval     none
*/
void spi_nandflash_pl_random_data(uint8_t *buffer,uint16_t address_in_page,uint32_t byte_cnt)
{
    uint32_t i=0;
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
#ifdef SPI_NANDFLASH
    /* send "Program Load Random Data" command */
    spi_nandflash_send_byte(SPI_NAND_PAGE_RAND_LOAD);
    /* send the address in page */
    spi_nandflash_send_byte((address_in_page>>8)&0xFF);
    spi_nandflash_send_byte(address_in_page&0xFF);
#endif
#ifdef QSPI_NANDFLASH
    /* send "Program Load Random Data" command */
    spi_nandflash_send_byte(SPI_NAND_PAGE_RAND_LOAD4);
    /* send the address in page */
    spi_nandflash_send_byte((address_in_page>>8)&0xFF);
    spi_nandflash_send_byte(address_in_page&0xFF);
    qspi_enable(SPI5);
    qspi_write_enable(SPI5);
#endif
    for(i=0;i<byte_cnt;i++){
        spi_nandflash_send_byte(*buffer++);
    }
    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
    spi_quad_disable(SPI5);
}

/*!
    \brief      detect the nandflash block is or not bad 
    \param[in]  blockNo: the serial number of nandlash block
    \param[out] none
    \retval     none
*/
uint8_t spi_nandflash_badblock_detect(uint32_t blockNo)
{
    uint8_t ucFlag,BakFeature,result;
    result = 0;

    spi_nandflash_get_feature( FEATURE1, &BakFeature );
#ifdef DEBUG
    //printf("\BakFeature:%x\n",BakFeature);
#endif
    spi_nandflash_set_register(FEATURE1,BakFeature&0xEF);       //close ECC
    /* read the bad flag in spare area */
    spi_nandflash_read_spare(&ucFlag, blockNo * SPI_NAND_BLOCK_SIZE, BI_OFFSET, 1);
    if (ucFlag != 0xFF){
#ifdef DEBUG
        //printf("\rIsBadBlock:%x\n",blockNo);
#endif
        result = 1;
    }
    /* read the bad flag in spare area(reserve area) */
    spi_nandflash_read_spare(&ucFlag, blockNo * SPI_NAND_BLOCK_SIZE + 1, BI_OFFSET, 1);
    if (ucFlag != 0xFF){
#ifdef DEBUG
        //printf("\rIsBadBlock:%x\n",blockNo);
#endif
        result = 1;
    }
    spi_nandflash_set_register(FEATURE1,BakFeature);

    return result;
}

void test(void)
{
    uint16_t i,blockIdx,pageIdx;
    uint8_t tem_buffer_test[SPI_NAND_PAGE_TOTAL_SIZE];
	
    for(i = 0; i < 800; i++){
    //    nandflash_block_erase(i);
    }

    for(i = 0; i < 1024; i++)
    {
        tem_buffer_test[i] = i%256;
    }
    //spi_nandflash_write_data(tem_buffer,0x3FD*SPI_NAND_BLOCK_SIZE+0,0,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));    
    //nandflash_page_program( tem_buffer_test,0x3FD, 0, TRUE );
    for(blockIdx = 0; blockIdx < 800; blockIdx++){
        memset(tem_buffer_test,blockIdx,2);
        for(pageIdx = 0; pageIdx < 64; pageIdx++){
            nandflash_page_program( tem_buffer_test,blockIdx, pageIdx,0x400);
        }
		for(pageIdx = 0; pageIdx < 64; pageIdx++){
		    nandflash_page_read( tem_buffer_test,blockIdx, 0, 0, 0x400);
		}
	}
}

/*!
    \brief      initialize SPI5 and reset SPI nandflash 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_nandflash_init()
{
//    uint8_t status,hold=1,enable_erase=1,enable_test=1;
#ifdef DEMO
    uint16_t i,enable_erase=1,hold=0;
#else
    uint16_t i,enable_erase=0,hold=0;
#endif
    /* initialize SPI5 */
    spi5_init();
    /* reset NANDFLASH */
    spi_nandflash_reset();
    /* configure the PROTECTION register*/
    spi_nandflash_set_register(PROTECTION,0x00);
    /* configure the FEATURE1 register*/
    spi_nandflash_set_register(FEATURE1,0x11);//0x01��0x11
    /* build bock table: LUT = Look up table */
    //erase for debug
    while(hold);
    if(enable_erase==1){
        for(i=0;i<1024;i++){
            spi_nandflash_block_erase(i);
        }
    }
//    enable_erase = spi_nandflash_block_erase(700);
//    enable_erase = spi_nandflash_badblock_detect(700);
    
    //rebuild_DBTRBT_array();//load meta data

    //env_check();
//    if(enable_test==1){
//        test();
//    }
}

/*!
    \brief      read flash identification
    \param[in]  none
    \param[out] none
    \retval     flash identification
*/
uint32_t spi_nandflash_read_id(void)
{
    uint32_t temp = 0, temp0 = 0, temp1 = 0, temp2 = 0;

    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();

    /* send "READ_ID " instruction */
    spi_nandflash_send_byte(SPI_NAND_READID);

    /* read a byte from the flash */
    temp0 = spi_nandflash_send_byte(DUMMY_BYTE);

    /* read a byte from the flash */
    temp1 = spi_nandflash_send_byte(DUMMY_BYTE);

    /* read a byte from the flash */
    temp2 = spi_nandflash_send_byte(DUMMY_BYTE);

    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();

    temp = (temp0 << 16) | (temp1 << 8) | temp2;

    return temp;
}

/*!
    \brief      erase the nandflash blcok
    \param[in]  block_No:the serial number of erase block
    \param[out] none
    \retval     SPI_NAND_FAIL: erase the nandflash block fail
    \retval     SPI_NAND_SUCCESS: erase the nandflash block success
*/
#ifdef NFTL_TEST
uint8_t spi_nandflash_block_erase(uint32_t block_No)
{
    uint8_t result = SPI_NAND_SUCCESS;
    uint8_t status = 0;
#ifdef DEBUG
    printf("\rnandflash_page_erase:%x\n",block_No);    
#endif 
// #ifdef TEST
//     if((block_No%10)==5){
//         return SPI_NAND_FAIL;
//     }
// #endif
    block_No<<=6;        //block_No=block_No*64
    spi_nandflash_write_enable();
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
    /* send "ERASE BLOCK" command */
    spi_nandflash_send_byte(SPI_NAND_BLOCK_ERASE);
    /* send the address of memory */
    spi_nandflash_send_byte((block_No>>16)&0xFF);
    spi_nandflash_send_byte((block_No>>8)&0xFF);
    spi_nandflash_send_byte(block_No&0xFF);
    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
    /* check program result */        
    while(1){
        spi_nandflash_get_feature( STATUS, &status );
        if( (status & E_FAIL) == E_FAIL ){
            result = SPI_NAND_FAIL;
            break;
        }
        if( (status & OIP) == 0 ){
            break;
        }
    }
#ifdef DEBUG
    printf("\rerase result=%x\n",result);
#endif
    
    return result;
}
#else
uint8_t spi_nandflash_block_erase(uint32_t block_No)
{
    uint8_t result = SPI_NAND_SUCCESS;
    uint8_t status = 0;
#ifdef DEBUG
    printf("\rnandflash_page_erase:%x\n",block_No);    
#endif 
#ifdef TEST
    if((block_No%10)==5){
        return SPI_NAND_FAIL;
    }
#endif
    block_No<<=6;        //block_No=block_No*64
    spi_nandflash_write_enable();
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
    /* send "ERASE BLOCK" command */
    spi_nandflash_send_byte(SPI_NAND_BLOCK_ERASE);
    /* send the address of memory */
    spi_nandflash_send_byte((block_No>>16)&0xFF);
    spi_nandflash_send_byte((block_No>>8)&0xFF);
    spi_nandflash_send_byte(block_No&0xFF);
    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
    /* check program result */        
    while(1){
        spi_nandflash_get_feature( STATUS, &status );
        if( (status & E_FAIL) == E_FAIL ){
            result = SPI_NAND_FAIL;
            break;
        }
        if( (status & OIP) == 0 ){
            break;
        }
    }
#ifdef DEBUG
    printf("\rerase result=%x\n",result);
#endif
    return result;
}
#endif
/*!
    \brief      read the data from nandflash
    \param[in]  *buffer:the data of array
    \param[in]  page_No: the serial number of nandflash page
    \param[in]  address_in_page: the address in nandflash page
    \param[in]  byte_cnt:the number of data
    \param[out] none
    \retval     SPI_NAND_FAIL,SPI_NAND_SUCCESS 
*/
uint8_t spi_nandflash_read_data(uint8_t *buffer,uint32_t page_No,uint32_t address_in_page,uint32_t byte_cnt)
{
    uint8_t result = SPI_NAND_SUCCESS;
    uint8_t status = 0;
    uint8_t retrycnt = 0;
#ifdef DEBUG
    printf("\rspi_nandflash_read_data:%x %x %x %x\n",page_No/64,page_No%64,address_in_page,byte_cnt);
#endif
    /* the capacity of page must be equal or greater than the taotal of address_in_page and byte_cnt */
    if((address_in_page+byte_cnt)>SPI_NAND_PAGE_TOTAL_SIZE){
        return SPI_NAND_FAIL;
    }
ReadRetry:
    /* send the read page command */
    spi_nandflash_page_read(page_No);
    /* wait for NANDFLASH is ready */
    while(spi_nandflash_get_status_flag(OIP)==SPI_NAND_BUSY);
    /* read data from cache */
    spi_nandflash_read_cache(buffer, address_in_page, byte_cnt);
//    printfData((uint32_t)buffer,16);      //for debug
    spi_nandflash_get_feature( STATUS, &status );
#ifdef DEBUG
    printf("status:%x\n",status);
#endif
    if(( (status & ECCS0) == 0 )&&( (status & ECCS1) == ECCS1 )){    //UECC
        if(retrycnt < 3){
            retrycnt++;
#ifdef DEBUG
            printf("\rReadretry:%x\n",retrycnt); 
#endif         
            goto ReadRetry;
        }
        else{
#ifdef DEBUG
            printf("\rRead Fail\n");
#endif
            result = SPI_NAND_FAIL;
        }      
    }               
    return result;
}

/*!
    \brief      write the data to nandflash
    \param[in]  *buffer:the data of array
    \param[in]  page_No: the serial number of nandflash page
    \param[in]  address_in_page: the address of nandflash page
    \param[in]  byte_cnt:the number of data
    \param[out] none
    \retval     SPI_NAND_FAIL,SPI_NAND_SUCCESS 
*/
uint8_t spi_nandflash_write_data(uint8_t *buffer,uint32_t page_No,uint16_t address_in_page,uint32_t byte_cnt)
{
    uint8_t result = SPI_NAND_SUCCESS;
    uint8_t status = 0;
    
#ifdef DEBUG
    printf("\rspi_nandflash_write_data:%x %x %x %x\n",page_No/64,page_No%64,address_in_page,byte_cnt);
#endif

// #ifdef TEST
//     if((page_No/64%10)==0){
//         return SPI_NAND_FAIL;
//     }
// #endif
    /*send the program load command,write data to cache*/
    spi_nandflash_program_load(buffer, address_in_page, byte_cnt);
    /*send the program excute command*/
    spi_nandflash_program_execute(page_No);

    /* check program result */
    while(1){
        //status = spi_nandflash_get_status_flag();
        spi_nandflash_get_feature( STATUS, &status );   
        if( (status & P_FAIL) == P_FAIL ){
            result = SPI_NAND_FAIL;
            break;
        }
        if( (status & OIP) == 0 ){
            break;
        }
    }       
    
#ifdef WRITE_PAGE_VERIFY_EN
    spi_nandflash_read_data (tem_buffer,page_No, address_in_page, byte_cnt);
    if (memcmp(tem_buffer, buffer,  byte_cnt) != 0){
#ifdef DEBUG
        printf("\rdata compare fail\n");
#endif
        result = SPI_NAND_FAIL;          
    }
#endif
#ifdef DEBUG
    printf("\rwrite_result:%x\n",result);
#endif

    return result;
}

/*!
    \brief      nandflash internal data move 
    \param[in]  source_page: the source page address of data
    \param[in]  target_page: the target page address of data
    \param[out] none
    \retval     SPI_NAND_SUCCESS
*/
uint8_t spi_nandflash_copy_page(uint32_t source_page,uint32_t target_page)
{
    /* read the source page data to cache*/
    spi_nandflash_page_read(source_page);
    /* wait for nandflash is ready */
    while(spi_nandflash_get_status_flag(OIP)==SPI_NAND_BUSY);
    /*write the cache data to target page*/
    spi_nandflash_program_execute(target_page);
    /* wait for nandflash is ready */
    while(spi_nandflash_get_status_flag(OIP)==SPI_NAND_BUSY);
    return SPI_NAND_SUCCESS;
}

/*!
    \brief      update new data and nandflash internal data move
    \param[in]  buffer:update new data
    \param[in]  source_page: the source page address of data
    \param[in]  target_page: the target page address of data
    \param[in]  address_in_page: the address in page
    \param[in]  byte_cnt: the number of updated data
    \param[out] none
    \retval     SPI_NAND_SUCCESS
*/
uint8_t spi_nandflash_copy_page_update(uint8_t *buffer,uint32_t source_page,uint32_t target_page,uint16_t address_in_page,uint32_t byte_cnt)
{
    /* read the source page data to cache*/
    spi_nandflash_page_read(source_page);
    /* wait for nandflash is ready */
    while(spi_nandflash_get_status_flag(OIP)==SPI_NAND_BUSY);
    /* update new data to cache */
    spi_nandflash_pl_random_data(buffer, address_in_page, byte_cnt);
    /*write the cache data to target page*/
    spi_nandflash_program_execute(target_page);
    /* wait for nandflash is ready */
    while(spi_nandflash_get_status_flag(OIP)==SPI_NAND_BUSY);
    return SPI_NAND_SUCCESS;
}

/*!
    \brief      write the data to spare area
    \param[in]  *buffer: the data of array
    \param[in]  page_No: the serial number of nandflash page(0~2048)
    \param[in]  spare_address: the address in nandflash page spare area
    \param[in]  byte_cnt: the number of data
    \param[out] none
    \retval     SPI_NAND_FAIL,SPI_NAND_SUCCESS 
*/
uint8_t spi_nandflash_write_spare(uint8_t *buffer,uint32_t page_No,uint16_t spare_address,uint16_t byte_cnt)
{
#ifdef DEBUG
    printf("\r%s\n",__func__);
#endif
    if (byte_cnt > SPI_NAND_SPARE_AREA_SIZE){
        return SPI_NAND_FAIL;
    }
    return spi_nandflash_write_data(buffer, page_No, SPI_NAND_PAGE_SIZE + spare_address, byte_cnt);
}

/*!
    \brief      read the data from spare area
    \param[in]  *buffer: the data of array
    \param[in]  page_No: the serial number of nandflash page
    \param[in]  spare_address: the address in nandflash page spare area
    \param[in]  byte_cnt: the number of data
    \param[out] none
    \retval     SPI_NAND_FAIL,SPI_NAND_SUCCESS 
*/
uint8_t spi_nandflash_read_spare(uint8_t *buffer,uint32_t page_No,uint16_t spare_address,uint16_t byte_cnt)
{
    if (byte_cnt > SPI_NAND_SPARE_AREA_SIZE){
        return SPI_NAND_FAIL;
    }
    return spi_nandflash_read_data(buffer, page_No, SPI_NAND_PAGE_SIZE + spare_address, byte_cnt);
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
