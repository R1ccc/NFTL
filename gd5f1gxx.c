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

uint16_t ABT[TOTAL_BLOCK];//PHYSICAL BLOCK ERASE COUNT
uint64_t PST[RBT_SIZE];
uint16_t L2P[RBT_SIZE];//LOGICAL BLOCK ADDRESS TO PHYSICAL BLOCK ADDRESS
uint8_t  ABT_CNT = 0;//COUNTER RECORD HOW MANY TIMES ABT MODIFIED 
uint8_t  L2P_CNT = 0;//COUNTER RECORD HOW MANY TIMES L2P MODIFIED 
uint8_t  BST_CNT = 0;//COUNTER RECORD HOW MANY TIMES BST MODIFIED 
uint8_t  DBT_BLOCK_1;
/* detect the nandflash block is or not bad */
bool     BST[TOTAL_BLOCK];//mark if the physical block is empty

bool FLAG_ABT_UPDATE = 0;
bool FLAG_L2P_UPDATE = 0;
bool FLAG_BST_UPDATE = 0;

static uint8_t spi_nandflash_badblock_detect(uint32_t blockNo);

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
static void compress_BST(const bool *BST, uint8_t *BST_compressed, uint16_t total_blocks);
static void decompress_BST(const uint8_t *BST_compressed, bool *BST, uint16_t total_blocks);


static uint16_t get_mapped_physical_block(uint16_t block_No);
static void set_ABT(uint16_t block_No, uint16_t value);
static uint16_t get_ABT(uint16_t block_No);
static void set_BST(uint16_t block_No, uint16_t value);
static bool get_BST(uint16_t block_No);

static uint16_t get_empty_block(bool *array, size_t size, uint16_t ori_block);
static uint16_t get_min_erase_block(void);
static uint16_t calculate_average(uint16_t *array, uint16_t size);
static uint16_t select_proper_block(uint16_t physical_block_No);
uint16_t select_unmapped_block(void);
void env_check(void);
//static void printfData(uint32_t a32SAddr, uint32_t byteCnt);

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
    qspi_disable(SPI5);
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
    qspi_disable(SPI5);
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
    qspi_disable(SPI5);
}

/*!
    \brief      detect the nandflash block is or not bad 
    \param[in]  blockNo: the serial number of nandlash block
    \param[out] none
    \retval     none
*/
static uint8_t spi_nandflash_badblock_detect(uint32_t blockNo)
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
    
    rebuild_DBTRBT_array();//load meta data

    env_check();
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
    uint32_t ori_block_no = block_No;
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
    uint32_t LBlockNo = page_No / SPI_NAND_BLOCK_SIZE;
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
    \brief      write the page data to nandflash
    \param[in]  buffer: the data of array
    \param[in]  block_No: the address in nandflash block(0~799)
    \param[in]  page_No: the address in nandflash page(0~63) in block_No
    \param[in]  buf_len: the length of array(1~2048)
    \param[out] none
    \retval     SPI_NAND_FAIL,SPI_NAND_SUCCESS 
*/
#ifdef NFTL_TEST
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
        replace_block = get_replace_block_from_ram(physical_block);
        #ifdef LOG
            printf("*********Bad Block**********\nBad block detected: %x Original PB: %X Updated PB: %X\n",block_No, physical_block, replace_block);
		#endif
        set_mapped_physical_block(logical_block, replace_block);//update L2P
    }else{
        replace_block = physical_block;
    }
    
    //check erase cnt
    //based on wear leveling select the proper physical block
	average = calculate_average(ABT, 1024);
    if((ABT[replace_block] - average) > WL_THRESHOLD){
    
        proper_block = select_proper_block(replace_block);
        //swap mapping table L2P
        L2P[block_No] = proper_block;
		#ifdef LOG
        printf("***************Wear Leveling************* \nphyblock %d ers_cnt %d average %d rpl_blk %d", replace_block, ABT[replace_block], average, proper_block);		
        #endif
        for (i = 0; i < sizeof(L2P); i++)
        {
            if (L2P[i] == proper_block && i != block_No)//find the logical block that mapped to properblock
            {
                L2P[i] = replace_block;//swap
                break;
            }           
        }
        L2P_CNT++;//COUNTER ++ 
        //CHECK IF NEED TO UPDATE L2P
    }
    replace_block = get_mapped_physical_block(logical_block);//ACTUAL PROGRAM PHYSICAL BLOCK
    memset(tem_buffer, 0x5A, SPI_NAND_PAGE_SIZE);
    memcpy(tem_buffer, buffer, buf_len);
    result = spi_nandflash_write_data(tem_buffer,replace_block*SPI_NAND_BLOCK_SIZE+page_No,0,SPI_NAND_PAGE_SIZE);
    if(result == SPI_NAND_FAIL){     
        update_DBTRBT_array(replace_block);
        re_mapping_RBT(block_No,replace_block);
        update_DBTRBT_to_nand(TYPE_DBT);
        update_DBTRBT_to_nand(TYPE_RBT); 
        //copy the old block content to new alloced block
        for(page_idx = 0;page_idx < page_No;page_idx++){
            new_replace_block = get_replace_block_from_ram(replace_block);
            move_page_data(new_replace_block,replace_block,page_idx);
        }
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

    if(BST_CNT > 10){
        update_L2PBST_to_nand();
        BST_CNT = 0;
    }

    #ifdef TEST_LOAD_UPDATE_ABTL2P
        //READ ABT L2P BST TO CHECK
        update_ABT_to_nand();
        //spi_nandflash_read_data(tem_buffer,g16ABT_Block[g8ABT_CRT_Block_Idx]*SPI_NAND_BLOCK_SIZE+g8ABT_CRT_Page,0,SPI_NAND_PAGE_SIZE);
        //CHECK: update ABT success        
        //spi_nandflash_read_data(tem_buffer,g16L2PBST_Block[g8DL2PBST_CRT_Block_Idx]*SPI_NAND_BLOCK_SIZE+g8DL2PBST_CRT_Page,0,SPI_NAND_PAGE_SIZE);
        //MODIFY CLEAR ABT TO 0, CHECK LOAD ABT FROM NAND 
        memset(ABT, 0, sizeof(ABT[0])*1024);
        load_ABT_from_nand();
        
        update_L2PBST_to_nand();
        memset(L2P, 0, sizeof(L2P[0])*800);
        memset(BST, 0, sizeof(BST[0])*1024);
        memset(tem_buffer, 0, sizeof(tem_buffer[0])*2112);
        load_L2PBST_from_nand();
    #endif
    return result;
}
#else
uint8_t nandflash_page_program(uint8_t *buffer, uint16_t block_No, uint8_t page_No, uint16_t buf_len)
{
    uint8_t result;
    uint8_t page_idx;
    uint16_t replace_block,new_replace_block;     
    if((block_No > USER_AREA_END)||(page_No>=SPI_NAND_BLOCK_SIZE)||(buf_len>SPI_NAND_PAGE_SIZE)){
        return SPI_NAND_FAIL;
    }
    if(check_whether_in_DBT_array(block_No)==true){
        replace_block = get_replace_block_from_ram(block_No);
    }else{
        replace_block = block_No;
    }
    memset(tem_buffer, 0x5A, SPI_NAND_PAGE_SIZE);
    memcpy(tem_buffer, buffer, buf_len);
    result = spi_nandflash_write_data(tem_buffer,replace_block*SPI_NAND_BLOCK_SIZE+page_No,0,SPI_NAND_PAGE_SIZE);
    if(result == SPI_NAND_FAIL){     
        update_DBTRBT_array(replace_block);
        re_mapping_RBT(block_No,replace_block);
        update_DBTRBT_to_nand(TYPE_DBT);
        update_DBTRBT_to_nand(TYPE_RBT); 
        //copy the old block content to new alloced block
        for(page_idx = 0;page_idx < page_No;page_idx++){
            new_replace_block = get_replace_block_from_ram(replace_block);
            move_page_data(new_replace_block,replace_block,page_idx);
        }
    }
    return result;
}
#endif
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
    if((block_No > USER_AREA_END)||(page_No>=SPI_NAND_BLOCK_SIZE)||(buf_len>SPI_NAND_PAGE_SIZE)){
        return SPI_NAND_FAIL;
    }
    
    if(check_whether_in_DBT_array(p_block_no)==true){//check pba is bb or not
        replace_block = get_replace_block_from_ram(p_block_no);
        L2P[block_No] = replace_block;//update L2P TABLE
    }else{
        replace_block = p_block_no;
    }
    
    if(spi_nandflash_read_data(buffer,replace_block*SPI_NAND_BLOCK_SIZE+page_No,address_in_page,buf_len) == SPI_NAND_SUCCESS){
//        memcpy(buffer, tem_buffer, buf_len);
        return SPI_NAND_SUCCESS;
    }else{ 
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
    uint16_t physical_block;

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

        replace_block = get_replace_block_from_ram(physical_block);
		#ifdef LOG
            printf("*********Bad Block**********\nBad block detected: %x Original PB: %X Updated PB: %X\n",block_No, physical_block, replace_block);
		#endif
        L2P[physical_block] = replace_block;//update the L2P table
    }else{
        replace_block = physical_block;
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
        update_ABT_to_nand();
        ABT_CNT = 0;
    }

    if(BST_CNT > 10){
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
static void load_L2PBST_from_nand()
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
    uint8_t   ABT_Erase_Flag,RBT_Erase_Flag;
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
static uint8_t update_L2PBST_to_nand()
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
        result = spi_nandflash_write_data(tem_buffer,BlockNo*SPI_NAND_BLOCK_SIZE+PageNo,0,(SPI_NAND_PAGE_SIZE+SPI_NAND_SPARE_AREA_SIZE));
        if(result==SPI_NAND_FAIL){
            return result;
        }
        else{
            BST[BlockNo] = NOT_EMPTY;
        }
        if(L2PBST_Erase_Flag == true){
            spi_nandflash_block_erase(g16ABT_Block[1-g8ABT_CRT_Block_Idx]);
            BST[g16ABT_Block[1-g8ABT_CRT_Block_Idx]] = EMPTY;
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
static void add_bad_Block_to_DBTRBT_ram(uint16_t BlockNo)
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
    DBT[1+g16BadBlockNum] = BlockNo;
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
static uint8_t update_DBTRBT_array(uint16_t BlockNo)
{
    uint8_t   result = true;
#ifdef DEBUG
    //printf("\r%s:%x\n",__func__,BlockNo);
#endif
    result = check_whether_in_DBT_array(BlockNo);
    if ( result==false ) {
        add_bad_Block_to_DBTRBT_ram(BlockNo);
    }
    return result;
}


/*!
    \brief      allocate the blocks to save bad block mapping table
    \param[in]  none
    \param[out] none
    \retval     none
*/
#ifdef NFTL_TEST
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
    memset(BST,0,TOTAL_BLOCK);//Initialize block status as unwritten
    //init L2P TABLE L2P[1] = 1(BLOCK1)....
	#ifdef TEST
        ABT[0x3B5] = 10;
        ABT[0x02]  = 0x1234;
    #endif
    for (i = 0; i < RBT_SIZE; ++i) {
        L2P[i] = (uint16_t)i;
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
#else
static void alloc_DBTRBT_block_addr(void)
{
    uint16_t BlockNo = Table_AREA_END;
    uint8_t  Idx;
    g16BadBlockNum = 0;
    memset(DBT,DBT_SIZE,0);
    memset(RBT,RBT_SIZE,0);
    
#ifdef DEBUG
    printf("\r%s\n",__func__);
#endif
    for(Idx = 0;Idx < DBT_BLOCK_NUM;Idx++){     //alloc DBT address 
        for(; BlockNo > Table_AREA_START; BlockNo--){   //950~1023
            if(spi_nandflash_badblock_detect(BlockNo)==0){               
                g16DBT_Block[Idx]=BlockNo;
#ifdef DEBUG
                printf("\rDBT_Block[%x]=%x\n",Idx,g16DBT_Block[Idx]);
#endif
                BlockNo--;
                break;
            }
            else{               //bad block
                g16BadBlockNum ++;          
                DBT[1+g16BadBlockNum] = BlockNo;
            }
        }
    }
    g8DBT_CRT_Block_Idx = 0;
    g8DBT_CRT_Page = 0xFF;

    for(Idx = 0;Idx < RBT_BLOCK_NUM;Idx++){     //alloc RBT address
        for(; BlockNo > Table_AREA_START; BlockNo--){
            if(spi_nandflash_badblock_detect(BlockNo)==0){
                g16RBT_Block[Idx]=BlockNo;
#ifdef DEBUG
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
}
#endif

/*!
    \brief      init bad block table by scan nand flash
    \param[in]  none
    \param[out] none
    \retval     none
*/
#ifdef NFTL_TEST
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
            add_bad_Block_to_DBTRBT_ram(BlockNo);
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
#else
static void init_build_DBTRBT(void)
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
            add_bad_Block_to_DBTRBT_ram(BlockNo);
            continue;
        }
        else{
            RBT[BlockNo] = BlockNo;
        }
    }
    DBT[0] = g16BadBlockNum;
    DBT[1] = g16LastValidBlock;
}
#endif

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
        //���û�ҵ�������ȫ��ɨ�裬�ؽ�DBTRBT
        #ifdef NFTL_TEST
            alloc_DBTRBTABTPST_block_addr();
            init_build_DBTRBTABTPST();
        #else
            alloc_DBTRBT_block_addr();
            init_build_DBTRBT();
        #endif
        update_ABT_to_nand();
        update_L2PBST_to_nand();
        update_DBTRBT_to_nand(TYPE_DBT);
        update_DBTRBT_to_nand(TYPE_RBT);
        update_DBTRBT_to_nand(TYPE_ENV);
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
        //return result;
    }else{
		update_DBTRBT_to_nand(TYPE_ENV);
        //return result;
    }   
		return 0;
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
    physical_replace_block = get_min_erase_block();
    
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
static uint16_t get_min_erase_block(void) {
    uint16_t i,j;
	bool skip = false;
    uint16_t block, return_block = 0xFFFF;
    uint16_t erase_cnt_min;
    uint16_t *skip_blocks[5][2] = {g16DBT_Block, g16RBT_Block, g16ABT_Block, g16L2PBST_Block, g16ENV_Block};
	for (block = 0; block < g16LastValidBlock; block++)
    {   
        if(check_whether_in_DBT_array(block) == true){
            continue;
        }
				skip = false;
         for (i = 0; i < 5; i++) {
            for ( j = 0; j < sizeof(g16DBT_Block); j++)
            {
                if (block == *(skip_blocks[i][j])) {
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
		//find minimum erase block		
        if(block == 0){
            erase_cnt_min = ABT[block];
            return_block = block;
						if(ABT[block] == 0){
							break;
						}
        }
        else if(ABT[block] == 0 && BST[block] == EMPTY){
            return_block = block;
            break;
        }
        else{
            if(ABT[block] < erase_cnt_min){
                return_block = block;
                erase_cnt_min = ABT[block];
            }
            else{
                continue;
            }
        }
    }
    return return_block;
}

/*!
    \brief      get the physical block from L2P
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/
static uint16_t get_mapped_physical_block(uint16_t block_No) {
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
void compress_BST(const bool *BST, uint8_t *BST_compressed, uint16_t total_blocks) {
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

void decompress_BST(const uint8_t *BST_compressed, bool *BST, uint16_t total_blocks) {
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

void set_ABT(uint16_t block_No, uint16_t value){
    ABT[block_No] = value;
}

/*!
    \brief      for test case only. Get ABT to certain value
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/

uint16_t get_ABT(uint16_t block_No){
    return ABT[block_No];
}

/*!
    \brief      for test case only. Set BST to certain value
    \param[in]  array: BST
    \param[out] none
    \retval     none
*/

void set_BST(uint16_t block_No, uint16_t value){
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

void env_check(void)
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

uint8_t test_env(uint16_t BlockNo)
{
    uint16_t i,blockIdx,pageIdx,enable_erase=1;
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
