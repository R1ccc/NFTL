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
#include "gd5f1gxx.h"
#include "esFtl_definitions.h"
#include "esFtl_disk.h"

#define MT29F1G01_DEVICE_ID 0x2c14
#define SE_TIMEOUT 10000
#define NFTL_TEST

/*
 * @brief initialize the GD5F chip
 *
 * @return 0 if it is successful
 */
 int esFtl_GD_NandFlashInit(uint32_t *pFlashID){
		/* configure SPI5 GPIO and parameter */
    spi_nandflash_init();

    /* read SPI NAND ID */
    *pFlashID = spi_nandflash_read_id();
	  
		/*ID Verification*/
//		if ((pFlashID != MT29F1G01_DEVICE_ID) && (pFlashID != W25N01GV_DEVICE_ID))
//        return -1;
		/*Set Status Register*/
		/*...*/
	 
	 return 0;
 }
 
/*
 * @brief reset a block to be ready to store data
 *
 * @param block
 * @return 0 if it is successful
 */
int esFtl_GD_NandFlashBlockErase(uint32_t block)
{
	if(SPI_NAND_FAIL == nandflash_block_erase(600)){
            printf("\n\rErase block failed!\n\r");
            while(1);
        }
	
	/*
    CharStream char_stream_send;
    uint8_t chars[4];
    uint8_t status_reg;

    if (block >= ESFTL_NANDNUMBLOCKS)
        return -1;

    block = block * ESFTL_NANDNUMPAGEBLOCK;

    if (IsFlashBusy())
        return -2;

    FlashWriteEnable();
    Set_Row_Stream(block, SPI_NAND_BLOCK_ERASE_INS, chars);

    char_stream_send.length = 4;
    char_stream_send.pChar = chars;

    Serialize_SPI(&char_stream_send, NULL, 1);

    WAIT_EXECUTION_COMPLETE(SE_TIMEOUT);
    FlashReadStatusRegister(&status_reg);
    if (status_reg & SPI_NAND_EF)
        return -3
				*/

    return 0;
}