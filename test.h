#include "stdio.h"
#include "stdbool.h"
#include "time.h"
/*pre define*/
#define SEQUENTIAL   0
#define RANDOM       1

uint8_t nandflash_page_program_fully(uint8_t *buffer, uint16_t buf_len, bool type);
uint8_t nandflash_page_program_partially(uint8_t *buffer, uint16_t buf_len);
uint8_t nandflash_erase(void);
uint8_t nandflash_block_erase_basic(uint16_t block_No);
uint8_t nandflash_page_program_basic(uint8_t *buffer, uint16_t block_No, uint8_t page_No, uint16_t buf_len);
uint8_t nandflash_page_program_certain_blocks(uint8_t *buffer1, uint8_t *buffer2, uint16_t buf_len);