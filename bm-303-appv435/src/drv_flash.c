/****************************************************************************
*  Copyright (C) 2019 RoboMaster.
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of 
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program. If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/
#include <stdint.h>
#include <string.h>
#include "drv_flash.h"


/**
 * @brief  write bytes to flash  
 *         start addr  = PARAM_SAVED_START_ADDRESS
 * @retval flash write status
 */
uint32_t page_err;
uint16_t final_halfword, temp, i;
#if defined (STM32F1)
uint8_t bsp_flash_write_2b(uint32_t start_addr, uint8_t *pbuff, uint32_t len)
{
    FLASH_EraseInitTypeDef EraseInitStruct;

    /*erase flash before program */
    // uint32_t start_addr = PARAM_SAVED_START_ADDRESS;
    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.NbPages = 1;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.PageAddress = start_addr;
    while (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &page_err));

    

    for (i=0; i<len; i += 2) {
        temp = pbuff[i + 1] << 8 | pbuff[i];
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, start_addr + i, temp);
    }
    if (len%2) {
        //should add a byte to halfword.
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, start_addr + i, pbuff[len - 1]);
    }

    /*write data end*/
    HAL_FLASH_Lock();
    return 0;
}

void flash_erase(uint32_t start_addr, uint32_t len) 
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    // uint32_t page_err;
    HAL_FLASH_Unlock();
    uint32_t page_cnt = len / FLASH_PAGE_SIZE + (len % FLASH_PAGE_SIZE ? 1 : 0);
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.NbPages = page_cnt;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.PageAddress = start_addr;
    while (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &page_err));
    HAL_FLASH_Lock();
}

//make sure flash erased before write!!!.
void flash_write(uint32_t start_addr, uint8_t* pbuff, uint32_t len)
{
    HAL_FLASH_Unlock();

    for (i=0; i<len; i += 2) {
        temp = pbuff[i + 1] << 8 | pbuff[i];
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, start_addr + i, temp);
    }
    if (len%2) {
        //should add a byte to halfword.
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, start_addr + len - 1, pbuff[len - 1]);
    }
    HAL_FLASH_Lock();
}
#endif

#if defined (STM32G4)

uint64_t data64;
uint8_t bsp_flash_write_8b(uint32_t start_addr, uint8_t *pbuff, uint32_t len)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    
    /*erase flash before program */
    // uint32_t start_addr = PARAM_SAVED_START_ADDRESS;
    HAL_FLASH_Unlock();

    int i = 0;
    int16_t cnt = len;
    while (cnt > 0) {
        //should add a byte to double word.
        //data64 = *(uint64_t*)&pbuff[i];
        memcpy(&data64, &pbuff[i], sizeof(uint64_t));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_addr + i, data64);
        i += 8;
        cnt -= 8;
    }

    /*write data end*/
    HAL_FLASH_Lock();
    return 0;
}

//default DBANK=1, so 64 double words once.
//fast program 就是一次写32个双字（8b， total=256byte）
//当写入最后一个row， 使用FLASH_TYPEPROGRAM_FAST_AND_LAST来关闭FAST-PG功能A
#define FLASH_ROW_SIZE          32
/* Table used for fast programming */
static uint64_t data64_buffer_aligned[FLASH_ROW_SIZE] = {
    0x0000000000000000, 0x1111111111111111, 0x2222222222222222, 0x3333333333333333,
    0x4444444444444444, 0x5555555555555555, 0x6666666666666666, 0x7777777777777777,
    0x8888888888888888, 0x9999999999999999, 0xAAAAAAAAAAAAAAAA, 0xBBBBBBBBBBBBBBBB,
    0xCCCCCCCCCCCCCCCC, 0xDDDDDDDDDDDDDDDD, 0xEEEEEEEEEEEEEEEE, 0xFFFFFFFFFFFFFFFF,
    0x0011001100110011, 0x2233223322332233, 0x4455445544554455, 0x6677667766776677,
    0x8899889988998899, 0xAABBAABBAABBAABB, 0xCCDDCCDDCCDDCCDD, 0xEEFFEEFFEEFFEEFF,
    0x2200220022002200, 0x3311331133113311, 0x6644664466446644, 0x7755775577557755,
    0xAA88AA88AA88AA88, 0xBB99BB99BB99BB99, 0xEECCEECCEECCEECC, 0xFFDDFFDDFFDDFFDD
};

//64 dwords.
int flash_write_row_stm32g4(uint32_t start_addr, uint64_t src_addr, uint16_t* write_bytes)
{
    uint64_t bytes_write;
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef ret;
    *write_bytes = FLASH_ROW_SIZE * sizeof(uint64_t);
    ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, start_addr, src_addr);
    //  + i * FLASH_ROW_SIZE * sizeof(uint64_t)
    /*write data end*/
    HAL_FLASH_Lock();
    return ret;
}

int stm32_internal_flash_erase(uint32_t start_addr, uint32_t len) 
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    // uint32_t page_err;
    HAL_FLASH_Unlock();
    uint32_t page_cnt = len / FLASH_PAGE_SIZE + (len % FLASH_PAGE_SIZE ? 1 : 0);
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.NbPages = page_cnt;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Page = (start_addr - 0x8000000) / FLASH_PAGE_SIZE;  //page index.
    while (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &page_err));
    HAL_FLASH_Lock();
}

//make sure flash erased before write!!!.
int stm32_internal_flash_write(uint32_t start_addr, uint8_t* pbuff, uint32_t len)
{
    return bsp_flash_write_8b(start_addr, pbuff, len);
}

static uint16_t wr;
uint8_t readback[FLASH_ROW_SIZE * 8];
uint8_t test[FLASH_PAGE_SIZE];
uint8_t test_rd[FLASH_PAGE_SIZE];

#include "log.h"

#define FLASH_ADDR_LUT_START    \
    (0x08000000 + FLASH_PAGE_SIZE*63)  //last page.2k

uint8_t g4_flash_test(uint32_t addr)
{
    flash_erase(FLASH_ADDR_LUT_START, FLASH_PAGE_SIZE);

    uint16_t wr;
    for (int i=0; i<sizeof(test); ++i) {
        test[i] = i;
    }
    // memcpy((uint8_t*)data64_buffer_aligned, test, len);

    flash_write(FLASH_ADDR_LUT_START, test, sizeof(test));

    memcpy(test_rd, (uint8_t*)FLASH_ADDR_LUT_START, FLASH_PAGE_SIZE);
    
    if (0 != memcmp(test, test_rd, FLASH_PAGE_SIZE) ) {
        LOGE("flash internal rw test fail.");
        return 1;
    }
    LOGI("internal flash rw test pass.");
    return 0;

    for (int p=0; p<FLASH_PAGE_SIZE / sizeof(data64_buffer_aligned); ++p)
    {
        bsp_flash_write_8b(
            FLASH_ADDR_LUT_START + p*sizeof(data64_buffer_aligned),
            (uint8_t*)data64_buffer_aligned, 
            FLASH_ROW_SIZE*8);

        memcpy(readback, (uint8_t*)FLASH_ADDR_LUT_START, 256);
        if (0 != memcmp(readback, data64_buffer_aligned, 256)) {
            return 1;   //fail
        }
    }

    // flash_write_row_stm32g4(addr, data64_buffer_aligned, &wr);
    bsp_flash_write_8b(addr, (uint8_t*)data64_buffer_aligned, FLASH_ROW_SIZE*8);
    memcpy(readback, (uint8_t*)FLASH_ADDR_LUT_START, FLASH_ROW_SIZE * 8);
    if (0 == memcmp(readback, data64_buffer_aligned, FLASH_ROW_SIZE*8)) {
        return 1;
    }
    return 0;
}

#endif

#if defined (STM32F3)

int flash_erase(uint32_t start_addr, uint32_t len) 
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    // uint32_t page_err;
    HAL_FLASH_Unlock();
    uint32_t page_cnt = len / FLASH_PAGE_SIZE + (len % FLASH_PAGE_SIZE ? 1 : 0);
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.NbPages = page_cnt;
    EraseInitStruct.PageAddress = start_addr;  //page index.
    while (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &page_err));
    HAL_FLASH_Lock();
}

int flash_write(uint32_t start_addr, uint8_t* pbuff, uint32_t len)
{
    HAL_FLASH_Unlock();
    //Program the user Flash area word by word
    const int program_size = 4;
    uint32_t temp32;
    for (i=0; i<len; i += program_size) {
        // temp = pbuff[i + 1] << 8 | pbuff[i]; //halfword
        temp32 = *(uint32_t*)(pbuff + i);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_addr + i, temp32);
    }
    if (len % program_size) {
        // uint32_t last_word = temp = *(uint32_t*)(pbuff + len - 1);
        //program tail.
        #warning "will lost tail if not 32bit align."
        // HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_addr + len - 1, pbuff[len - 1]);
    }
    HAL_FLASH_Lock();
}

#endif

#if defined (STM32F4)

#include "log.h"

int flash_erase(uint32_t start_addr, uint32_t len) 
{
    // FLASH_EraseInitTypeDef EraseInitStruct;
    // // uint32_t page_err;
    // HAL_FLASH_Unlock();
    // uint32_t page_cnt = len / FLASH_PAGE_SIZE + (len % FLASH_PAGE_SIZE ? 1 : 0);
    // EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    // EraseInitStruct.NbSectors = page_cnt;
    // EraseInitStruct.PageAddress = start_addr;  //page index.
    // while (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &page_err));
    // HAL_FLASH_Lock();
}

int flash_write(uint32_t start_addr, uint8_t* pbuff, uint32_t len)
{
    // HAL_FLASH_Unlock();
    // //Program the user Flash area word by word
    // const int program_size = 4;
    // uint32_t temp32;
    // for (i=0; i<len; i += program_size) {
    //     // temp = pbuff[i + 1] << 8 | pbuff[i]; //halfword
    //     temp32 = *(uint32_t*)(pbuff + i);
    //     HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_addr + i, temp32);
    // }
    // if (len % program_size) {
    //     // uint32_t last_word = temp = *(uint32_t*)(pbuff + len - 1);
    //     //program tail.
    //     #warning "will lost tail if not 32bit align."
    //     // HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_addr + len - 1, pbuff[len - 1]);
    // }
    // HAL_FLASH_Lock();
}




// #define FLASH_SECTOR_0     0U  /*!< Sector Number 0   */
// #define FLASH_SECTOR_1     1U  /*!< Sector Number 1   */
// #define FLASH_SECTOR_2     2U  /*!< Sector Number 2   */
// #define FLASH_SECTOR_3     3U  /*!< Sector Number 3   */
// #define FLASH_SECTOR_4     4U  /*!< Sector Number 4   */
// #define FLASH_SECTOR_5     5U  /*!< Sector Number 5   */
// #define FLASH_SECTOR_6     6U  /*!< Sector Number 6   */
// #define FLASH_SECTOR_7     7U  /*!< Sector Number 7   */
// #define FLASH_SECTOR_8     8U  /*!< Sector Number 8   */
// #define FLASH_SECTOR_9     9U  /*!< Sector Number 9   */
// #define FLASH_SECTOR_10    10U /*!< Sector Number 10  */
// #define FLASH_SECTOR_11    11U /*!< Sector Number 11  */

typedef struct __flash_sector_layout{
  uint32_t addr;
  uint32_t size;
}flash_sector_layout_t;

const flash_sector_layout_t sector_layouts[] = {
  {0x08000000UL, 16*0x400,},
  {0x08004000UL, 16*0x400,},
  {0x08008000UL, 16*0x400,},
  {0x0800C000UL, 16*0x400,},

  {0x08010000UL, 64*0x400,},

  {0x08020000UL, 128*0x400,},
  {0x08040000UL, 128*0x400,},
  {0x08060000UL, 128*0x400,},
  {0x08080000UL, 128*0x400,},
  {0x080A0000UL, 128*0x400,},
  {0x080C0000UL, 128*0x400,},
  {0x080E0000UL, 128*0x400,},
};

//this function can only find correct addr
uint32_t flash_sector_idx_find(uint32_t addr)
{
  #define ARRAY_CNT(a)  (sizeof(a)/sizeof(a[0]))
  uint32_t cnts = ARRAY_CNT(sector_layouts);
  for (int i=0; i<cnts; ++i) {
    if (addr == sector_layouts[i].addr ) 
    {
      return i;
    }
    else if (addr > sector_layouts[i].addr 
        && i < cnts 
        && addr < sector_layouts[i+1].addr) 
    {
      return i;
    }
  }
  return 0xff;  //fail
}

uint32_t flash_sector_nb_get(uint32_t addr, uint32_t require_size)
{
  uint32_t req_sector_cnt = 0;
  uint32_t current_total_size = 0;
  uint32_t i = flash_sector_idx_find(addr);
  do {
    current_total_size += sector_layouts[i].size;
    i++;
    req_sector_cnt++;
  } while (current_total_size < require_size);
  return req_sector_cnt;
}

void stm32f4_flash_erase(uint32_t addr, uint32_t size)
{
  HAL_FLASH_Unlock();
  static FLASH_EraseInitTypeDef fe;
  static uint32_t err;
  HAL_StatusTypeDef ret;
  fe.Banks = FLASH_BANK_1;
  fe.Sector = flash_sector_idx_find(addr);
  fe.NbSectors = flash_sector_nb_get(addr, size);
  fe.TypeErase = FLASH_TYPEERASE_SECTORS;
  fe.VoltageRange = FLASH_VOLTAGE_RANGE_3;  //2.7-3.6
  ret = HAL_FLASHEx_Erase(&fe, &err);
  if (ret != HAL_OK) {
    LOGI("flash erase error. %d", err);
  }
  HAL_FLASH_Lock();
}

void stm32f4_flash_write(uint32_t addr, uint8_t *buf, uint32_t size, void (*stage_disp_func)(uint32_t i))
{
  HAL_FLASH_Unlock();
  HAL_StatusTypeDef ret;
  for (int i=0; i<size; ++i) {
    // *stage = i * 100 / size;
    if (stage_disp_func) {
      stage_disp_func(i);
    }
    ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr+i, buf[i]);
    if (ret != HAL_OK) {
      LOGI("flash write error. %d", i);
    }
  }
  HAL_FLASH_Lock();
}



#endif

#if defined (STM32H7)

//sector erase & program by 256bits = 32byte!!
uint64_t data64;
uint64_t FlashWord[4] = { 0x0102030405060708,
                          0x1112131415161718,
                          0x2122232425262728,    
                          0x3132333435363738
                        };
uint8_t bsp_flash_write_32byte(uint32_t start_addr, uint8_t *pbuff, uint32_t len)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    
    /*erase flash before program */
    HAL_FLASH_Unlock();

    int i = 0;
    int16_t cnt = len;
    while (cnt > 0) {
        // memcpy(&data64, &pbuff[i], sizeof(uint64_t));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, 
                        start_addr + i, 
                        &pbuff[i]);
        i += 32;
        cnt -= 32;
    }

    /*write data end*/
    HAL_FLASH_Lock();
    return 0;
}

int stm32_internal_flash_erase(uint32_t start_addr, uint32_t len) 
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    // uint32_t page_err;
    HAL_FLASH_Unlock();
    uint32_t erase_nb = len / FLASH_SECTOR_SIZE + (len % FLASH_SECTOR_SIZE ? 1 : 0);
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.NbSectors = erase_nb;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Sector = (start_addr - 0x8000000) / FLASH_SECTOR_SIZE;  //page index.
    while (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &page_err));
    HAL_FLASH_Lock();
}

//make sure flash erased before write!!!.
int stm32_internal_flash_write(uint32_t start_addr, uint8_t* pbuff, uint32_t len)
{
    return bsp_flash_write_32byte(start_addr, pbuff, len);
}


#define TEST_FLASH_SIZE (4096)
uint8_t readback[TEST_FLASH_SIZE];
uint8_t test_wr[TEST_FLASH_SIZE];
uint8_t test_rd[TEST_FLASH_SIZE];

#include "log.h"

#define FLASH_ADDR_LUT_START    \
    (FLASH_BASE + FLASH_SECTOR_SIZE*7)  //last sector.80E0000

uint8_t h7_flash_test()
{
    uint32_t addr = FLASH_ADDR_LUT_START;
    stm32_internal_flash_erase(addr, FLASH_SECTOR_SIZE);

    uint16_t wr;
    for (int i=0; i<sizeof(test_wr); ++i) {
        test_wr[i] = i;
    }
    stm32_internal_flash_write(addr, 
                                test_wr, 
                                sizeof(test_wr));

    memcpy(test_rd, (uint8_t*)addr, TEST_FLASH_SIZE);
    
    if (0 != memcmp(test_wr, test_rd, TEST_FLASH_SIZE) ) {
        LOGE("flash internal rw test fail.");
        return 1;
    }
    LOGI("internal flash rw test pass.");
    return 0;
}
#endif

store_param_t store_param;
void write_store_param(store_param_t *sp)
{
    // sp->store_valid_magic = FLASH_STORE_VALID_MAGIC;
    // stm32_internal_flash_erase(FLASH_STORE_PARAM_ADDR, sizeof(store_param_t));
    // stm32_internal_flash_write(FLASH_STORE_PARAM_ADDR, sp, sizeof(*sp));
}

void read_store_param(store_param_t *sp)
{
    memcpy(sp, FLASH_STORE_PARAM_ADDR, sizeof(*sp));
}


