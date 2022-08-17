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

#ifndef __DRV_FLASH_H__
#define __DRV_FLASH_H__


// #include "stm32h7xx_hal.h"
// #include "stm32f4xx_hal_flash.h"

#define STM32F103XB_FLASH_SIZE    (0x10000)  //(0x400*64)
#define PARAM_SAVED_START_ADDRESS (0x08000000 + STM32F103XB_FLASH_SIZE - FLASH_PAGE_SIZE) //last sector of flash 2MB flash
#define FLASH_USER_START_ADDR ADDR_FLASH_SECTOR_2      /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR ADDR_FLASH_SECTOR_23       /* End @ of user Flash area */

// #define FLASH_STORE_PARAM_ADDR  0x0800D000UL  //xiaomi fan use next next addr
#define FLASH_STORE_PARAM_ADDR  0x080E0000UL  //h7
#define FLASH_STORE_VALID_MAGIC (0x66666666UL)

#define BIT_POS(x)                  (1u << x)
#define PARAM_STATUS_CURRENT_OFFSET BIT_POS(0)
#define PARAM_STATUS_THETA_OFFSET   BIT_POS(1)
#define PARAM_STATUS_HALL_OFFSET    BIT_POS(2)
#define PARAM_STATUS_THETA_TAB      BIT_POS(3)

#define PARAM_STATUS_ALL_VALID      (0x0F)
#define RECORDER_FLASH_CNT        (4096)
// #define PARAM_STATUS_CURRENT_IA     BIT_POS(4)

typedef struct _store_param 
{
  /* data */
  uint16_t current_raw_offset[3];
  uint16_t hall_raw_offset[2];
  uint16_t elec_angle_offset;
  uint32_t param_status;      //1 = valid.

  uint16_t record_theta_tab[RECORDER_FLASH_CNT];

  uint32_t store_valid_magic; //0x6666
}store_param_t;

extern store_param_t store_param;

uint8_t bsp_flash_write(uint8_t *pbuff, uint32_t len);

uint8_t bsp_flash_write_8b(uint32_t start_addr, uint8_t *pbuff, uint32_t len);
int flash_write_row_stm32g4(uint32_t start_addr, uint64_t src_addr, uint16_t* write_bytes);

int flash_erase(uint32_t start_addr, uint32_t len);
//make sure flash erased before write!!!.
int flash_write(uint32_t start_addr, uint8_t* buf, uint32_t len);
uint8_t g4_flash_test(uint32_t addr);

uint8_t h7_flash_test();

void stm32f4_flash_erase(uint32_t addr, uint32_t size);
void stm32f4_flash_write(uint32_t addr, uint8_t *buf, uint32_t size, void (*stage_disp_func)(uint32_t i));

void write_store_param(store_param_t *sp);
void read_store_param(store_param_t *sp);

static inline int is_store_param_valid(store_param_t *sp) {
    return FLASH_STORE_VALID_MAGIC == sp->store_valid_magic; 
}


#endif // __DRV_FLASH_H__
