/***************************************************************************************************
 *   Project:       
 *   Author:        Lupandin Oleg
 ***************************************************************************************************
 *   Distribution:  Copyright © 2010 Ультра Стар
 ***************************************************************************************************
 *   MCU Family:    STM8L152C6xx
 *   Compiler:      ST Visual Develop
 ***************************************************************************************************
 *   File:          lis3dh.h
 *   Description:   функции для обмена данными с LIS3DH
 ***************************************************************************************************
 *   History:       21.01.2011 - [Lupandin Oleg] - file created
 **************************************************************************************************/

/******************************************************************************
 *                             ЗАГОЛОВОЧНЫЕ ФАЙЛЫ
 ******************************************************************************/

/******************************************************************************
 *                         ГЛОБАЛЬНЫЕ МАКРООПРЕДЕЛЕНИЯ
 ******************************************************************************/

#ifndef __LIS_3DH__
#define __LIS_3DH__


    #include <stddef.h>
    #include <stdint.h>
    #include <stdbool.h>

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
}lis3dh_data_t;

typedef enum
{
        LIS3DH_FIFO_MODE_BYPASS = 0x00,
        LIS3DH_FIFO_MODE_FIFO = 0x40,
        LIS3DH_FIFO_MODE_STREAM = 0x80,
        LIS3DH_FIFO_MODE_TRIGGER = 0xC0,
        LIS3DH_FIFO_MODE_STREAM_WTM_25 = 0x99  //режим STREAM с генерацией прерывания INT1 по уровню 25 значений в буфере
}LIS3DH_FIFO_Mode_Typedef;

typedef enum
{
        LIS3DH_MODE_NORMAL = 0x00,
        LIS3DH_MODE_LP = 0x08
}LIS3DH_Mode_Typedef;

typedef enum
{
        LIS3DH_SCALE_2G = 0x00,
        LIS3DH_SCALE_4G = 0x10,
        LIS3DH_SCALE_8G = 0x20,
        LIS3DH_SCALE_16G = 0x30
}LIS3DH_Scale_Typedef;

typedef enum
{
        LIS3DH_FREQ_0Hz_PD = 0x00,
        LIS3DH_FREQ_1Hz = 0x10,
        LIS3DH_FREQ_10Hz = 0x20,
        LIS3DH_FREQ_25Hz = 0x30,
        LIS3DH_FREQ_50Hz = 0x40,
        LIS3DH_FREQ_100Hz = 0x50,
        LIS3DH_FREQ_200Hz = 0x60,
        LIS3DH_FREQ_400Hz = 0x70,
        LIS3DH_FREQ_1600Hz_LP = 0x80,
        LIS3DH_FREQ_1250Hz_N_5kHz_LP = 0x90,
}LIS3DH_Freq_Typedef;

typedef enum
{
        LIS3DH_FIFO_Int_WTM = 0x02,
}LIS3DH_FIFO_Int_Typedef;

//TODO: После получения результатов по задаче XN-4615 убрать в lis3dh.c, сняв префиксы
#define LIS3DH_FIFO_MAX_CNT                         64                              ///< Максимальное количество пачек значений акселерометров в FIFO lis3dh
//#warning "И.Ш.: На время тестирования для задачи XN-4615 буфер увеличен в 2 раза, чтобы не потерять значения"
//#define FIFO_MAX_CNT                        64
#define LIS3DH_DATA_PACK_SIZE                       6                               ///< Размер одной пачки значений акселерометра, включающее показания 3-ех осей

typedef struct
{
    uint8_t  arr[LIS3DH_FIFO_MAX_CNT][LIS3DH_DATA_PACK_SIZE];
    uint8_t  pack_idx;                          ///< индекс записи следующих значений, принимает значения от 0 до FIFO_MAX_CNT
    uint8_t  unread_cnt;                        ///< количество не вычитанных значений акселерометра из буфера
} lis3dh_buf_t;

/******************************************************************************
 *                           ГЛОБАЛЬНЫЕ ТИПЫ ДАННЫХ
 ******************************************************************************/
/******************************************************************************
 *                              ГЛОБАЛЬНЫЕ ДАННЫЕ
 ******************************************************************************/
/******************************************************************************
 *                        ПРОТОТИПЫ ГЛОБАЛЬНЫХ ФУНКЦИЙ
 ******************************************************************************/
void    lis3dh_all_registers_read(uint8_t *buf);
bool    lis3dh_id_check(void);
void    lis3dh_preinit(void);
bool    lis3dh_init(void);

uint8_t lis3dh_fifo_save(void);
bool	lis3dh_data_get(int16_t *x, int16_t *y, int16_t *z);
//void    lis3dh_fifo_config(LIS3DH_FIFO_Mode_Typedef mode);
void    lis3dh_reboot(void);

uint8_t lis3dh_int_src_get(void);
void    lis3dh_int_wtm_on(void);
void    lis3dh_int_wtm_off(void);

//И.Ш. Убрать эти 3 ф-ии после разрешения проблем из задачи XN-4615
lis3dh_buf_t *lis3dh_buf_ptr_get(void);
bool lis3dh_byte_damaged_check(void);
void lis3dh_byte_damaged_clear(void);

uint8_t lis3dh_get_reg1(void);
uint8_t lis3dh_get_reg2(void);
uint8_t lis3dh_get_reg3(void);
uint8_t lis3dh_get_reg4(void);
uint8_t lis3dh_get_reg5(void);
uint8_t lis3dh_get_reg6(void);

//uint16_t LIS3DH_Data_Convert(uint8_t * axis_data);

//bool LIS3DH_Data_New_Get(lis3dh_data_t * data);
//uint8_t LIS3DH_Data_FIFO_Get(lis3dh_data_t * data);
//uint8_t LIS3DH_Data_FIFO_Get_old(lis3dh_data_t * data);

//void LIS3DH_FIFO_Config(bool cond, LIS3DH_FIFO_Mode_Typedef mode, uint8_t watermark, uint8_t int_mask);

//void LIS3DH_FIFO_Int_Cond(LIS3DH_FIFO_Int_Typedef mode);
//void lis3dh_int_config(LIS3DH_FIFO_Int_Typedef mode);

#endif //#define __LIS_3DH__

/******************************************************************************
 *                                 КОНЕЦ ФАЙЛА
 ******************************************************************************/

