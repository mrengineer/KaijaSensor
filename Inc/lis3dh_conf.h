/******************************************************************************
 * \file lis3dh_conf.h
 * \author Илья Шипачев <shipachev@starline.ru>
 *
 * \brief Конфигурационный файл библиотеки драйвера LIS3DH
 *
 * \copyright ООО "НПО "СтарЛайн", 16.11.2015. Все права защищены.
 *
 ******************************************************************************/
  
#ifndef __LIS3DH_CONF_H__ // Для предотвращения повторного включения
#define __LIS3DH_CONF_H__

/******************************************************************************
 *                             ЗАГОЛОВОЧНЫЕ ФАЙЛЫ
 ******************************************************************************/

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_spi.h"

/******************************************************************************
 *                         ГЛОБАЛЬНЫЕ МАКРООПРЕДЕЛЕНИЯ
 ******************************************************************************/

// Пре-инициализация драйвера LIS3DH
#define LIS3DH_PREINIT bsp_lis3dh_preinit

// Вспомогательные макросы для обмена данными
//Acc
#define ACC_ENABLE			HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET); 	HAL_Delay(1);
#define ACC_DISABLE			HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET); 	HAL_Delay(1);

// Захват шины данных
#define LIS3DH_BUS_LOCK   //bsp_lis3dh_spi_init

// Освобождение шины данных
#define LIS3DH_BUS_UNLOCK //bsp_lis3dh_spi_deinit

// Прерывание акселлерометра используется в приложении
//#define LIS3DH_BOTH_INTERRUPT_OPERATE
//#define LIS3DH_ONLY_ONE_INTERRUPT_OPERATE

// Задержка в десятках микросекунд
//#define lis3dh_delay(delay_10x_us) COMMON_SLEEP((delay_10x_us) / 100)

///----------------------------CTRL_REGS_CONF--------------------------------
#define LIS3DH_CTRL_REG1_CONF        0x57       ///< DR_100Hz | normal_mode | X_Y_Z_enable
#define LIS3DH_CTRL_REG2_CONF        0x00       ///< все по-умолчанию, внутренние фильтры пропускаются
#define LIS3DH_CTRL_REG3_CONF        0x04       ///< FIFO_WTM_int_active (прерывание по границе заполнения FIFO)
#define LIS3DH_CTRL_REG4_CONF        0x80       ///< block_data_update | LSBL_at_lower | scale_2g | high_res_off | self_test_off | spi_4wire
#define LIS3DH_CTRL_REG5_CONF        0x48       ///< FIFO_enable | Latch interupt
#define LIS3DH_CTRL_REG6_CONF        0x00       ///< в даташите нет нормального описания, стоит default
#define LIS3DH_FIFO_CTRL_REG_CONF    0x8A       ///< stream_mode | trig_on_int1 | threshold_wtm_10valuess

/*
///----------------------------INTx_REGS_CONF--------------------------------
#define LIS3DH_INT1_CFG_CONF       
#define LIS3DH_INT1_THS_CONF          INT1_THS_VAL
#define LIS3DH_INT1_DURATION_CONF     INT1_DUR_VAL

//INT2 не используется!
#define LIS3DH_INT2_CFG_CONF          
#define LIS3DH_INT2_THS_CONF          
#define LIS3DH_INT2_DURATION_CONF     
*/

#endif //#ifndef __LIS3DH_CONF_H__ Предотвращает повторное включение
 
/******************************************************************************
 *                                 КОНЕЦ ФАЙЛА
 ******************************************************************************/

