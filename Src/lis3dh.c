/***************************************************************************************************
 *   Project:       X96/M96
 *   Author:        Lupandin Oleg
 ***************************************************************************************************
 *   Distribution:  Copyright © 2010 Ультра Стар
 ***************************************************************************************************
 *   MCU Family:    STM32f205
 *   Compiler:      Keil ARMCC
 ***************************************************************************************************
 *   File:          lis3dh.c
 *   Description:   Функции для обмена данными с LIS3DH
 ***************************************************************************************************
 *   History:       21.01.2011 - [Lupandin Oleg] - file created
 *                  09.01.2013 - [Lupandin Oleg] - modified
 *                  13.11.2015 - [Шипачев Илья]  - переделан под проект X96/M96
 * 
 *   TODO:          Поднять LIS3DH_BUS_LOCK и LIS3DH_CS_ACTIVE с их парами на уровни выше, чтобы
 *                  в 2 раза реже захватывать шину
 **************************************************************************************************/

/******************************************************************************
 *                             ЗАГОЛОВОЧНЫЕ ФАЙЛЫ
 ******************************************************************************/


#include  "lis3dh_conf.h"
#include  "lis3dh.h"


/******************************************************************************
 *                         ЛОКАЛЬНЫЕ МАКРООПРЕДЕЛЕНИЯ
 ******************************************************************************/

#define LIS3DH_ID_NUM                   0x33                                                        // ID акселлерометра
#define LIS3DH_REG_CNT                  0x3D                                                        // Количество регистров в акселерометре

// I2C

#define LIS3DH_I2C_ADRESS               0x32                                                        // адрес для обращения по i2c
#define LIS3DH_DATA_BYTE                0x28                                                        // адрес первого байта данных акселерометра

#define LIS3DH_I2C_WRITE_MASK           0x00                                                        // операция записи
#define LIS3DH_I2C_READ_MASK            0x01                                                        // операция чтения
#define LIS3DH_I2C_MULTIPLE_MASK        0x80                                                        // операция многобайтной записи/чтения

// SPI

#define LIS3DH_SPI_READ_MASK            0x80                                                        // чтение по SPI
#define LIS3DH_SPI_WRITE_MASK           0x00                                                        // запись по SPI
#define LIS3DH_SPI_MULTIPLE_MASK        0x40                                                        // запись/чтение нескольких байт

// registers

#define REG_LIS3DH_STATUS_REG_AUX      0x07
#define REG_LIS3DH_OUT_ADC1_L          0x08
#define REG_LIS3DH_OUT_ADC1_H          0x09
#define REG_LIS3DH_OUT_ADC2_L          0x0A
#define REG_LIS3DH_OUT_ADC2_H          0x0B
#define REG_LIS3DH_OUT_ADC3_L          0x0C
#define REG_LIS3DH_OUT_ADC3_H          0x0D
#define REG_LIS3DH_INT_COUNTER_REG     0x0E
#define REG_LIS3DH_WHO_AM_I            0x0F
#define REG_LIS3DH_SERIAL_MODE         0x17    // Номер регистра получен от ST-шников в личной переписке: установка старшего бита в нем отключает i2c
#define REG_LIS3DH_TEMP_CFG_REG        0x1F
#define REG_LIS3DH_CTRL_REG1           0x20
#define REG_LIS3DH_CTRL_REG2           0x21
#define REG_LIS3DH_CTRL_REG3           0x22
#define REG_LIS3DH_CTRL_REG4           0x23
#define REG_LIS3DH_CTRL_REG5           0x24
#define REG_LIS3DH_CTRL_REG6           0x25
#define REG_LIS3DH_REFERENCE           0x26
#define REG_LIS3DH_STATUS_REG2         0x27
#define REG_LIS3DH_OUT_X_L             0x28
#define REG_LIS3DH_OUT_X_H             0x29
#define REG_LIS3DH_OUT_Y_L             0x2A
#define REG_LIS3DH_OUT_Y_H             0x2B
#define REG_LIS3DH_OUT_Z_L             0x2C
#define REG_LIS3DH_OUT_Z_H             0x2D
#define REG_LIS3DH_FIFO_CTRL_REG       0x2E
#define REG_LIS3DH_FIFO_SRC_REG        0x2F
#define REG_LIS3DH_INT1_CFG            0x30
#define REG_LIS3DH_INT1_SOURCE         0x31
#define REG_LIS3DH_INT1_THS            0x32
#define REG_LIS3DH_INT1_DURATION       0x33
#define REG_LIS3DH_CLICK_CFG           0x38
#define REG_LIS3DH_CLICK_SRC           0x39
#define REG_LIS3DH_CLICK_THS           0x3A
#define REG_LIS3DH_TIME_LIMIT          0x3B
#define REG_LIS3DH_TIME_LATENCY        0x3C
#define REG_LIS3DH_TIME_WINDOW         0x3D

#define REG_LIS3DH_CTRL_REG3_CONF_WTM_OFF    0x00
#define REG_LIS3DH_FIFO_SRC_REG_EMPTY_MASK  (1<<5)  ///< 00100000 бит показывает, пустой ли FIFO буфер у акселерометра
#define REG_LIS3DH_FIFO_SRC_REG_OVRN_MASK   (1<<6)  ///< 01000000 бит показывает, произошло ли переполнение буфера (так же маркер наличия 32-ух значений в буфере)
#define REG_LIS3DH_FIFO_SRC_REG_DATA_MASK    0x1F   ///< 00011111 биты, показывающие количество оставшихся значений в fifo буфере акселерометра

/* Перенесено в lis3dh.h на время задачи XN-4615
#define LIS3DH_FIFO_MAX_CNT                         32                              ///< Максимальное количество паков (по значению на ось) значений акселерометров в FIFO lis3dh
//#warning "И.Ш.: На время тестирования для задачи XN-4615 буфер увеличен в 2 раза, чтобы не потерять значения"
//#define LIS3DH_FIFO_MAX_CNT                        64
#define LIS3DH_FIFO_MAX_CNT                       6                               ///< Объем памяти на один пак данных
//#define FIFO_MAX_SIZE                       (LIS3DH_FIFO_MAX_CNT * LIS3DH_FIFO_MAX_CNT)  ///< Максимальное количество байт данных в FIFO буфере LIS3DH (6 - байты данных на один тик, 32 - размер FIFO буфера)
*/

#define SERIAL_MODE_I2C_DISABLE        0x80                                  ///< См. комментарий к определению REG_LIS3DH_SERIAL_MODE

/******************************************************************************
 *                            ЛОКАЛЬНЫЕ ТИПЫ ДАННЫХ
 ******************************************************************************/

/* Временно перенесено в lis3dh.h
typedef struct
{
    uint8_t  arr[LIS3DH_FIFO_MAX_CNT][LIS3DH_FIFO_MAX_CNT];
    uint8_t  pack_idx;                          ///< индекс записи следующих значений, принимает значения от 0 до LIS3DH_FIFO_MAX_CNT
    uint8_t  unread_cnt;                        ///< количество не вычитанных значений акселерометра из буфера
} lis3dh_buf_t;
*/

/******************************************************************************
 *                              ГЛОБАЛЬНЫЕ ДАННЫЕ
 ******************************************************************************/
/******************************************************************************
 *                              ЛОКАЛЬНЫЕ ДАННЫЕ
 ******************************************************************************/

static lis3dh_buf_t  s_lis3dh_buf = {0};      ///< локальный буфер, хранящий fifo акселерометра lis3dh
static bool s_byte_damaged = false;                ///< флаг повреждения бит в байте данных, которые не используются для расчетов, но должны быть нулями

/******************************************************************************
 *                         ПРОТОТИПЫ ЛОКАЛЬНЫХ ФУНКЦИЙ
 ******************************************************************************/

static void lis3dh_byte_write(uint8_t addr, uint8_t data);
static void lis3dh_read(uint8_t addr, uint8_t *buf, uint8_t bytes);
static void lis3dh_data_read(uint8_t num);
//static void lis3dh_data_convert(uint8_t const * const axis_data, uint8_t *data);
static void damaged_byte_set(void);

extern SPI_HandleTypeDef hspi1;
/******************************************************************************
 *                              ЛОКАЛЬНЫЕ ФУНКЦИИ
 ******************************************************************************/

//запись байта
static void lis3dh_byte_write(uint8_t addr, uint8_t data)
{
   // LIS3DH_BUS_LOCK();
    ACC_ENABLE;
	
    //bsp_lis3dh_exchange(addr | LIS3DH_SPI_WRITE_MASK);
		addr = addr | LIS3DH_SPI_WRITE_MASK;
		HAL_SPI_Transmit(&hspi1, &addr, sizeof(addr), 0x1000);	

  //  bsp_lis3dh_exchange(data);
		HAL_SPI_Transmit(&hspi1, &data, sizeof(data), 0x1000);
	
//		bsp_lis3dh_bus_free_wait();

    ACC_DISABLE;
  //  LIS3DH_BUS_UNLOCK();
}

//чтение байта
static void lis3dh_read(uint8_t addr, uint8_t *buf, uint8_t bytes)
{
//    uint8_t cnt;

//    LIS3DH_BUS_LOCK();  
    ACC_ENABLE;

    //bsp_lis3dh_exchange(addr | LIS3DH_SPI_READ_MASK | LIS3DH_SPI_MULTIPLE_MASK);
		addr = addr | LIS3DH_SPI_READ_MASK | LIS3DH_SPI_MULTIPLE_MASK;
		HAL_SPI_Transmit(&hspi1, &addr, sizeof(addr), 0x1000);
	
    //for (cnt = 0; cnt < bytes; cnt++)
    //{
    //  *(buf + cnt) =  bsp_lis3dh_exchange(0);
    //}
    //bsp_lis3dh_bus_free_wait();

		HAL_SPI_TransmitReceive(&hspi1, buf, buf, bytes, 0x1000);
	
    ACC_DISABLE;
//    LIS3DH_BUS_UNLOCK();
}

//функция для чтения данных в акселерометре, чтобы вычитывать их за один захват SPI-шины.
static void lis3dh_data_read(uint8_t num)
{
  //  *buf = *s_lis3dh_buf.buf;
    uint8_t cnt, pack;
		uint8_t addr;
	
//    LIS3DH_BUS_LOCK();  
    ACC_ENABLE;

    //bsp_lis3dh_exchange(LIS3DH_DATA_BYTE | LIS3DH_SPI_READ_MASK | LIS3DH_SPI_MULTIPLE_MASK);
		addr = LIS3DH_DATA_BYTE | LIS3DH_SPI_READ_MASK | LIS3DH_SPI_MULTIPLE_MASK;
		HAL_SPI_Transmit(&hspi1, &addr, sizeof(addr), 0x1000);
   
	for (pack = 0; pack < num; ++pack)
    {
        for (cnt = 0; cnt < LIS3DH_DATA_PACK_SIZE; ++cnt)
        {
          // // *(buf + (LIS3DH_FIFO_MAX_CNT * pack) + cnt) = bsp_lis3dh_exchange(0);
          //s_lis3dh_buf.arr[s_lis3dh_buf.pack_idx][cnt] = bsp_lis3dh_exchange(0);
					s_lis3dh_buf.arr[s_lis3dh_buf.pack_idx][cnt] = 0;
					HAL_SPI_TransmitReceive(&hspi1, &s_lis3dh_buf.arr[s_lis3dh_buf.pack_idx][cnt], &s_lis3dh_buf.arr[s_lis3dh_buf.pack_idx][cnt], 1, 0x1000);
        }
        if (++(s_lis3dh_buf.pack_idx) >= LIS3DH_FIFO_MAX_CNT) { s_lis3dh_buf.pack_idx = 0; }
    }
    //bsp_lis3dh_bus_free_wait();

    ACC_DISABLE;
//    LIS3DH_BUS_UNLOCK();
}

static void damaged_byte_set(void)
{
    s_byte_damaged = true;
}

//конвертирует машинное представление axis_data в честное пресдтавление uint16_t в data
//TODO: переделать на структуру lis3dh_data_t
/*
static void lis3dh_data_convert(uint8_t const * const axis_data, uint8_t *data)
{
    uint8_t cnt;
    
    for (cnt = 0; cnt < 3; cnt++)
    {
      *((uint16_t*)data + cnt) = ((int)(*(axis_data + cnt*2)& 0xC0) >> 6)+(((int)*(axis_data + cnt*2 + 1)) << 2);
      if (*((uint16_t*)data + cnt)&0x0200)
        *((uint16_t*)data + cnt) += 0xFC00;
    }
}
*/

/******************************************************************************
 *                             ГЛОБАЛЬНЫЕ ФУНКЦИИ
 ******************************************************************************/

//Функция, вычитывающая значения всех регистров в lis3dh
void lis3dh_all_registers_read(uint8_t *buf)
{
    uint8_t i;
    for (i = 0; i < LIS3DH_REG_CNT; ++i)
    {
        //TODO: переделать на чтение нескольких байт сразу, а не по одному
        lis3dh_read(i, buf + i, 1);
    }
}

//Проверить id микросхемы
bool lis3dh_id_check(void) 
{
  uint8_t tmp;
  
  lis3dh_read(REG_LIS3DH_WHO_AM_I, &tmp, 1);
  if (tmp == LIS3DH_ID_NUM) { return true;  }
  else                      { return false; }
}

/** -------------------------------------------------------------------------------
  *
  * \brief          Отключение i2c. 
  *  
  * \note           Эта функция должна быть вызвана до Init'а самой микросхемы и всех 
  *                 остальных lis'ов, которые висят на том же SPI 
  *  
  * \details        В рамках задачи XN-4170 было обнаружено, что пин CS в микросхеме LIS
  *                 является не совсем CS. Когда он в "0" - она CS, когда в "1" - он переключает
  *                 микросхему в режим i2c. Тогда весь обмен данными по spi с другими устройставми
  *                 будет восприниматься как шум на i2c линии.
  *  
  *                 Проблема в том, что момент инициализации SPI генерит ложный i2c-старт (линия MOSI (=SDA)
  *                 падает в "0" при стоящей в "1" линии SCK (=SCL)). При этом запись в регистры 0x30, 0x31,
  *                 0x32 и 0x33 другого Lis'а, висящего на той же шине, декодируются как i2c-адрес. И тогда
  *                 lis начинает сам управлять линией SDA (=MOSI)
  *                 
  * -------------------------------------------------------------------------------
  */
void lis3dh_preinit(void)
{
    uint8_t  temp;

    //bsp_lis3dh_init();
    lis3dh_read(REG_LIS3DH_SERIAL_MODE, &temp, 1);
    temp |= SERIAL_MODE_I2C_DISABLE;
    lis3dh_byte_write(REG_LIS3DH_SERIAL_MODE, temp);
}


/** -------------------------------------------------------------------------------
  *
  * \brief          Инициализация lis3dh начальной конфигурацией 
  *  
  * -------------------------------------------------------------------------------
  */
bool lis3dh_init(void)
{
    lis3dh_byte_write(REG_LIS3DH_CTRL_REG1, LIS3DH_CTRL_REG1_CONF);
    lis3dh_byte_write(REG_LIS3DH_CTRL_REG2, LIS3DH_CTRL_REG2_CONF);
    lis3dh_byte_write(REG_LIS3DH_CTRL_REG3, LIS3DH_CTRL_REG3_CONF);
    lis3dh_byte_write(REG_LIS3DH_CTRL_REG4, LIS3DH_CTRL_REG4_CONF);
    lis3dh_byte_write(REG_LIS3DH_CTRL_REG5, LIS3DH_CTRL_REG5_CONF);
    lis3dh_byte_write(REG_LIS3DH_CTRL_REG6, LIS3DH_CTRL_REG6_CONF);
    lis3dh_byte_write(REG_LIS3DH_FIFO_CTRL_REG, LIS3DH_FIFO_CTRL_REG_CONF);
    return lis3dh_id_check();
}

/** -------------------------------------------------------------------------------
  *
  * \brief          Вычитывание FIFO из lis3dh
  *  
  * \details        В два приема захвата шины SPI сначала читаем статус FIFO буфера, 
  *                 потом, в зависимости от значения в нем хранящегося, вычитываем
  *                 нужное количество пачек данных, сохраняя их в буфер
  *                 s_lis3dh_buf.buf
  *  
  * \todo           Сделать один захват шины на чтение статуса FIFO и чтения данных. 
  * \todo           Избавиться от промежуточного буфера s_lis3dh_buf.buf, опустив 
  *                 вниз преобразование данных и осуществляя запись сразу в общий
  *                 буфер в acc_sens.c, что сэкономит 200 байт ОЗУ                  
  *                 
  * -------------------------------------------------------------------------------
  */
uint8_t lis3dh_fifo_save(void)
{
    uint8_t  data_cnt = 0; 
    uint8_t  tmp;

    lis3dh_read(REG_LIS3DH_FIFO_SRC_REG, &tmp, 1);
    if(!(tmp & REG_LIS3DH_FIFO_SRC_REG_EMPTY_MASK))     ///< фифо не пустое
    {
        data_cnt = tmp & REG_LIS3DH_FIFO_SRC_REG_DATA_MASK;   // Считываем биты, показывающие кол-во данных в FIFO буфере
        if (data_cnt == 0)       // FIFO буфер пустой, хотя приходим за данными раз в N милисекунд(см. биты WTM в LIS3DH_FIFO_CTRL_REG_CONF)
        {
            s_lis3dh_buf.unread_cnt = 0;
            return 0;
        }            
        else
        {
            if (tmp & REG_LIS3DH_FIFO_SRC_REG_OVRN_MASK)        //Установленный этот флаг означает, что в буфере 32 значения, и часть уже, возможно, потеряна
            {    
                data_cnt = LIS3DH_FIFO_MAX_CNT;
            }
            //printf("Будет прочитано байт из FIFO буфера LIS3DH:            %d\n", data_cnt);
            lis3dh_data_read(data_cnt);
            //s_lis3dh_buf.cnt = data_cnt;
            s_lis3dh_buf.unread_cnt += data_cnt;
            return data_cnt;
        }
    }
    else
    {
        s_lis3dh_buf.unread_cnt = 0;
        return 0;
    }
}

/** -------------------------------------------------------------------------------
  *
  * \brief          Функция взятия одной тройки значений из буфера и их преобразование
  * 
  * \note           захват SPI шины НЕ производится
  *  
  * \details        Ранее заполненный сырыми байтами буфер s_lis3dh_buf читается этой
  *                 функцией и выполняются все необходимые преобразования данных
  *  
  * \todo           Отдавать данные lis3dh наверх в сыром виде, а разбирать уже 
  *                 из task_sensor.c 
  *                 
  * -------------------------------------------------------------------------------
  */
bool lis3dh_data_get(int16_t *x, int16_t *y, int16_t *z) 
{
    int16_t  data[3];
    int8_t   read_pack_idx;
    uint8_t  bh, bl;         ///< старший и младший байты значения акселерометра
    uint8_t  cnt;          
    uint16_t val;
    
    if (s_lis3dh_buf.unread_cnt > 0)
    {
        //учитываем, что pack_idx указывает на место для следующей записи
        read_pack_idx = s_lis3dh_buf.pack_idx - s_lis3dh_buf.unread_cnt;    

        while (read_pack_idx < 0)
        {
            read_pack_idx += LIS3DH_FIFO_MAX_CNT;
        }

        for (cnt = 0; cnt < 3; ++cnt)
        {
            bh  = s_lis3dh_buf.arr[read_pack_idx][cnt * 2];
            bl  = s_lis3dh_buf.arr[read_pack_idx][cnt * 2 + 1];
            val = ((uint16_t)(bh & 0xC0) >> 6) + ((uint16_t)bl << 2);
            if (val & 0x0200)   //проверка на отрицательность
            {
                val |= 0xFC00;
            }
            data[cnt] = (int16_t)val;

            //проверка пустоты нужных полей, которые не используются в итоговых рассчетов
            if ((bh & 0x3F) != 0)
            {
                damaged_byte_set();
            }
        }

        *x = data[0];         
        *y = data[1];         
        *z = data[2];

        --(s_lis3dh_buf.unread_cnt);  
        return true;               
    }
    else
    {
        return false;
    }
}

//TODO: Реализовать софтварную перезагрузку c перезаписью всех значений в нужные регистры
void lis3dh_reboot(void)
{

}

/** -------------------------------------------------------------------------------
  *
  * \brief          Снятие прерывания путем чтения источника прерывания
  * 
  * -------------------------------------------------------------------------------
  */
uint8_t lis3dh_int_src_get(void)
{
    uint8_t  tmp;

    lis3dh_read(REG_LIS3DH_INT1_SOURCE,  &tmp, 1);
    return tmp;
}

uint8_t lis3dh_get_reg1(void)
{
    uint8_t  tmp;

    lis3dh_read(REG_LIS3DH_CTRL_REG1,  &tmp, 1);
    return tmp;
}

uint8_t lis3dh_get_reg2(void)
{
    uint8_t  tmp;

    lis3dh_read(REG_LIS3DH_CTRL_REG2,  &tmp, 1);
    return tmp;
}

uint8_t lis3dh_get_reg3(void)
{
    uint8_t  tmp;

    lis3dh_read(REG_LIS3DH_CTRL_REG3,  &tmp, 1);
    return tmp;
}

uint8_t lis3dh_get_reg4(void)
{
    uint8_t  tmp;

    lis3dh_read(REG_LIS3DH_CTRL_REG4,  &tmp, 1);
    return tmp;
}

uint8_t lis3dh_get_reg5(void)
{
    uint8_t  tmp;

    lis3dh_read(REG_LIS3DH_CTRL_REG5,  &tmp, 1);
    return tmp;
}

uint8_t lis3dh_get_reg6(void)
{
    uint8_t  tmp;

    lis3dh_read(REG_LIS3DH_CTRL_REG6,  &tmp, 1);
    return tmp;
}

uint8_t lis3dh_get_regfifo(void)
{
    uint8_t  tmp;

    lis3dh_read(REG_LIS3DH_FIFO_CTRL_REG,  &tmp, 1);
    return tmp;
}
/** -------------------------------------------------------------------------------
  *
  * \brief          Установка режима прерываний по watermark'у
  * 
  * -------------------------------------------------------------------------------
  */
void lis3dh_int_wtm_on(void)
{
    lis3dh_byte_write(REG_LIS3DH_CTRL_REG3, LIS3DH_CTRL_REG3_CONF);
}

/** -------------------------------------------------------------------------------
  *
  * \brief          Отключение прерываний с lis3dh
  * 
  * -------------------------------------------------------------------------------
  */
void lis3dh_int_wtm_off(void)
{
    lis3dh_byte_write(REG_LIS3DH_CTRL_REG3, REG_LIS3DH_CTRL_REG3_CONF_WTM_OFF);      //отключаем прерывания с lis3dh
}

//отладочная передача ссылки на буфер
lis3dh_buf_t *lis3dh_buf_ptr_get(void)
{
    return &s_lis3dh_buf;
}

bool lis3dh_byte_damaged_check(void)
{
    return s_byte_damaged;
}

void lis3dh_byte_damaged_clear(void)
{
    s_byte_damaged = false;
}


/******************************************************************************
 *                                 КОНЕЦ ФАЙЛА
 ******************************************************************************/

