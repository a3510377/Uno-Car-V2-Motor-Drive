#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#include "main.h"
#include "stdbool.h"

#define BUFFER_LENGTH 32

#ifndef I2C_TX_RX_BUFFER_SIZE
  #define I2C_TX_RX_BUFFER_SIZE 32
#elif I2C_TX_RX_BUFFER_SIZE >= 256
  #error I2C buffer size cannot exceed 255
#endif

#define I2C_SLAVE_MODE_TRANSMIT 0
#define I2C_SLAVE_MODE_RECEIVE 1
#define I2C_SLAVE_MODE_LISTEN 2

typedef struct i2c_s i2c_t;

struct i2c_s {
  I2C_HandleTypeDef _handle;

  void (*_i2c_onSlaveReceive)(i2c_t *);
  void (*_i2c_onSlaveTransmit)(i2c_t *);

  void (*onReceive)(int);
  void (*onRequest)(void);

  bool _transmitting;

  uint8_t *_tx_buffer;
  uint16_t _tx_data_size;

  uint8_t *_rx_buffer;
  uint16_t _rx_buffer_index;
  uint16_t _rx_buffer_length;
  uint16_t _rx_buffer_allocated;

  // Number of accumulated bytes received in Slave mode
  volatile int _slave_rx_nb_data;
  volatile uint8_t _i2c_tx_rx_buffer[I2C_TX_RX_BUFFER_SIZE];
  volatile uint8_t _i2c_tx_rx_buffer_size;
  volatile uint8_t _slave_mode;
};

void i2c_begin(void);
int i2c_available(void);
int i2c_read(void);
uint8_t i2c_readAddress(void);
void i2c_setOnReceive(void (*onReceive)(int));
void i2c_setOnRequest(void (*onRequest)(void));
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transfer_direction,
                          uint16_t addr_match_code);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif /* INC_I2C_SLAVE_H_ */
