#include "i2c_slave.h"

#include <stdlib.h>
#include <string.h>

#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;
#define DEFINE_I2C hi2c1

static const uint8_t MASTER_ADDRESS = 0x01;

static void _onReceiveService(i2c_t *obj);
static void _onRequestService(i2c_t *obj);
static void _allocateRxBuffer(i2c_t *obj, int length);

i2c_t _i2c;

// i2c_readAddress() << 1
void i2c_begin() {
  if (_i2c._tx_buffer) {
    free(_i2c._tx_buffer);
    _i2c._tx_buffer = NULL;
  }

  if (_i2c._rx_buffer) {
    free(_i2c._rx_buffer);
    _i2c._rx_buffer = NULL;
  }

  _i2c._transmitting = false;
  _i2c._rx_buffer_index = 0;
  _i2c._rx_buffer_length = 0;
  _i2c._rx_buffer_allocated = 0;

  MX_I2C1_Init();

  _i2c._handle = hi2c1;
  uint8_t address = _i2c._handle.Init.OwnAddress1 >> 1;
  UART_Transmit("[DEBUG]: I2C Address 0x%02x\n", address);
  if (_i2c._handle.Init.OwnAddress1 != MASTER_ADDRESS) {
    _i2c._i2c_onSlaveReceive = _onReceiveService;
    _i2c._i2c_onSlaveTransmit = _onRequestService;

    HAL_I2C_EnableListen_IT(&hi2c1);
  }
}

int i2c_available() {
  return _i2c._rx_buffer_length - _i2c._rx_buffer_index;
}

int i2c_read() {
  int value = -1;

  if (_i2c._rx_buffer_index < _i2c._rx_buffer_length) {
    value = _i2c._rx_buffer[_i2c._rx_buffer_index++];
  }

  return value;
}

uint8_t i2c_write(uint8_t data) {
  if (i2c_slave_write_IT(&_i2c, &data, 1) != HAL_OK) {
    return 0;
  }
  return 1;
}

HAL_StatusTypeDef i2c_slave_write_IT(i2c_t *obj, uint8_t *data, uint16_t size) {
  uint8_t i = 0;
  HAL_StatusTypeDef ret = HAL_OK;

  if (size > I2C_TX_RX_BUFFER_SIZE) {
    ret = HAL_TIMEOUT;
  } else {
    for (i = 0; i < size; i++) {
      obj->_i2c_tx_rx_buffer[obj->_i2c_tx_rx_buffer_size + i] = *(data + i);
    }

    obj->_i2c_tx_rx_buffer_size += size;
  }
  return ret;
}

void i2c_setOnReceive(void (*onReceive)(int)) {
  _i2c.onReceive = onReceive;
}

void i2c_setOnRequest(void (*onRequest)(void)) {
  _i2c.onRequest = onRequest;
}

uint8_t i2c_readAddress() {
  return (0xc << 3) | (GPIOB->IDR >> 13);
}

void _onReceiveService(i2c_t *obj) {
  uint8_t *in_bytes = (uint8_t *)obj->_i2c_tx_rx_buffer;
  int num_bytes = obj->_slave_rx_nb_data;

  if (obj->onReceive && obj->_rx_buffer_index >= obj->_rx_buffer_length) {
    _allocateRxBuffer(obj, num_bytes);

    memcpy(obj->_rx_buffer, in_bytes, num_bytes);
    obj->_rx_buffer_index = 0;
    obj->_rx_buffer_length = num_bytes;
    obj->onReceive(num_bytes);
  }
}

void _onRequestService(i2c_t *obj) {
  if (obj->onRequest) {
    obj->_tx_data_size = 0;
    obj->onRequest();
  }
}

void _allocateRxBuffer(i2c_t *obj, int length) {
  if (obj->_rx_buffer_allocated < length) {
    if (length < BUFFER_LENGTH) {
      length = BUFFER_LENGTH;
    }

    uint8_t *tmp = (uint8_t *)realloc(obj->_rx_buffer, length);
    if (tmp != NULL) {
      obj->_rx_buffer = tmp;
      obj->_rx_buffer_allocated = length;
    } else UART_Transmit("No enough memory! (%i)\n", length);
  }
}

/* -------------------------------------------------------------------------- */
/* ---------------------------- HAL Event Handle ---------------------------- */
/* -------------------------------------------------------------------------- */

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transfer_direction,
                          uint16_t addr_match_code) {
  if (hi2c->Instance != DEFINE_I2C.Instance) return;

  if (_i2c._slave_mode == I2C_SLAVE_MODE_RECEIVE && _i2c._slave_rx_nb_data) {
    _i2c._i2c_onSlaveReceive(&_i2c);
    _i2c._slave_mode = I2C_SLAVE_MODE_LISTEN;
    _i2c._slave_rx_nb_data = 0;
  }

  if (transfer_direction == I2C_DIRECTION_RECEIVE) {
    _i2c._slave_mode = I2C_SLAVE_MODE_TRANSMIT;

    if (_i2c._i2c_onSlaveTransmit != NULL) {
      _i2c._i2c_tx_rx_buffer_size = 0;
      _i2c._i2c_onSlaveTransmit(&_i2c);
    }

    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t *)_i2c._i2c_tx_rx_buffer,
                                  _i2c._i2c_tx_rx_buffer_size, I2C_LAST_FRAME);
  } else {
    _i2c._slave_rx_nb_data = 0;
    _i2c._slave_mode = I2C_SLAVE_MODE_RECEIVE;

    HAL_I2C_Slave_Seq_Receive_IT(
        hi2c, (uint8_t *)&(_i2c._i2c_tx_rx_buffer[_i2c._slave_rx_nb_data]), 1,
        I2C_NEXT_FRAME);
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance != DEFINE_I2C.Instance) return;

  if (_i2c._slave_mode == I2C_SLAVE_MODE_RECEIVE && _i2c._slave_rx_nb_data) {
    _i2c._i2c_onSlaveReceive(&_i2c);
  }
  _i2c._slave_mode = I2C_SLAVE_MODE_LISTEN;
  _i2c._slave_rx_nb_data = 0;
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (_i2c._slave_rx_nb_data < I2C_TX_RX_BUFFER_SIZE) {
    _i2c._slave_rx_nb_data++;
  } else {
    UART_Transmit("[ERROR] [I2C]: Slave RX overflow\n");
  }

  if (_i2c._slave_mode == I2C_SLAVE_MODE_RECEIVE) {
    HAL_I2C_Slave_Seq_Receive_IT(
        hi2c, (uint8_t *)&(_i2c._i2c_tx_rx_buffer[_i2c._slave_rx_nb_data]), 1,
        I2C_NEXT_FRAME);
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  _i2c._i2c_tx_rx_buffer_size = 0;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  int err_code = HAL_I2C_GetError(hi2c);
  if (err_code != HAL_I2C_ERROR_AF) {
    UART_Transmit("[ERROR] [I2C]: code %d\n", err_code);
  }

  if (_i2c._handle.Init.OwnAddress1 != MASTER_ADDRESS) {
    HAL_I2C_EnableListen_IT(hi2c);
  }
}
