#ifndef __MCP2515_H
#define __MCP2515_H

#include "stm32l4xx_hal.h"

typedef enum {
  MCP2515_RESET = (uint8_t)0b11000000,
  MCP2515_READ = (uint8_t)0b00000011,
  MCP2515_READ_RX_BUF = (uint8_t)0b10010000,  // 1001 0nm0
  MCP2515_WRITE = (uint8_t)0b00000010,
  MCP2515_LOAD_TX_BUF = (uint8_t)0b01000000,  // 0100 0abc
  MCP2515_RTS = (uint8_t)0b10000000,          // 1000 0nnn
  MCP2515_READ_STATUS = (uint8_t)0b10100000,
  MCP2515_RX_STATUS = (uint8_t)0b10110000,
  MCP2515_BIT_MODIFY = (uint8_t)0b00000101
} MCP2515_Instruction;

typedef enum {
  MCP2515_BFPCTRL = (uint8_t)0x0C,
  MCP2515_TXRTSCTRL = (uint8_t)0x0D,
  MCP2515_CANSTAT = (uint8_t)0x0E,  // 0xXE
  MCP2515_CANCTRL = (uint8_t)0x0F,  // 0xXF
  MCP2515_TEC = (uint8_t)0x1C,
  MCP2515_REC = (uint8_t)0x1D,
  MCP2515_CNF3 = (uint8_t)0x28,
  MCP2515_CNF2 = (uint8_t)0x29,
  MCP2515_CNF1 = (uint8_t)0x2A,
  MCP2515_CANINTE = (uint8_t)0x2B,
  MCP2515_CANINTF = (uint8_t)0x2C,
  MCP2515_EFLG = (uint8_t)0x2D,
  MCP2515_TXB0CTRL = (uint8_t)0x30,
  MCP2515_TXB1CTRL = (uint8_t)0x40,
  MCP2515_TXB2CTRL = (uint8_t)0x50,
  MCP2515_RXB0CTRL = (uint8_t)0x60,
  MCP2515_RXB1CTRL = (uint8_t)0x70
} MCP2515_Control_Register;

typedef enum {
  MCP2515_FILTER0 = (uint8_t)0x00,
  MCP2515_FILTER1 = (uint8_t)0x04,
  MCP2515_FILTER2 = (uint8_t)0x08,
  MCP2515_FILTER3 = (uint8_t)0x10,
  MCP2515_FILTER4 = (uint8_t)0x14,
  MCP2515_FILTER5 = (uint8_t)0x18
} MCP2515_AcceptanceFilter;

typedef enum {
  MCP2515_RX0_MASK = (uint8_t)0x20,
  MCP2515_RX1_MASK = (uint8_t)0x24
} MCP2515_RX_Mask;

typedef enum {
  MCP2515_CAN_STDFRAME = 0,
  MCP2515_CAN_EXTFRAME = 1
} MCP2515_CAN_FrameType;

typedef enum {
  MCP2515_RX0BF = (uint8_t)0b0101,
  MCP2515_RX1BF = (uint8_t)0b1010
} MCP2515_RXBn_Pin;

typedef enum {
  MCP2515_TX0RTS = (uint8_t)(1 << 0),
  MCP2515_TX1RTS = (uint8_t)(1 << 1),
  MCP2515_TX2RTS = (uint8_t)(1 << 2)
} MCP2515_TXBn_Pin;

typedef enum {
  MCP2515_TXB0 = (uint8_t)(0),
  MCP2515_TXB1 = (uint8_t)(2),
  MCP2515_TXB2 = (uint8_t)(4)
} MCP2515_TXBn;

typedef enum {
  MCP2515_RXB0 = (uint8_t)(0),
  MCP2515_RXB1 = (uint8_t)(2)
} MCP2515_RXBn;

typedef enum {
  MCP2515_RX0I = (uint8_t)(1 << 0),
  MCP2515_RX1I = (uint8_t)(1 << 1),
  MCP2515_TX0I = (uint8_t)(1 << 2),
  MCP2515_TX1I = (uint8_t)(1 << 3),
  MCP2515_TX2I = (uint8_t)(1 << 4),
  MCP2515_ERRI = (uint8_t)(1 << 5),
  MCP2515_WAKI = (uint8_t)(1 << 6),
  MCP2515_MERR = (uint8_t)(1 << 7)
} MCP2515_Interrupt;

typedef enum {
  MCP2515_NORMAL_MODE = (uint8_t)0,
  MCP2515_SLEEP_MODE = (uint8_t)1,
  MCP2515_LOOPBACK_MODE = (uint8_t)2,
  MCP2515_LISTENONLY_MODE = (uint8_t)3,
  MCP2515_CONFIGURATION_MODE = (uint8_t)4
} MCP2515_OperationMode;

typedef enum {
  MCP2515_EWARN = (uint8_t)(1 << 0),
  MCP2515_RXWAR = (uint8_t)(1 << 1),
  MCP2515_TXWAR = (uint8_t)(1 << 2),
  MCP2515_RXEP = (uint8_t)(1 << 3),
  MCP2515_TXEP = (uint8_t)(1 << 4),
  MCP2515_TXBO = (uint8_t)(1 << 5),
  MCP2515_RX0OVR = (uint8_t)(1 << 6),
  MCP2515_RX1OVR = (uint8_t)(1 << 7)
} MCP2515_ErrorFlag;

typedef struct {
  uint16_t cs_pin;
  GPIO_TypeDef* cs_base;
  SPI_HandleTypeDef* hspi;
} MCP2515_HandleTypeDef;

// HAL_StatusTypeDef MCP2515_Init(MCP2515_HandleTypeDef* hmcp2515);

// TODO add CLKOUT config to _PinConfig(...)
HAL_StatusTypeDef MCP2515_PinConfig(MCP2515_HandleTypeDef* hmcp2515,
                                    uint8_t rxbf_pins,
                                    uint8_t txrts_pins);

void MCP2515_ConvertFrameID(uint8_t* out, uint32_t in);

HAL_StatusTypeDef MCP2515_SetRXFilter(MCP2515_HandleTypeDef* hmcp2515,
                                      MCP2515_CAN_FrameType frametype,
                                      MCP2515_AcceptanceFilter filter,
                                      uint32_t filterbits);

HAL_StatusTypeDef MCP2515_SetRXMask(MCP2515_HandleTypeDef* hmcp2515,
                                    MCP2515_RX_Mask mask,
                                    uint32_t maskbits);

HAL_StatusTypeDef MCP2515_SetTimeSegments(MCP2515_HandleTypeDef* hmcp2515,
                                          uint8_t BRP,
                                          uint8_t SJW,
                                          uint8_t PRSEG,
                                          uint8_t PHSEG1,
                                          uint8_t PHSEG2);

HAL_StatusTypeDef MCP2515_GetRegister(MCP2515_HandleTypeDef* hmcp2515,
                                      MCP2515_Control_Register registr,
                                      uint8_t* out);

HAL_StatusTypeDef MCP2515_GetTransmitErrorCount(MCP2515_HandleTypeDef* hmcp2515,
                                                uint8_t* out);

HAL_StatusTypeDef MCP2515_GetReceiveErrorCount(MCP2515_HandleTypeDef* hmcp2515,
                                               uint8_t* out);

HAL_StatusTypeDef MCP2515_GetErrorFlagRegister(MCP2515_HandleTypeDef* hmcp2515,
                                               uint8_t* out);

HAL_StatusTypeDef MCP2515_GetInterruptFlags(MCP2515_HandleTypeDef* hmcp2515,
                                            uint8_t* out);

HAL_StatusTypeDef MCP2515_ClearInterruptFlag(MCP2515_HandleTypeDef* hmcp2515,
                                             MCP2515_Interrupt interrupt);

HAL_StatusTypeDef MCP2515_EnableInterrupts(MCP2515_HandleTypeDef* hmcp2515,
                                           uint8_t interrupts);

HAL_StatusTypeDef MCP2515_SetTransmitBuffer(MCP2515_HandleTypeDef* hmcp2515,
                                            MCP2515_TXBn txbn,
                                            MCP2515_CAN_FrameType frametype,
                                            uint32_t frame_id,
                                            uint8_t* data,
                                            uint8_t datasize);

HAL_StatusTypeDef MCP2515_GetReceiveBuffer(MCP2515_HandleTypeDef* hmcp2515,
                                           MCP2515_RXBn rxbn,
                                           MCP2515_CAN_FrameType* frametype,
                                           uint32_t* frame_id,
                                           uint8_t* data,
                                           uint8_t* datasize);

HAL_StatusTypeDef MCP2515_ChangeOperationMode(MCP2515_HandleTypeDef* hmcp2515,
                                              MCP2515_OperationMode mode);

HAL_StatusTypeDef MCP2515_IsInConfigurationMode(MCP2515_HandleTypeDef* hmcp2515, uint8_t* is_config_mode);

// TODO WakeUp filter
// TODO (TX/RX)CTRL regs
#endif /* __MCP2515_H */