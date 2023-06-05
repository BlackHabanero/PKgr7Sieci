#include "mcp2515.h"

/**
 * @file mcp2515.c
 * @brief This file provides functions for using Microchip MCP2515 CAN
 * controller
 * @author Artur Poteraj
 * @copyright
 * Copyright 2023 Artur Poteraj
 *
 * Permission is hereby granted, free of charge,
 * to any person obtaining a copy of this software and associated documentation
 * files (the “Software”), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the
 * following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
 * EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

static HAL_StatusTypeDef MCP2515_AbortAllPendingTransmissions(
    MCP2515_HandleTypeDef* hmcp2515,
    uint8_t abat);

static uint8_t MCP2515_PinConfig(MCP2515_HandleTypeDef* hmcp2515,
                                 uint8_t rxbf_pins,
                                 uint8_t txrts_pins);

static void MCP2515_ConvertFrameID(uint8_t* out, uint32_t in);

static HAL_StatusTypeDef MCP2515_OneShotMode(MCP2515_HandleTypeDef* hmcp2515,
                                             uint8_t osm);

static HAL_StatusTypeDef MCP2515_ReceiveFilter(MCP2515_HandleTypeDef* hmcp2515,
                                               MCP2515_RXBn rxbn,
                                               uint8_t value);

static HAL_StatusTypeDef MCP2515_Rollover(MCP2515_HandleTypeDef* hmcp2515,
                                          uint8_t value);

// TODO improve existing and add missing doxygen comments

uint8_t MCP2515_Init(MCP2515_HandleTypeDef* hmcp2515,
                     uint8_t rxbf_pins,
                     uint8_t txrts_pins,
                     uint8_t init_flags,
                     uint8_t clkout_flags) {
  uint8_t rst = MCP2515_RESET;
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  uint8_t status = HAL_SPI_Transmit(hmcp2515->hspi, &rst, 1, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  if (status)
    return status;

  status = MCP2515_PinConfig(hmcp2515, rxbf_pins, txrts_pins);
  if (status)
    return status;

  uint8_t pData[4] = {
      MCP2515_BIT_MODIFY, MCP2515_CNF3, 192,
      ((!!(init_flags & MCP2515_WAKFIL)) << 6) | (clkout_flags & MCP2515_SOF)};
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(hmcp2515->hspi, pData, 4, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  if (status)
    return status;

  pData[1] = MCP2515_CANCTRL;
  pData[2] = 7;
  pData[3] = clkout_flags;
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(hmcp2515->hspi, pData, 4, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);

  return status;
}

/**
 * @brief Configures RXnBF and TXnRTS pins of MCP2515 CAN controller.
 * @param hmcp2515 MCP2515 handle.
 * @param rxbf_pins Use MCP2515_RXBn enum to specify.
 * @param txrts_pins Use MCP2515_TXBn enum to specify.
 * @attention Only available when device is in configuration mode
 * @return HAL_StatusTypeDef or MCP2515_NOT_IN_CONFIGMODE when device not in
 * configuration mode.
 */
static uint8_t MCP2515_PinConfig(MCP2515_HandleTypeDef* hmcp2515,
                                 uint8_t rxbf_pins,
                                 uint8_t txrts_pins) {
  uint8_t is_config_mode;
  HAL_StatusTypeDef status =
      MCP2515_IsInConfigurationMode(hmcp2515, &is_config_mode);
  if (status)
    return status;
  if (!is_config_mode)
    return MCP2515_NOT_IN_CONFIGMODE;

  uint8_t rxbf;
  if (rxbf_pins & MCP2515_RXB0)
    rxbf |= 0b0101;
  if (rxbf_pins & MCP2515_RXB1)
    rxbf |= 0b1010;

  uint8_t pData[4] = {MCP2515_WRITE, MCP2515_BFPCTRL, (rxbf_pins & 15),
                      (txrts_pins & 7)};

  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(hmcp2515->hspi, pData, 3, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);

  return status;
}

/// @brief Reads receive error count register from MCP2515 CAN controller.
/// @param hmcp2515 MCP2515 handle.
/// @param interrupts use MCP2515_Interrupt enum to specify.
/// @attention Only available when device is in configuration mode
/// @return SPI status or MCP2515_NOT_IN_CONFIGMODE when device not in
/// configuration mode.
HAL_StatusTypeDef MCP2515_EnableInterrupts(MCP2515_HandleTypeDef* hmcp2515,
                                           uint8_t interrupts) {
  uint8_t is_config_mode;
  HAL_StatusTypeDef status =
      MCP2515_IsInConfigurationMode(hmcp2515, &is_config_mode);
  if (status)
    return status;
  if (!is_config_mode)
    return MCP2515_NOT_IN_CONFIGMODE;

  uint8_t pData[3] = {MCP2515_WRITE, MCP2515_CANINTE, interrupts};

  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(hmcp2515->hspi, pData, 3, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);

  return status;
}

HAL_StatusTypeDef MCP2515_ClearInterruptFlag(MCP2515_HandleTypeDef* hmcp2515,
                                             MCP2515_Interrupt interrupt) {
  uint8_t pData[4] = {MCP2515_BIT_MODIFY, MCP2515_CANINTF, interrupt, 0};

  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, pData, 4, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);

  return status;
}

HAL_StatusTypeDef MCP2515_ClearErrorFlag(MCP2515_HandleTypeDef* hmcp2515,
                                             MCP2515_ErrorFlag flag) {
  uint8_t pData[4] = {MCP2515_BIT_MODIFY, MCP2515_EFLG, flag, 0};

  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, pData, 4, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);

  return status;
}

/// @brief Reads interrupt flags register from MCP2515 CAN controller.
/// @param hmcp2515 MCP2515 handle.
/// @param[out] out register contents. Interrupt flags in MCP2515_Interrupt
/// enum.
/// @return SPI status.
HAL_StatusTypeDef MCP2515_GetInterruptFlags(MCP2515_HandleTypeDef* hmcp2515,
                                            uint8_t* out) {
  return MCP2515_GetRegister(hmcp2515, MCP2515_CANINTF, out);
}

/// @brief Reads transmit error count register from MCP2515 CAN controller.
/// @param hmcp2515 MCP2515 handle.
/// @param[out] out register contents.
/// @return SPI status.
HAL_StatusTypeDef MCP2515_GetTransmitErrorCount(MCP2515_HandleTypeDef* hmcp2515,
                                                uint8_t* out) {
  return MCP2515_GetRegister(hmcp2515, MCP2515_TEC, out);
}

/// @brief Reads receive error count register from MCP2515 CAN controller.
/// @param hmcp2515 MCP2515 handle.
/// @param[out] out register contents.
/// @return SPI status.
HAL_StatusTypeDef MCP2515_GetReceiveErrorCount(MCP2515_HandleTypeDef* hmcp2515,
                                               uint8_t* out) {
  return MCP2515_GetRegister(hmcp2515, MCP2515_REC, out);
}

/// @brief Reads error flag register from MCP2515 CAN controller.
/// @param hmcp2515 MCP2515 handle.
/// @param[out] out register contents.
/// @return SPI status.
HAL_StatusTypeDef MCP2515_GetErrorFlagRegister(MCP2515_HandleTypeDef* hmcp2515,
                                               uint8_t* out) {
  return MCP2515_GetRegister(hmcp2515, MCP2515_EFLG, out);
}

/// @brief Reads control register from MCP2515 CAN controller.
/// @param hmcp2515 MCP2515 handle.
/// @param registr register address.
/// @param[out] out register contents.
/// @return SPI status.
HAL_StatusTypeDef MCP2515_GetRegister(MCP2515_HandleTypeDef* hmcp2515,
                                      MCP2515_Control_Register registr,
                                      uint8_t* out) {
  uint8_t pData[2] = {MCP2515_READ, registr};

  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, pData, 2, 100);
  if (status) {
    HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
    return status;
  }
  status = HAL_SPI_Receive(hmcp2515->hspi, out, 1, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  return status;
}

/**
 * @brief Converts continuous CAN Frame to MCP2515 registers' format
 * @param[out] out CAN Frame in MCP2515 registers' format {sidh, sidl, eid8,
 * eid0}
 * @param[in] in CAN Frame in continuous format (29b or 11b)
 * @warning ensure out is 4 elements wide!
 */
static void MCP2515_ConvertFrameID(uint8_t* out, uint32_t in) {
  uint8_t sidh = (in >> 3) & 255;  // higher 8b of std frame
  uint8_t sidl = (in & 7) << 5;    // lower 3b of std frame

  sidl |= (in >> 27) & 3;           // 17th and 16th bit of ext frame
  uint8_t eid8 = (in >> 19) & 255;  // 15th-8th bit of ext frame
  uint8_t eid0 = (in >> 11) & 255;  // 7th-0th bit of ext frame

  out[0] = sidh;
  out[1] = sidl;
  out[2] = eid8;
  out[3] = eid0;
}

/**
 * @brief Configures the message acceptance filters of MCP2515 CAN controller.
 * @param hmcp2515 MCP2515 handle.
 * @param frametype type of CAN frame (standard 11b/extended 29b).
 * @param filter acceptance filter selector.
 * @param filterbits filter.
 * @attention Only available when device is in configuration mode.
 * @return SPI status or MCP2515_NOT_IN_CONFIGMODE when device not in
 * configuration mode.
 */
HAL_StatusTypeDef MCP2515_SetReceiveFilter(MCP2515_HandleTypeDef* hmcp2515,
                                      MCP2515_CAN_FrameType frametype,
                                      MCP2515_AcceptanceFilter filter,
                                      uint32_t filterbits) {
  uint8_t is_config_mode;
  HAL_StatusTypeDef status =
      MCP2515_IsInConfigurationMode(hmcp2515, &is_config_mode);
  if (status)
    return status;
  if (!is_config_mode)
    return MCP2515_NOT_IN_CONFIGMODE;

  uint8_t filter_regs[4];
  MCP2515_ConvertFrameID(filter_regs, filterbits);
  filter_regs[1] |=
      (frametype << 3);  // whether to apply filter to std (0) or ext (1) frame

  uint8_t pData[6] = {MCP2515_WRITE,  filter,         filter_regs[0],
                      filter_regs[1], filter_regs[2], filter_regs[3]};
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(hmcp2515->hspi, pData, 6, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);

  return status;
}

/**
 * @brief Configures the message filter masks of MCP2515 CAN controller.
 * @param hmcp2515 MCP2515 handle.
 * @param mask mask selector.
 * @param maskbits filter mask.
 * @attention Only available when device is in configuration mode.
 * @return SPI status or MCP2515_NOT_IN_CONFIGMODE when device not in
 * configuration mode.
 */
HAL_StatusTypeDef MCP2515_SetReceiveMask(MCP2515_HandleTypeDef* hmcp2515,
                                    MCP2515_RX_Mask mask,
                                    uint32_t maskbits) {
  uint8_t is_config_mode;
  HAL_StatusTypeDef status =
      MCP2515_IsInConfigurationMode(hmcp2515, &is_config_mode);
  if (status)
    return status;
  if (!is_config_mode)
    return MCP2515_NOT_IN_CONFIGMODE;

  uint8_t mask_regs[4];
  MCP2515_ConvertFrameID(mask_regs, maskbits);

  uint8_t pData[6] = {MCP2515_WRITE, mask,         mask_regs[0],
                      mask_regs[1],  mask_regs[2], mask_regs[3]};
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(hmcp2515->hspi, pData, 6, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);

  return status;
}

/// @brief Configures registers responsible for CAN bit rate in MCP2515 CAN
/// controller.
/// @param hmcp2515 MCP2515 handle.
/// @param BRP Baud Rate Prescaler register.
/// @param SJW Synchronization Jump Width register.
/// @param PRSEG Propagation Segment register.
/// @param PHSEG1 Phase Segment 1 register.
/// @param PHSEG2 Phase Segment 2 register.
/// @attention Only available when device is in configuration mode.
/// @return SPI status or MCP2515_NOT_IN_CONFIGMODE when device not in
/// configuration mode.
HAL_StatusTypeDef MCP2515_SetTimeSegments(MCP2515_HandleTypeDef* hmcp2515,
                                          uint8_t BRP,
                                          uint8_t SJW,
                                          uint8_t PRSEG,
                                          uint8_t PHSEG1,
                                          uint8_t PHSEG2) {
  uint8_t is_config_mode;
  HAL_StatusTypeDef status =
      MCP2515_IsInConfigurationMode(hmcp2515, &is_config_mode);
  if (status)
    return status;
  if (!is_config_mode)
    return MCP2515_NOT_IN_CONFIGMODE;

  uint8_t masks[3] = {255, 63, 7};
  uint8_t values[3] = {((SJW & 3) << 6) | (BRP & 63),
                       ((PHSEG1 & 7) << 3) | (PRSEG & 7), PHSEG2};
  uint8_t pData[4] = {MCP2515_BIT_MODIFY, MCP2515_CNF3, 0, 0};

  // iterates over CNF3, CNF2 and CNF1 registers.
  // CNF3 and CNF2 contain fields that are set elsewhere
  // so care has to be taken not to modify those by accident.
  // That is why bit modify instruction is used here.
  for (uint8_t i = 3; i; i--) {
    pData[2] = masks[i];
    pData[3] = values[i];
    HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hmcp2515->hspi, pData, 4, 100);
    HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
    if (status)
      return status;
    ++pData[1];
  }
  return HAL_OK;
}

HAL_StatusTypeDef MCP2515_SetTransmitBuffer(MCP2515_HandleTypeDef* hmcp2515,
                                            MCP2515_TXBn txbn,
                                            MCP2515_CAN_FrameType frametype,
                                            uint32_t frame_id,
                                            uint8_t* data,
                                            uint8_t datasize) {
  uint8_t fid_regs[4];
  MCP2515_ConvertFrameID(fid_regs, frame_id);
  fid_regs[1] |=
      (frametype
       << 3);  // whether message will transmit standard or extended frame
  uint8_t dlc = datasize & 15;
  uint8_t pData[15];
  pData[0] = MCP2515_LOAD_TX_BUF | (txbn & 6);
  pData[1] = txbn;
  for (uint8_t i = 4; i; i--) {
    pData[i + 1] = fid_regs[i - 1];
  }
  pData[6] = dlc;
  for (uint8_t i = datasize; i; i--) {
    pData[7 + i] = data[i];
  }

  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, pData, 7 + datasize, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);

  return status;
}

HAL_StatusTypeDef MCP2515_GetReceiveBuffer(MCP2515_HandleTypeDef* hmcp2515,
                                           MCP2515_RXBn rxbn,
                                           MCP2515_CAN_FrameType* frametype,
                                           uint32_t* frame_id,
                                           uint8_t* data,
                                           uint8_t* datasize) {
  uint8_t pData = MCP2515_READ_RX_BUF | ((rxbn & 2) << 1);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, &pData, 1, 100);
  if (status) {
    HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
    return status;
  }
  uint8_t receivd[13];
  status = HAL_SPI_Receive(hmcp2515->hspi, receivd, 13, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  if (status)
    return status;

  *frame_id = ((receivd[1] >> 5) & 7) | (((uint16_t)receivd[0]) << 3) |
              (((uint32_t)(receivd[1] & 3) << 27)) |
              (((uint32_t)receivd[2]) << 19) | (((uint32_t)receivd[3]) << 11);
  *frametype = receivd[1] & (1 << 3);
  *datasize = receivd[4] & 15;
  for (uint8_t i = 8; i; i--) {
    data[i] = receivd[i + 5];
  }
  status = MCP2515_ClearInterruptFlag(hmcp2515, rxbn);
  if (status)
    return status;
  status = MCP2515_ClearErrorFlag(hmcp2515, 64 * rxbn);
  
  return status;
}

HAL_StatusTypeDef MCP2515_SetOperationMode(MCP2515_HandleTypeDef* hmcp2515,
                                           MCP2515_OperationMode mode) {
  uint8_t pData[4] = {MCP2515_BIT_MODIFY, MCP2515_CANCTRL, 0b11100000,
                      (mode & 7) << 5};
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, pData, 4, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  return status;
}

HAL_StatusTypeDef MCP2515_IsInConfigurationMode(MCP2515_HandleTypeDef* hmcp2515,
                                                uint8_t* is_config_mode) {
  uint8_t mode;

  HAL_StatusTypeDef status = MCP2515_GetOperationMode(hmcp2515, &mode);

  *is_config_mode = !!(mode & MCP2515_CONFIGURATION_MODE);

  return status;
}

HAL_StatusTypeDef MCP2515_GetOperationMode(MCP2515_HandleTypeDef* hmcp2515,
                                           MCP2515_OperationMode* mode) {
  uint8_t canstat;

  HAL_StatusTypeDef status =
      MCP2515_GetRegister(hmcp2515, MCP2515_CANSTAT, &canstat);
  *mode = (canstat >> 5);

  return status;
}

static HAL_StatusTypeDef MCP2515_AbortAllPendingTransmissions(
    MCP2515_HandleTypeDef* hmcp2515,
    uint8_t abat) {
  uint8_t pData[4] = {MCP2515_BIT_MODIFY, MCP2515_CANCTRL, 0b00010000,
                      (!!abat) << 4};
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, pData, 4, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  return status;
}

HAL_StatusTypeDef MCP2515_RequestAbortAllPendingTransmissions(
    MCP2515_HandleTypeDef* hmcp2515) {
  return MCP2515_AbortAllPendingTransmissions(hmcp2515, 1);
}

HAL_StatusTypeDef MCP2515_TerminateAbortAllPendingTransmissions(
    MCP2515_HandleTypeDef* hmcp2515) {
  return MCP2515_AbortAllPendingTransmissions(hmcp2515, 0);
}

static HAL_StatusTypeDef MCP2515_OneShotMode(MCP2515_HandleTypeDef* hmcp2515,
                                             uint8_t osm) {
  uint8_t pData[4] = {MCP2515_BIT_MODIFY, MCP2515_CANCTRL, 0b00001000,
                      (!!osm) << 3};
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, pData, 4, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  return status;
}

HAL_StatusTypeDef MCP2515_EnableOneShotMode(MCP2515_HandleTypeDef* hmcp2515) {
  return MCP2515_OneShotMode(hmcp2515, 1);
}

HAL_StatusTypeDef MCP2515_DisableOneShotMode(MCP2515_HandleTypeDef* hmcp2515) {
  return MCP2515_OneShotMode(hmcp2515, 0);
}

HAL_StatusTypeDef MCP2515_SetTransmitBufferPriority(
    MCP2515_HandleTypeDef* hmcp2515,
    MCP2515_TXBn txbn,
    MCP2515_TXB_Priority priority) {
  uint8_t address = MCP2515_TXB0CTRL + ((txbn & 0b110) << 3);
  uint8_t pData[4] = {MCP2515_BIT_MODIFY, address, 3, priority};
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, pData, 4, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  return status;
}

HAL_StatusTypeDef MCP2515_RequestToSend(MCP2515_HandleTypeDef* hmcp2515,
                                        uint8_t txbs) {
  uint8_t pData = MCP2515_RTS | (txbs & 7);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, &pData, 1, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  return status;
}

HAL_StatusTypeDef MCP2515_GetTXB_Status(MCP2515_HandleTypeDef* hmcp2515,
                                        MCP2515_TXBn txbn,
                                        uint8_t* status) {
  uint8_t address = MCP2515_TXB0CTRL + ((txbn & 0b110) << 3);
  return MCP2515_GetRegister(hmcp2515, address, status);
}

static HAL_StatusTypeDef MCP2515_ReceiveFilter(MCP2515_HandleTypeDef* hmcp2515,
                                               MCP2515_RXBn rxbn,
                                               uint8_t value) {
  uint8_t address = MCP2515_RXB0CTRL + (16 * (rxbn >> 1));
  uint8_t pData[4] = {MCP2515_BIT_MODIFY, address, 96, value};
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, pData, 1, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  return status;
}

HAL_StatusTypeDef MCP2515_EnableReceiveFilter(MCP2515_HandleTypeDef* hmcp2515,
                                              MCP2515_RXBn rxbn) {
  return MCP2515_ReceiveFilter(hmcp2515, rxbn, 0);
}

HAL_StatusTypeDef MCP2515_DisableReceiveFilter(MCP2515_HandleTypeDef* hmcp2515,
                                               MCP2515_RXBn rxbn) {
  return MCP2515_ReceiveFilter(hmcp2515, rxbn, 255);
}

static HAL_StatusTypeDef MCP2515_Rollover(MCP2515_HandleTypeDef* hmcp2515,
                                          uint8_t value) {
  uint8_t pData[4] = {MCP2515_BIT_MODIFY, MCP2515_RXB0CTRL, 4, value};
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hmcp2515->hspi, pData, 1, 100);
  HAL_GPIO_WritePin(hmcp2515->cs_base, hmcp2515->cs_pin, GPIO_PIN_SET);
  return status;
}

HAL_StatusTypeDef MCP2515_EnableRollover(MCP2515_HandleTypeDef* hmcp2515) {
  return MCP2515_Rollover(hmcp2515, 255);
}

HAL_StatusTypeDef MCP2515_DisableRollover(MCP2515_HandleTypeDef* hmcp2515) {
  return MCP2515_Rollover(hmcp2515, 0);
}

HAL_StatusTypeDef MCP2515_GetRXB_Status(MCP2515_HandleTypeDef* hmcp2515,
                                        MCP2515_RXBn rxbn,
                                        uint8_t* status) {
  uint8_t address = MCP2515_RXB0CTRL + (16 * (rxbn >> 1));
  return MCP2515_GetRegister(hmcp2515, address, status);
}