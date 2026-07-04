/**
 * @file UartPort.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-04-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

#include "UartPort.hpp"

#include <cstring>

UartPort *UartPort::map_[UartPort::kMaxMap] = {nullptr, nullptr, nullptr,
                                               nullptr};

UartPort::UartPort(UART_HandleTypeDef *huart, DMA_USE dma_use, uint8_t *rx_dma_buf,
                   size_t rx_dma_buf_size, uint8_t *tx_dma_buf,
                   size_t tx_dma_buf_size, RxCallback cb, void *cb_user)
    : dma_use_(dma_use), huart_(huart), rx_dma_buf_(rx_dma_buf), rx_dma_buf_size_(rx_dma_buf_size),
      tx_dma_raw_{(tx_dma_buf != nullptr && tx_dma_buf_size >= 2)
                      ? static_cast<void *>(tx_dma_buf)
                      : static_cast<void *>(tx_fallback_),
                  (tx_dma_buf != nullptr && tx_dma_buf_size >= 2)
                      ? tx_dma_buf_size
                      : sizeof(tx_fallback_)},
      tx_dma_buffer_(tx_dma_raw_), rx_callback_(cb),
      rx_callback_user_(cb_user) {
  tx_use_double_buffer_ =
      (tx_dma_buf != nullptr && tx_dma_buf_size >= 2 &&
       (tx_dma_buf_size % 2U == 0U) && tx_dma_buffer_.Size() > 0U);

  for (size_t i = 0; i < kMaxMap; ++i) {
    if (map_[i] == nullptr) {
      map_[i] = this;
      break;
    }
  }
}

HAL_StatusTypeDef UartPort::startRx() {
  if(dma_use_ == DMA_USE::DMA_off) {
    return HAL_OK;
  }
  if (huart_ == nullptr || rx_dma_buf_ == nullptr || rx_dma_buf_size_ == 0) {
    return HAL_ERROR;
  }
  last_rx_pos_ = 0;
  return HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_dma_buf_, rx_dma_buf_size_);
}

HAL_StatusTypeDef UartPort::write(const uint8_t *data, size_t len,
                                  uint32_t timeout_ms) {
  if (huart_ == nullptr || data == nullptr || len == 0) {
    return HAL_ERROR;
  }
  if (len > 0xFFFFU) {
    return HAL_ERROR;
  }
  return HAL_UART_Transmit(huart_, const_cast<uint8_t *>(data),
                           static_cast<uint16_t>(len), timeout_ms);
}

HAL_StatusTypeDef UartPort::writeDma(const uint8_t *data, size_t len) {
  // HAL UART DMA 接口不接受空数据，且传输长度参数为 uint16_t。
  if (huart_ == nullptr || data == nullptr || len == 0) {
    return HAL_ERROR;
  }
  if (len > 0xFFFFU) {
    return HAL_ERROR;
  }

  // 未配置双缓冲区时直接发送调用方提供的数据。
  // DMA 传输完成前调用方必须保证 data 指向的内存持续有效。
  if (!tx_use_double_buffer_) {
    if (tx_busy_) {
      return HAL_BUSY;
    }

    // 在启动 DMA 前置忙，避免前一次发送尚未结束时重复启动。
    tx_busy_ = true;
    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(
        huart_, const_cast<uint8_t *>(data), static_cast<uint16_t>(len));
    if (ret != HAL_OK) {
      // DMA 未成功启动，不会产生发送完成回调，因此在此恢复空闲状态。
      tx_busy_ = false;
    }
    return ret;
  }

  // 当前没有发送任务：将数据复制到活动缓冲区并立即启动 DMA。
  if (!tx_busy_) {
    if (!tx_dma_buffer_.FillActive(data, len)) {
      return HAL_ERROR;
    }

    tx_busy_ = true;
    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(
        huart_, tx_dma_buffer_.ActiveBuffer(),
        static_cast<uint16_t>(tx_dma_buffer_.GetActiveLength()));
    if (ret != HAL_OK) {
      // 启动失败时释放忙状态，允许调用方稍后重试。
      tx_busy_ = false;
    }
    return ret;
  }

  // DMA 正在发送活动缓冲区；待发送缓冲区也已占用时无法继续排队。
  if (tx_dma_buffer_.HasPending()) {
    return HAL_BUSY;
  }

  // 将本次数据复制到待发送缓冲区。当前 DMA 完成后，onTxCplt()
  // 会切换双缓冲区并自动启动这笔发送。
  if (!tx_dma_buffer_.FillPending(data, len)) {
    return HAL_BUSY;
  }

  // 数据已成功排队，并不表示 DMA 已经完成发送。
  return HAL_OK;
}

bool UartPort::Read(Packet &packet) {
  return rx_queue_.TryPop(packet) == Algorithm::QueueError::OK;
}

void UartPort::onRxEvent() {
  if (huart_ == nullptr || huart_->hdmarx == nullptr) {
    return;
  }

  size_t curr_pos = rx_dma_buf_size_ - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
  if (curr_pos == last_rx_pos_) {
    return;
  }

  auto pushRxSpan = [this](const uint8_t *data, size_t len) {
    if (data == nullptr || len == 0) {
      return;
    }

    size_t offset = 0;
    while (offset < len) {
      Packet packet{};
      size_t part = len - offset;
      if (part > sizeof(packet.data)) {
        part = sizeof(packet.data);
      }

      packet.len = static_cast<uint16_t>(part);
      std::memcpy(packet.data, data + offset, part);

      if (rx_queue_.TryPush(packet) != Algorithm::QueueError::OK) {
        break;
      }

      offset += part;
    }
  };

  if (curr_pos > last_rx_pos_) {
    pushRxSpan(rx_dma_buf_ + last_rx_pos_, curr_pos - last_rx_pos_);
    if (rx_callback_ != nullptr) {
      rx_callback_(rx_dma_buf_ + last_rx_pos_, curr_pos - last_rx_pos_,
                   rx_callback_user_);
    }
  } else {
    pushRxSpan(rx_dma_buf_ + last_rx_pos_, rx_dma_buf_size_ - last_rx_pos_);
    if (rx_callback_ != nullptr) {
      rx_callback_(rx_dma_buf_ + last_rx_pos_, rx_dma_buf_size_ - last_rx_pos_,
                   rx_callback_user_);
    }
    if (curr_pos > 0) {
      pushRxSpan(rx_dma_buf_, curr_pos);
      if (rx_callback_ != nullptr) {
        rx_callback_(rx_dma_buf_, curr_pos, rx_callback_user_);
      }
    }
  }

  last_rx_pos_ = curr_pos;
}

void UartPort::onTxCplt() {
  if (!tx_use_double_buffer_) {
    tx_busy_ = false;
    return;
  }

  if (!tx_dma_buffer_.HasPending()) {
    tx_busy_ = false;
    return;
  }

  tx_dma_buffer_.Switch();

  HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(
      huart_, tx_dma_buffer_.ActiveBuffer(),
      static_cast<uint16_t>(tx_dma_buffer_.GetActiveLength()));
  if (ret != HAL_OK) {
    tx_busy_ = false;
  }
}

void UartPort::onError(uint32_t error_code) {
  if (huart_ == nullptr || error_code == HAL_UART_ERROR_NONE) {
    return;
  }

  // UART的 ErrorCallback 绝大多数是因为接收管脚收到噪声产生的 RX 错误 (如 ORE, FE, NE 等)
  // 如果直接调用 HAL_UART_DMAStop，会将正在进行中的 TX DMA 也强行终止！
  // 导致发送任务的数据在有电气噪声时（比如电机工作）被持续强迫打断，进而出现几秒钟无数据然后突然恢复的现象。
  
  // 只停止当前的接收 DMA，不干扰正在发送的 TX 数据
  (void)HAL_UART_AbortReceive(huart_);

  // 清除所有 RX 相关的错误标志位
  __HAL_UART_CLEAR_PEFLAG(huart_);
  __HAL_UART_CLEAR_FEFLAG(huart_);
  __HAL_UART_CLEAR_NEFLAG(huart_);
  __HAL_UART_CLEAR_OREFLAG(huart_);
  __HAL_UART_CLEAR_IDLEFLAG(huart_);
  huart_->ErrorCode = HAL_UART_ERROR_NONE;

  // 重启 RX DMA
  last_rx_pos_ = 0;
  (void)startRx();
}

UartPort *UartPort::fromHandle(UART_HandleTypeDef *huart) {
  if (huart == nullptr) {
    return nullptr;
  }
  for (size_t i = 0; i < kMaxMap; ++i) {
    if (map_[i] != nullptr && map_[i]->handle() == huart) {
      return map_[i];
    }
  }
  return nullptr;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t) {
  UartPort *port = UartPort::fromHandle(huart);
  if (port != nullptr) {
    port->onRxEvent();
  }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  UartPort *port = UartPort::fromHandle(huart);
  if (port != nullptr) {
    port->onTxCplt();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  UartPort *port = UartPort::fromHandle(huart);
  if (port != nullptr) {
    port->onError(huart->ErrorCode);
  }
}
