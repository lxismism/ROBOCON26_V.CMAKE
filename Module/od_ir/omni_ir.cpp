#include "omni_ir.hpp"
#include <cstdint>

IR_SINGLE::IR_SINGLE(UartPort *uart_port, void (*on_frame_func)(IR_FRAME_t *))
    : on_frame_func_(on_frame_func), uart_port_(uart_port)
{
  last_tx_time_ = 0;
  biggest_rx_uid_ = 0;
}

bool IR_SINGLE::processData(const uint8_t *data, size_t len) {
    if(data == nullptr || len == 0) {
        return false;
    }

    bool frame_complete = false;
    for(unsigned int i=0;i<len;i++)
    {
        if(data[i] == 0xAA)   ir_data_rx_state = data_rx_state_t::wait_for_HEAD;
        switch (ir_data_rx_state) {
            case data_rx_state_t::wait_for_HEAD:
                if(data[i] == 0xAA)   ir_data_rx_state = data_rx_state_t::wait_for_data;
                break;
            case data_rx_state_t::wait_for_data:
                ir_frame.data = data[i];
                ir_data_rx_state = data_rx_state_t::wait_for_uidL;
                break;
            case data_rx_state_t::wait_for_uidL:
                ir_frame.uid = 0;
                ir_frame.uid |= data[i];
                ir_data_rx_state = data_rx_state_t::wait_for_uidH;
                break;
            case data_rx_state_t::wait_for_uidH:
                ir_frame.uid |= static_cast<uint16_t>(data[i]) << 8;
                ir_data_rx_state = data_rx_state_t::wait_for_data_copy;
                break;
            case data_rx_state_t::wait_for_data_copy:
                if(data[i] != ir_frame.data) {
                    ir_data_rx_state = data_rx_state_t::wait_for_HEAD;
                }
                else {
                    ir_data_rx_state = data_rx_state_t::wait_for_END;
                }
                break;
            case data_rx_state_t::wait_for_END:
                if(data[i] == 0xBB) {
                    if(ir_frame.uid > biggest_rx_uid_) {
                        biggest_rx_uid_ = ir_frame.uid;
                    }
                    if(on_frame_func_ != nullptr) {
                        on_frame_func_(&ir_frame);
                    }
                    frame_complete = true;
                }                
                ir_data_rx_state = data_rx_state_t::wait_for_HEAD;
                break;
            default:
                ir_data_rx_state = data_rx_state_t::wait_for_HEAD;
                break;
        }
    }
    return frame_complete;
}

HAL_StatusTypeDef IR_SINGLE::trySend(uint16_t uid, uint8_t data) {
    if(uart_port_ == nullptr) {
        return HAL_ERROR;
    }

    uint32_t current_time = HAL_GetTick();
    if(current_time - last_tx_time_ < 175) {
        return HAL_BUSY;
    }

    uint8_t tx[6];
    tx[0] = 0xAA; // 数据帧头
    tx[1] = data;
    tx[2] = uid & 0xFF; // UID低8位
    tx[3] = (uid >> 8) & 0xFF; // UID高8位
    tx[4] = data;
    tx[5] = 0xBB; // 数据帧尾

    HAL_StatusTypeDef ret = uart_port_->writeDma(tx, sizeof(tx));
    if (ret == HAL_OK) {
        last_tx_time_ = HAL_GetTick();
    }
    return ret;
}