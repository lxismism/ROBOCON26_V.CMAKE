#include "omni_ir.hpp"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include <cstdint>

IrSingle::IrSingle(UartPort *uart_port, void (*on_frame_func)(IR_FRAME_t *))
    : on_frame_func_(on_frame_func), uart_port_(uart_port)
{
  last_tx_time_ = 0;
  biggest_rx_uid_ = 0;
}

bool IrSingle::processData(const uint8_t *data, size_t len) {
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
                ir_frame_.data = data[i];
                ir_data_rx_state = data_rx_state_t::wait_for_uidL;
                break;
            case data_rx_state_t::wait_for_uidL:
                ir_frame_.uid = 0;
                ir_frame_.uid |= data[i];
                ir_data_rx_state = data_rx_state_t::wait_for_uidH;
                break;
            case data_rx_state_t::wait_for_uidH:
                ir_frame_.uid |= static_cast<uint16_t>(data[i]) << 8;
                ir_data_rx_state = data_rx_state_t::wait_for_data_copy;
                break;
            case data_rx_state_t::wait_for_data_copy:
                if(data[i] != ir_frame_.data) {
                    ir_data_rx_state = data_rx_state_t::wait_for_HEAD;
                }
                else {
                    ir_data_rx_state = data_rx_state_t::wait_for_END;
                }
                break;
            case data_rx_state_t::wait_for_END:
                if(data[i] == 0xBB) {
                    if(ir_frame_.uid > biggest_rx_uid_) {
                        biggest_rx_uid_ = ir_frame_.uid;
                    }
                    if(on_frame_func_ != nullptr) {
                        on_frame_func_(&ir_frame_);
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

HAL_StatusTypeDef IrSingle::trySend(uint16_t uid, uint8_t data) {
    if(uart_port_ == nullptr) {
        return HAL_ERROR;
    }

    uint32_t current_time = HAL_GetTick();
    if(current_time - last_tx_time_ < kSendCd) {
        return HAL_BUSY;
    }

    uint8_t tx[6];
    tx[0] = 0xAA; // 数据帧头
    tx[1] = data;
    tx[2] = uid & 0xFF; // UID低8位
    tx[3] = (uid >> 8) & 0xFF; // UID高8位
    tx[4] = data;
    tx[5] = 0xBB; // 数据帧尾

    HAL_StatusTypeDef ret = uart_port_->write(tx, sizeof(tx), HAL_MAX_DELAY);
    if (ret == HAL_OK) {
        last_tx_time_ = HAL_GetTick();
    }
    return ret;
}

OmniIr::OmniIr(IrSingle *IrSingle[], int IrSingle_num, void (*on_update_func)(IR_FRAME_t *))
    : on_update_func_(on_update_func)
{
    IrSingle_num_ = (IrSingle_num > kMaxMap) ? kMaxMap : IrSingle_num;
    for(int i=0;i<IrSingle_num && i<kMaxMap;i++) {
        map_[i] = IrSingle[i];
    }
}

bool OmniIr::tryUpdate(IR_FRAME_t *frame) {
    if(frame == nullptr) {
        return false;
    }
    //防止重复的数据触发更新，防止自己发出的数据触发更新
    if(frame->uid <= biggest_used_uid_) {
        return false;
    }

    if(frame->uid > biggest_used_uid_) {
        biggest_used_uid_ = frame->uid;
        if(on_update_func_ != nullptr) {
            on_update_func_(frame);
        }
    }

    return true;
}
//依次发到每一个模块
bool OmniIr::sendData(uint16_t uid, uint8_t data) {
    
    biggest_used_uid_ = uid;
    for(int i=0;i<IrSingle_num_;i++) {
        uint32_t first_try_time;
        if(map_[i] != nullptr) {
            first_try_time = HAL_GetTick();
            while(map_[i]->trySend(uid, data) != HAL_OK && HAL_GetTick() - first_try_time < kTrySendTimeout) {
                osDelay(20);
            }
            osDelay(200);
        }
    }
    return true;
}