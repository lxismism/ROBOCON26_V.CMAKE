#pragma once

#include "UartPort.hpp"
#include "main.h"
#include <cstdint>

constexpr int kMaxMap = 5;
constexpr int kMaxDataLength = 32;
constexpr uint32_t kSendCd = 175;
constexpr uint32_t kTrySendTimeout = 300;

constexpr uint8_t CMD_RELEASE_CLAW  = 0x0A;
constexpr uint8_t CMD_PICK_NEW      = 0x1A;
constexpr uint8_t CMD_ENTER_MF      = 0x1B;

struct IR_FRAME_t{
    uint16_t uid;
    uint8_t data;
};

class IrSingle {

    public:
        
        IrSingle(UartPort *uart_port, void (*on_frame_func)(IR_FRAME_t *));
        
        HAL_StatusTypeDef trySend(uint16_t uid, uint8_t data);
        //解析到至少一个完整帧时返回true，并通过回调函数传出解析到的帧数据；否则返回false
        bool processData(const uint8_t *data, size_t len);

        uint32_t getLastTxTime() const { return last_tx_time_; }
        uint16_t getLastRxUid() const { return biggest_rx_uid_; }

    private:
        
        enum class data_rx_state_t {
            wait_for_HEAD,
            wait_for_data,
            wait_for_uidL,
            wait_for_uidH,
            wait_for_data_copy,
            wait_for_END
        };

        void (*on_frame_func_)(IR_FRAME_t*);

        UartPort *uart_port_;

        uint32_t last_tx_time_;
        uint16_t biggest_rx_uid_;

        data_rx_state_t ir_data_rx_state{data_rx_state_t::wait_for_HEAD};
        IR_FRAME_t ir_frame_{};

};

class OmniIr {

    public:

        OmniIr(IrSingle *IrSingle[], int IrSingle_num, void (*on_update_func)(IR_FRAME_t *));

        bool tryUpdate(IR_FRAME_t *frame);
        bool sendData(uint16_t uid, uint8_t data);

    private:

        uint16_t biggest_used_uid_{0};

        void (*on_update_func_)(IR_FRAME_t*);

        IrSingle *map_[kMaxMap];
        int IrSingle_num_;
};
