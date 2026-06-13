#pragma once

#include "UartPort.hpp"
#include "main.h"
#include <cstdint>

constexpr int kMaxMap = 5;
constexpr int kMaxDataLength = 32;

struct IR_FRAME_t{
    uint16_t uid;
    uint8_t data;
};

class IR_SINGLE {

public:
    
    IR_SINGLE(UartPort *uart_port, void (*on_frame_func)(IR_FRAME_t *));
    
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
    IR_FRAME_t ir_frame{};

};

class OMNI_IR {

enum class ir_tx_state_t {
    wait_for_data,
    wait_for_Send_1,
    wait_for_Send_2,
    wait_for_Send_3,
    wait_for_Send_4
};

public:

    OMNI_IR(IR_SINGLE *ir_single, int ir_single_num);

    bool tryUpdate(IR_FRAME_t *frame);
    void sendData(uint16_t uid, uint8_t data);

private:

    uint16_t biggest_rx_uid;
    
    IR_SINGLE *map_[kMaxMap];
    int ir_single_num;
};
