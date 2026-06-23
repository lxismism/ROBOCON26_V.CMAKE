#pragma once

#include "UartPort.hpp"
#include "main.h"
#include "stm32h7xx_hal_def.h"
#include <cstdint>
#include <cstring>

constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;

constexpr uint8_t CRSF_FRAME_LENGTH_MAX = 0x62;

constexpr uint8_t CRSF_CP_TYPE = 0x16;

struct CRSF_broadcast_frame_t {
    uint8_t length;
    uint8_t type;
    uint8_t payload[CRSF_FRAME_LENGTH_MAX];
    uint8_t crc;
};

//按钮
enum class RC_BTN_State_t {
    RELESED,
    PRESSED,
};


//两段开关
enum class RC_2_POS_SW_State_t {
    UP,
    DOWN
};


//三段开关
enum class RC_3_POS_SW_State_t {
    UP, //后端按下，前端翘起
    MIDDLE,
    DOWN
};

enum class RC_Trim_State_t {
    UP,
    DOWN,
    LEFT,
    RIGHT,
    MIDDLE
};
class rmPocket {

    enum class CRSF_rx_state_t {
        WAITING_FOR_SYNC,
        WAITING_FOR_LENGTH,
        WAITING_FOR_TYPE,
        WAITING_FOR_PAYLOAD,
        WAITING_FOR_CRC
    };

    public:

    struct RC_state_t {

        //摇杆，值域172-1810 建议映射值：min200 max1780 mid~=985 死区+-100
        uint16_t joyLHori;  //左小
        uint16_t joyLVert;  //下小
        uint16_t joyRHori;  //左小
        uint16_t joyRVert;  //下小

        RC_2_POS_SW_State_t swA;        //阴刻有SA的两端开关
        RC_2_POS_SW_State_t swA_last;
        RC_3_POS_SW_State_t swB;        //阴刻有SB的三段开关
        RC_3_POS_SW_State_t swB_last;
        RC_3_POS_SW_State_t swC;        //阴刻有SC的三段开关
        RC_3_POS_SW_State_t swC_last;
        RC_2_POS_SW_State_t swD;        //阴刻有SD的两段开关
        RC_2_POS_SW_State_t swD_last;
        RC_2_POS_SW_State_t swE;        //阴刻有SE的按钮
        RC_2_POS_SW_State_t swE_last;

        RC_Trim_State_t trimLeft;        //左微调按钮
        RC_Trim_State_t trimLeft_last;
        RC_Trim_State_t trimRight;       //右微调按钮
        RC_Trim_State_t trimRight_last;

        //电位器 往左推小，值域172-1810 实际可能取不到端点
        uint16_t pot;                   //阴刻有S1的拨盘

        //左微调按钮控制的光标，-9~+9
        int8_t x_cnt;
        int8_t y_cnt;

        //右微调按钮控制的光标，-9~+9
        int8_t cursor;

    };

    void init(void) {
        std::memset(&rc_state_, 0, sizeof(rc_state_));
        resetState();
    };

    bool processByte(uint8_t byte);

    const RC_state_t &getRCState() const {
        return rc_state_;
    }

    private:
    CRSF_rx_state_t rx_state_;
    uint8_t rx_buffer_[CRSF_FRAME_LENGTH_MAX]; //数据内容缓冲
    uint8_t rx_index_;
    RC_state_t rc_state_;
    CRSF_broadcast_frame_t current_frame_;

    void resetState() {
        rx_state_ = CRSF_rx_state_t::WAITING_FOR_SYNC;
        rx_index_ = 0;
        std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
        std::memset(&current_frame_, 0, sizeof(current_frame_));
    };

    bool onFrameComplete(const CRSF_broadcast_frame_t &frame);

};