#include "rm_pocket.hpp"
#include <cstdint>

bool rmPocket::processByte(uint8_t byte) {
    switch (rx_state_) {
        case CRSF_rx_state_t::WAITING_FOR_SYNC:
            if (byte == CRSF_SYNC_BYTE) {
                rx_state_ = CRSF_rx_state_t::WAITING_FOR_LENGTH;
            }
            break;

        case CRSF_rx_state_t::WAITING_FOR_LENGTH:
            current_frame_.length = byte;
            if (current_frame_.length > CRSF_FRAME_LENGTH_MAX
                || current_frame_.length < 2) {
                resetState();
            } else {
                std::memset(current_frame_.payload, 0, sizeof(current_frame_.payload));
                rx_index_ = 0;
                rx_state_ = CRSF_rx_state_t::WAITING_FOR_TYPE;
            }
            break;

        case CRSF_rx_state_t::WAITING_FOR_TYPE:
            current_frame_.type = byte;
            rx_state_ = CRSF_rx_state_t::WAITING_FOR_PAYLOAD;
            break;

        case CRSF_rx_state_t::WAITING_FOR_PAYLOAD:
            current_frame_.payload[rx_index_++] = byte;
            if (rx_index_ >= current_frame_.length - 2) {
                rx_state_ = CRSF_rx_state_t::WAITING_FOR_CRC;
            }
            break;

        case CRSF_rx_state_t::WAITING_FOR_CRC: {
            bool is_uncode = false;
            if (true)   //CRC暂时跳过
            {
                is_uncode = onFrameComplete(current_frame_);
            } 
            resetState();
            return is_uncode;
        }

        default:
            resetState();
            break;
    }

    return 0;
}

bool rmPocket::onFrameComplete(const CRSF_broadcast_frame_t &frame) {
    
    auto fuzzyEqual = [](int16_t a, int16_t b) -> int8_t {
        return std::abs(a - b) <=35 ? 1 : 0;
    };
    //该映射表适用于16ch Rate/2模式
    auto cntTrans = [fuzzyEqual](uint16_t ch_value) -> int8_t{
        return (fuzzyEqual(ch_value, 186) * -10
                + fuzzyEqual(ch_value, 254) * -9
                + fuzzyEqual(ch_value, 330) * -8
                + fuzzyEqual(ch_value, 416) * -7
                + fuzzyEqual(ch_value, 500) * -6
                + fuzzyEqual(ch_value, 582) * -5
                + fuzzyEqual(ch_value, 664) * -4
                + fuzzyEqual(ch_value, 742) * -3
                + fuzzyEqual(ch_value, 828) * -2
                + fuzzyEqual(ch_value, 906) * -1
                + fuzzyEqual(ch_value, 992) * 0
                + fuzzyEqual(ch_value, 1076) * 1
                + fuzzyEqual(ch_value, 1156) * 2
                + fuzzyEqual(ch_value, 1240) * 3
                + fuzzyEqual(ch_value, 1320) * 4
                + fuzzyEqual(ch_value, 1400) * 5
                + fuzzyEqual(ch_value, 1482) * 6
                + fuzzyEqual(ch_value, 1568) * 7
                + fuzzyEqual(ch_value, 1654) * 8
                + fuzzyEqual(ch_value, 1730) * 9
                + fuzzyEqual(ch_value, 1796) * 10);
    };
    
    if(frame.type == CRSF_CP_TYPE)
    {       
        if(frame.length < 22) {
            resetState();
            return false;
        }
        //调试监看用
        static uint16_t ch[16] = {0};

        ch[0]  = ((frame.payload[0]      | frame.payload[1] << 8) & 0x07FF);
        ch[1]  = ((frame.payload[1] >> 3 | frame.payload[2] << 5) & 0x07FF);
        ch[2]  = ((frame.payload[2] >> 6 | frame.payload[3] << 2 | frame.payload[4] << 10) & 0x07FF);
        ch[3]  = ((frame.payload[4] >> 1 | frame.payload[5] << 7) & 0x07FF);
        ch[4]  = ((frame.payload[5] >> 4 | frame.payload[6] << 4) & 0x07FF);
        ch[5]  = ((frame.payload[6] >> 7 | frame.payload[7] << 1 | frame.payload[8] << 9) & 0x07FF);
        ch[6]  = ((frame.payload[8] >> 2 | frame.payload[9] << 6) & 0x07FF);
        ch[7]  = ((frame.payload[9] >> 5 | frame.payload[10] << 3) & 0x07FF);

        ch[8]  = ((frame.payload[11]      | frame.payload[12] << 8) & 0x07FF);
        ch[9]  = ((frame.payload[12] >> 3 | frame.payload[13] << 5) & 0x07FF);
        ch[10] = ((frame.payload[13] >> 6 | frame.payload[14] << 2 | frame.payload[15] << 10) & 0x07FF);
        ch[11] = ((frame.payload[15] >> 1 | frame.payload[16] << 7) & 0x07FF);
        ch[12] = ((frame.payload[16] >> 4 | frame.payload[17] << 4) & 0x07FF);
        ch[13] = ((frame.payload[17] >> 7 | frame.payload[18] << 1 | frame.payload[19] << 9) & 0x07FF);
        ch[14] = ((frame.payload[19] >> 2 | frame.payload[20] << 6) & 0x07FF);
        ch[15] = ((frame.payload[20] >> 5 | frame.payload[21] << 3) & 0x07FF);
        
        rc_state_.joyRHori = ch[0];
        rc_state_.joyRVert = ch[1];
        rc_state_.joyLVert = ch[2];
        rc_state_.joyLHori = ch[3];

        rc_state_.swE = ch[4] > 1500 ? RC_2_POS_SW_State_t::DOWN : RC_2_POS_SW_State_t::UP;

        rc_state_.x_cnt = cntTrans(ch[5]);
        rc_state_.y_cnt = cntTrans(ch[6]);
        rc_state_.cursor = cntTrans(ch[7]);

        rc_state_.swB = (fuzzyEqual(ch[8], 172) ? RC_3_POS_SW_State_t::UP : (fuzzyEqual(ch[8], 992) ? RC_3_POS_SW_State_t::MIDDLE : RC_3_POS_SW_State_t::DOWN));
        rc_state_.swC = (fuzzyEqual(ch[9], 172) ? RC_3_POS_SW_State_t::UP : (fuzzyEqual(ch[9], 992) ? RC_3_POS_SW_State_t::MIDDLE : RC_3_POS_SW_State_t::DOWN));

        rc_state_.swA = ch[10] > 1500 ? RC_2_POS_SW_State_t::DOWN : RC_2_POS_SW_State_t::UP;
        rc_state_.swD = ch[11] > 1500 ? RC_2_POS_SW_State_t::DOWN : RC_2_POS_SW_State_t::UP;
        rc_state_.pot = ch[12];

        return true;
    }
    return false;
}
