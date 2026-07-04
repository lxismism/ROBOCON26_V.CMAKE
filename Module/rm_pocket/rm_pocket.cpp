#include "rm_pocket.hpp"
#include <cstdint>

static const uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

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
            current_frame_.crc = byte;

            uint8_t crc = crsfCrc(current_frame_);
            
            bool is_uncode = false;
            if (current_frame_.crc == crc)
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
    
    auto fuzzyEqual35 = [](int16_t a, int16_t b) -> int8_t {
        return std::abs(a - b) <=35 ? 1 : 0;
    };

    auto fuzzyEqual100 = [](int16_t a, int16_t b) -> int8_t {
        return std::abs(a - b) <=100 ? 1 : 0;
    };

    //该映射表适用于16ch Rate/2模式
    auto cntTrans = [fuzzyEqual35](uint16_t ch_value) -> int8_t{
        return (fuzzyEqual35(ch_value, 186) * -10
                + fuzzyEqual35(ch_value, 254) * -9
                + fuzzyEqual35(ch_value, 330) * -8
                + fuzzyEqual35(ch_value, 416) * -7
                + fuzzyEqual35(ch_value, 500) * -6
                + fuzzyEqual35(ch_value, 582) * -5
                + fuzzyEqual35(ch_value, 664) * -4
                + fuzzyEqual35(ch_value, 742) * -3
                + fuzzyEqual35(ch_value, 828) * -2
                + fuzzyEqual35(ch_value, 906) * -1
                + fuzzyEqual35(ch_value, 992) * 0
                + fuzzyEqual35(ch_value, 1076) * 1
                + fuzzyEqual35(ch_value, 1156) * 2
                + fuzzyEqual35(ch_value, 1240) * 3
                + fuzzyEqual35(ch_value, 1320) * 4
                + fuzzyEqual35(ch_value, 1400) * 5
                + fuzzyEqual35(ch_value, 1482) * 6
                + fuzzyEqual35(ch_value, 1568) * 7
                + fuzzyEqual35(ch_value, 1654) * 8
                + fuzzyEqual35(ch_value, 1730) * 9
                + fuzzyEqual35(ch_value, 1796) * 10);
    };

    auto trimTrans = [fuzzyEqual100](uint16_t ch_value) -> RC_Trim_State_t {
        if (fuzzyEqual100(ch_value, 1760)) {
            return RC_Trim_State_t::UP;
        } else if (fuzzyEqual100(ch_value, 1401)) {
            return RC_Trim_State_t::DOWN;
        } else if (fuzzyEqual100(ch_value, 222)) {
            return RC_Trim_State_t::LEFT;
        } else if (fuzzyEqual100(ch_value, 583)) {
            return RC_Trim_State_t::RIGHT;
        } else {
            return RC_Trim_State_t::MIDDLE; //默认返回中立
        }
    };
    
    if(frame.type == CRSF_CP_TYPE)
    {       
        if(frame.length != 24) {
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
        rc_state_.cursor = (ch[7] > 1500 ? 2 : (ch[7] < 500 ? 0 : 1));

        rc_state_.swB = (fuzzyEqual35(ch[8], 172) ? RC_3_POS_SW_State_t::UP : (fuzzyEqual35(ch[8], 992) ? RC_3_POS_SW_State_t::MIDDLE : RC_3_POS_SW_State_t::DOWN));
        rc_state_.swC = (fuzzyEqual35(ch[9], 172) ? RC_3_POS_SW_State_t::UP : (fuzzyEqual35(ch[9], 992) ? RC_3_POS_SW_State_t::MIDDLE : RC_3_POS_SW_State_t::DOWN));

        rc_state_.swA = ch[10] > 1500 ? RC_2_POS_SW_State_t::DOWN : RC_2_POS_SW_State_t::UP;
        rc_state_.swD = ch[11] > 1500 ? RC_2_POS_SW_State_t::DOWN : RC_2_POS_SW_State_t::UP;
        rc_state_.pot = ch[12];

        rc_state_.trimLeft = trimTrans(ch[13]);
        rc_state_.trimRight = trimTrans(ch[14]);

        return true;
    }
    return false;
}

uint8_t rmPocket::crsfCrc(const CRSF_broadcast_frame_t &frame) {
    uint8_t crc = 0;
    crc = crc8tab[crc ^ frame.type];
    for (uint8_t i = 0; i < frame.length - 2; i++)
        crc = crc8tab[crc ^ frame.payload[i]];
    return crc;
}

uint8_t rmPocket::packFrame(uint8_t *tx_buf, const CRSF_broadcast_frame_t &frame) {
    if(tx_buf == nullptr) {
        return 0;
    }
    tx_buf[0] = CRSF_SYNC_BYTE;
    tx_buf[1] = frame.length;
    tx_buf[2] = frame.type;
    std::memcpy(&tx_buf[3], frame.payload, frame.length - 2);
    tx_buf[frame.length + 1] = crsfCrc(frame);
    return frame.length + 2; //返回总长度
}
