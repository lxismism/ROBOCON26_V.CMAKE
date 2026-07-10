local MATRIX_SIZE = 16
local CELL_SIZE = 4
local DOT_SIZE = 3
local SCREEN_W = LCD_W or 128
local SCREEN_H = LCD_H or 64
local ORIGIN_X = math.floor((SCREEN_W - MATRIX_SIZE * CELL_SIZE) / 2)
local ORIGIN_Y = math.floor((SCREEN_H - MATRIX_SIZE * CELL_SIZE) / 2)
local MF_MAP_Y_OFFSET = -2
local MF_PIXEL_Y_OFFSET = 3
local S1_VALUE_Y = 22
local S1_IND_X = SCREEN_W - 17
local S1_IND_Y = 32
local IR_LABEL_Y = 42
local IR_VALUE_Y = 50
local S1_MIDDLE_LIMIT = 307
local MOTOR_MARKER_SIZE = 5
local PATH_OUTER_OFFSET = 2

local motor_points = {
    { px = 0,                 py = 0,                  byte = 0, mask = 128 }, -- chassis_motor1
    { px = SCREEN_W - 5,      py = 0,                  byte = 0, mask = 64  }, -- chassis_motor2
    { px = SCREEN_W - 5,      py = SCREEN_H - 5,       byte = 0, mask = 32  }, -- chassis_motor3
    { px = 0,                 py = SCREEN_H - 5,       byte = 0, mask = 16  }, -- chassis_motor4
    { px = 6,                 py = 0,                  byte = 0, mask = 8   }, -- picker_yaw_motor
    { px = 6,                 py = 6,                  byte = 0, mask = 4   }, -- picker_extend_motor
    { px = SCREEN_W - 11,     py = 6,                  byte = 0, mask = 2   }, -- weapon_extend_motor
    { px = 0,                 py = SCREEN_H - 11,      byte = 0, mask = 1   }, -- lift_left_motor
    { px = SCREEN_W - 5,      py = SCREEN_H - 11,      byte = 1, mask = 128 }, -- lift_right_motor
    { px = 0,                 py = 6,                  byte = 1, mask = 64  }, -- picker_lift_motor
    { px = SCREEN_W - 5,      py = 6,                  byte = 1, mask = 32  }, -- weapon_lift_motor
}

local function has_bit(value, mask)
    return value % (mask * 2) >= mask
end

local function read_number(name)
    local value = getValue(name)
    if type(value) == "number" then
        return math.floor(value + 0.5)
    end
    return 0
end

local function read_raw_number(name)
    local value = getValue(name)
    if type(value) == "number" then
        return value
    end
    return nil
end

local function read_telem_payload(name)
    local value = read_raw_number(name)
    if not value then
        return 0
    end
    local scaled = value * 10
    if scaled >= 0 then
        return math.floor(scaled + 0.5)
    end
    return math.ceil(scaled - 0.5)
end

local function draw_text_right(x, y, text)
    lcd.drawText(x - string.len(text) * 5, y, text, SMLSIZE)
end

local function decode_point(value)
    if value < 100 then
        return nil
    end

    local data = value - 100
    local x = math.floor(data / 10)
    local y = data - x * 10

    if x >= 0 and x <= 5 and y >= 0 and y <= 4 then
        return x, y
    end

    return nil
end

local function draw_pixel(x, y)
    if x < 0 or x >= MATRIX_SIZE or y < 0 or y >= MATRIX_SIZE then
        return
    end

    lcd.drawFilledRectangle(
        ORIGIN_X + x * CELL_SIZE,
        ORIGIN_Y + y * CELL_SIZE,
        DOT_SIZE,
        DOT_SIZE
    )
end

local function draw_mf_tile(x, y)
    local px = ORIGIN_X + x * CELL_SIZE
    local py = ORIGIN_Y + y * CELL_SIZE + MF_PIXEL_Y_OFFSET
    local span = CELL_SIZE * 3 - 1
    local inner = math.floor(span / 2)
    local inner_offset = math.floor((span - inner) / 2)

    lcd.drawRectangle(px, py, span, span)
    lcd.drawRectangle(px + inner_offset, py + inner_offset, inner, inner)
end

local function draw_mf_map()
    local y = 4 + MF_MAP_Y_OFFSET
    for _ = 1, 3 do
        local x = 2
        for _ = 1, 4 do
            draw_mf_tile(x, y)
            x = x + 3
        end
        y = y + 3
    end
end

local function map_mf_pos(mf_x, mf_y)
    local x = 15 - 3 * mf_x
    local y = 14 - 3 * mf_y

    if mf_x == 0 then x = 14 end
    if mf_x == 5 then x = 1 end
    if mf_y == 0 then y = 13 end
    if mf_y == 4 then y = 3 end

    y = y + MF_MAP_Y_OFFSET

    if x < 0 then x = 0 end
    if x > 15 then x = 15 end
    if y < 0 then y = 0 end
    if y > 15 then y = 15 end

    return x, y
end

local function draw_matrix_marker(x, y, size, filled, offset_x, offset_y)
    local px = ORIGIN_X + x * CELL_SIZE + math.floor((DOT_SIZE - size) / 2) + (offset_x or 0)
    local py = ORIGIN_Y + y * CELL_SIZE + math.floor((DOT_SIZE - size) / 2) + MF_PIXEL_Y_OFFSET + (offset_y or 0)

    if filled then
        lcd.drawFilledRectangle(px, py, size, size)
    else
        lcd.drawRectangle(px, py, size, size)
    end
end

local function draw_mf_cursor(mf_x, mf_y)
    local x, y = map_mf_pos(mf_x, mf_y)
    local offset_x = 0
    local offset_y = 0

    if mf_x == 0 then
        offset_x = PATH_OUTER_OFFSET
    elseif mf_x == 5 then
        offset_x = -PATH_OUTER_OFFSET
    end

    if mf_y == 0 then
        offset_y = PATH_OUTER_OFFSET
    elseif mf_y == 4 then
        offset_y = -PATH_OUTER_OFFSET
    end

    draw_matrix_marker(x, y, 7, false, offset_x, offset_y)
end

local function draw_mf_plan(mf_x, mf_y)
    local x, y = map_mf_pos(mf_x, mf_y)
    local offset_x = 0
    local offset_y = 0

    if mf_x == 0 then
        offset_x = PATH_OUTER_OFFSET
    elseif mf_x == 5 then
        offset_x = -PATH_OUTER_OFFSET
    end

    if mf_y == 0 then
        offset_y = PATH_OUTER_OFFSET
    elseif mf_y == 4 then
        offset_y = -PATH_OUTER_OFFSET
    end

    draw_matrix_marker(x, y, 3, true, offset_x, offset_y)
end

local function draw_telem_point(name, is_cursor)
    local x, y = decode_point(read_telem_payload(name))
    if x and y then
        if is_cursor then
            draw_mf_cursor(x, y)
        else
            draw_mf_plan(x, y)
        end
    end
end

local function draw_mf_plans()
    local seen = {}

    for i = 17, 31 do
        local x, y = decode_point(read_telem_payload("telem" .. i))
        if x and y then
            local key = x * 10 + y
            if not seen[key] then
                seen[key] = true
                draw_mf_plan(x, y)
            end
        end
    end
end

local function has_return_signal()
    local value = read_raw_number("telem16")
    if not value then
        return false
    end

    local x, y = decode_point(read_telem_payload("telem16"))
    return x ~= nil and y ~= nil
end

local function draw_motor_status()
    local packed = read_telem_payload("telem32")
    if packed < 0 then
        packed = packed + 65536
    end

    local high = math.floor(packed / 256) % 256
    local low = packed % 256

    local function draw_motor_marker(px, py, is_not_online)
        if is_not_online then
            lcd.drawFilledRectangle(px,     py,     1, 1)
            lcd.drawFilledRectangle(px + 4, py,     1, 1)
            lcd.drawFilledRectangle(px + 1, py + 1, 1, 1)
            lcd.drawFilledRectangle(px + 3, py + 1, 1, 1)
            lcd.drawFilledRectangle(px + 2, py + 2, 1, 1)
            lcd.drawFilledRectangle(px + 1, py + 3, 1, 1)
            lcd.drawFilledRectangle(px + 3, py + 3, 1, 1)
            lcd.drawFilledRectangle(px,     py + 4, 1, 1)
            lcd.drawFilledRectangle(px + 4, py + 4, 1, 1)
        else
            lcd.drawRectangle(px, py, MOTOR_MARKER_SIZE, MOTOR_MARKER_SIZE)
        end
    end

    for i = 1, #motor_points do
        local motor = motor_points[i]
        local state = motor.byte == 0 and high or low
        local is_not_online = has_bit(state, motor.mask)
        draw_motor_marker(motor.px, motor.py, is_not_online)
    end
end

local function draw_return_signal()
    if has_return_signal() then
        lcd.drawText(41, SCREEN_H - 8, "connected", SMLSIZE)
    else
        lcd.drawText(32, SCREEN_H - 8, "disconnected", SMLSIZE)
    end
end

local function draw_arena_ir_count()
    draw_text_right(SCREEN_W - 1, IR_LABEL_Y, "IRCNT")
    draw_text_right(SCREEN_W - 8, IR_VALUE_Y, string.format("%d", read_telem_payload("telem33")))
end

local function read_s1()
    return read_raw_number("s1") or read_raw_number("S1") or 0
end

local function draw_s1_state()
    local raw_s1 = read_s1()
    local selected = 2

    if raw_s1 < -S1_MIDDLE_LIMIT then
        selected = 1
    elseif raw_s1 > S1_MIDDLE_LIMIT then
        selected = 3
    end

    draw_text_right(SCREEN_W - 1, S1_VALUE_Y, string.format("%d", math.floor(raw_s1 + 0.5)))

    for i = 1, 3 do
        local x = S1_IND_X + (i - 1) * 6
        if selected == i then
            lcd.drawFilledRectangle(x, S1_IND_Y, 5, 5)
        else
            lcd.drawRectangle(x, S1_IND_Y, 5, 5)
        end
    end
end

local function my_init()
end

local function my_background()
end

local function my_run(event)
    lcd.clear()

    lcd.drawText(0, math.floor(SCREEN_H / 2) - 8, "L", DBLSIZE)
    draw_mf_map()
    draw_mf_plans()
    draw_telem_point("telem16", true)

    draw_motor_status()
    draw_arena_ir_count()
    draw_return_signal()
    draw_s1_state()

    return 0
end

return { run = my_run, background = my_background, init = my_init }
