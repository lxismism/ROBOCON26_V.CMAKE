local MATRIX_SIZE = 16
local CELL_SIZE = 4
local DOT_SIZE = 3
local SCREEN_W = LCD_W or 128
local SCREEN_H = LCD_H or 64
local ORIGIN_X = math.floor((SCREEN_W - MATRIX_SIZE * CELL_SIZE) / 2)
local ORIGIN_Y = math.floor((SCREEN_H - MATRIX_SIZE * CELL_SIZE) / 2)
local TELEMETRY_OK_X = 10
local TELEMETRY_OK_Y = 13
local S1_X = ORIGIN_X + MATRIX_SIZE * CELL_SIZE + 8
local S1_Y = 21
local S1_MIDDLE_LIMIT = 307

local motor_points = {
    { x = 0,  y = 0,  byte = 0, mask = 128 }, -- chassis_motor1
    { x = 15, y = 0,  byte = 0, mask = 64  }, -- chassis_motor2
    { x = 15, y = 15, byte = 0, mask = 32  }, -- chassis_motor3
    { x = 0,  y = 15, byte = 0, mask = 16  }, -- chassis_motor4
    { x = 2,  y = 0,  byte = 0, mask = 8   }, -- picker_yaw_motor
    { x = 2,  y = 2,  byte = 0, mask = 4   }, -- picker_extend_motor
    { x = 13, y = 2,  byte = 0, mask = 2   }, -- weapon_extend_motor
    { x = 0,  y = 12, byte = 0, mask = 1   }, -- lift_left_motor
    { x = 15, y = 12, byte = 1, mask = 128 }, -- lift_right_motor
    { x = 0,  y = 2,  byte = 1, mask = 64  }, -- picker_lift_motor
    { x = 15, y = 2,  byte = 1, mask = 32  }, -- weapon_lift_motor
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
    local py = ORIGIN_Y + y * CELL_SIZE
    local span = CELL_SIZE * 3 - 1
    local inner = math.floor(span / 2)
    local inner_offset = math.floor((span - inner) / 2)

    lcd.drawRectangle(px, py, span, span)
    lcd.drawRectangle(px + inner_offset, py + inner_offset, inner, inner)
end

local function draw_mf_map()
    local y = 4
    for _ = 1, 3 do
        local x = 2
        for _ = 1, 4 do
            draw_mf_tile(x, y)
            x = x + 3
        end
        y = y + 3
    end
end

local function draw_mf_pos(mf_x, mf_y)
    local x = 15 - 3 * mf_x
    local y = 14 - 3 * mf_y

    if mf_x == 0 then x = 14 end
    if mf_x == 5 then x = 1 end
    if mf_y == 0 then y = 13 end
    if mf_y == 4 then y = 3 end

    draw_pixel(x, y)
end

local function draw_telem_point(name)
    local x, y = decode_point(read_number(name))
    if x and y then
        draw_mf_pos(x, y)
    end
end

local function has_return_signal()
    local value = read_raw_number("telem16")
    if not value then
        return false
    end

    local x, y = decode_point(math.floor(value + 0.5))
    return x ~= nil and y ~= nil
end

local function draw_motor_status()
    local packed = read_number("telem32")
    if packed < 0 then
        packed = packed + 65536
    end

    local high = math.floor(packed / 256) % 256
    local low = packed % 256

    for i = 1, #motor_points do
        local motor = motor_points[i]
        local state = motor.byte == 0 and high or low
        local is_not_online = has_bit(state, motor.mask)
        if is_not_online then
            draw_pixel(motor.x, motor.y)
        end
    end
end

local function draw_return_signal()
    lcd.drawText(1, 1, "TEL", SMLSIZE)

    if has_return_signal() then
        lcd.drawFilledRectangle(TELEMETRY_OK_X, TELEMETRY_OK_Y, 8, 8)
        lcd.drawText(2, 25, "OK", SMLSIZE)
    else
        lcd.drawRectangle(TELEMETRY_OK_X, TELEMETRY_OK_Y, 8, 8)
        lcd.drawText(2, 25, "--", SMLSIZE)
    end
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

    lcd.drawText(S1_X, 1, "S1", SMLSIZE)
    lcd.drawText(S1_X - 8, 9, string.format("%d", math.floor(raw_s1 + 0.5)), SMLSIZE)

    local labels = { "L", "M", "R" }
    for i = 1, 3 do
        local y = S1_Y + (i - 1) * 15
        if selected == i then
            lcd.drawFilledRectangle(S1_X, y, 8, 8)
        else
            lcd.drawRectangle(S1_X, y, 8, 8)
        end
        lcd.drawText(S1_X + 11, y - 1, labels[i], SMLSIZE)
    end
end

local function my_init()
end

local function my_background()
end

local function my_run(event)
    lcd.clear()

    draw_mf_map()
    draw_telem_point("telem16")

    for i = 17, 31 do
        draw_telem_point("telem" .. i)
    end

    draw_motor_status()
    draw_return_signal()
    draw_s1_state()

    return 0
end

return { run = my_run, background = my_background, init = my_init }
