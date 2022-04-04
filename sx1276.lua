-- Ehanced by Jakub Mazurek @jayu in 2021

-- The MIT License (MIT)
--
-- Copyright (c) 2016 Jaap Braam
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in all
-- copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
-- SOFTWARE.
--
-- Author: Jaap Braam

REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_OCP = 0x0b
REG_LNA = 0x0c
REG_FIFO_ADDR_PTR = 0x0d
REG_FIFO_TX_BASE_ADDR = 0x0e
REG_FIFO_RX_BASE_ADDR = 0x0f
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_PKT_SNR_VALUE = 0x19
REG_PKT_RSSI_VALUE = 0x1a
REG_RSSI_VALUE = 0x1b
REG_MODEM_CONFIG_1 = 0x1d
REG_MODEM_CONFIG_2 = 0x1e
REG_PREAMBLE_MSB = 0x20
REG_PREAMBLE_LSB = 0x21
REG_PAYLOAD_LENGTH = 0x22
REG_MODEM_CONFIG_3 = 0x26
REG_FREQ_ERROR_MSB = 0x28
REG_FREQ_ERROR_MID = 0x29
REG_FREQ_ERROR_LSB = 0x2a
REG_RSSI_WIDEBAND = 0x2c
REG_DETECTION_OPTIMIZE = 0x31
REG_INVERTIQ = 0x33
REG_DETECTION_THRESHOLD = 0x37
REG_SYNC_WORD = 0x39
REG_INVERTIQ2 = 0x3b
REG_DIO_MAPPING_1 = 0x40
REG_VERSION = 0x42
REG_PA_DAC = 0x4d

-- modes
MODE_LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03
MODE_RX_CONTINUOUS = 0x05
MODE_RX_SINGLE = 0x06

-- PA config
PA_BOOST = 0x80

-- IRQ masks
IRQ_TX_DONE_MASK = 0x08
IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20
IRQ_RX_DONE_MASK = 0x40

RF_MID_BAND_THRESHOLD = 525E6
RSSI_OFFSET_HF_PORT = 157
RSSI_OFFSET_LF_PORT = 164

MAX_PKT_LENGTH = 255

local now = tmr.now
local gpiowrite = gpio.write
local spisend = spi.send
local spirecv = spi.recv
local bor = bit.bor
local band = bit.band
local bnot = bit.bnot
local delay = tmr.delay
local byte = string.byte
local rtcepoch2cal = rtctime.epoch2cal
local rtcget = rtctime.get

local function padBase64(s)
    local p = 4 - (#s % 4)
    return s .. string.rep("=", p % 4)
end

local MC1 = {
    BW125 = 0x70,
    BW250 = 0x80,
    BW500 = 0x90,
    BW150 = 0x00
}
MC1["4/5"] = 0x02
MC1["4/6"] = 0x04
MC1["4/7"] = 0x06
MC1["4/8"] = 0x08

local MC2 = {
    FSK = 0x00,
    SF6 = 0x60,
    SF7 = 0x70,
    SF8 = 0x80,
    SF9 = 0x90,
    SF10 = 0xA0,
    SF11 = 0xB0,
    SF12 = 0xC0
}

local function getName(table, value, mask)
    for k, v in pairs(table) do
        if bit.band(value, mask) == v then
            return k
        end
    end
    return "?"
end

local M = {
    rxnb = 0,
    rxok = 0,
    rxto = 0,
    txnb = 0
}

local _nss = 0
local _dio0 = 1
local _dio1 = 2
local _freq = 868100000
local _sf = MC2.SF7
local _bw = MC1.BW125
local _cr = "4/5"
local _scanner = function()
    print("no scanner active")
end

local function read(addr)
    gpiowrite(_nss, 0)
    spisend(1, addr)
    local b = spirecv(1, 1)
    gpiowrite(_nss, 1)
    return byte(b)
end

local function readBuffer(addr, len)
    gpiowrite(_nss, 0)
    spisend(1, addr)
    local buf = spirecv(1, len)
    gpiowrite(_nss, 1)
    return buf
end

local function write(addr, value)
    gpiowrite(_nss, 0)
    spisend(1, addr + 0x80, value)
    gpiowrite(_nss, 1)
end

local function idle()
    write(REG_OP_MODE, bor(MODE_LONG_RANGE_MODE, MODE_STDBY))
end

local function pktData()
    -- local FIFO_RX_CURRENT_ADDR=0x10
    -- local RX_NB_BYTES=0x13
    -- local FIFO_ADDR_PTR=0x0D
    -- local FIFO=0x00

    local curr = read(0x10)
    local count = read(0x13)
    write(0x0D, curr)
    return readBuffer(0x00, count) or ""
end

local tmst = 0
local function rxDone()
    --  local IRQ_FLAGS=0x12
    --  local RxDone=0x40
    -- local tmst=now()
    local pkt = {}
    pkt.tmst = tmst
    -- clear rxDone
    write(0x12, 0x40)
    -- message counter
    M.rxnb = M.rxnb + 1

    -- CRC
    local irqflags = read(0x12)
    if band(irqflags, 0x20) == 0x20 then
        write(0x12, 0x20)
        pkt.stat = -1
    else
        local rhc = read(0x1C)
        if band(rhc, 0x40) == 0x40 then
            pkt.stat = 1
        else
            pkt.stat = 0
        end
    end

    if pkt.stat ~= -1 then
        local hopch = read(0x1C)
        pkt.chan = band(hopch, 0x1F) -- pktChan()

        pkt.rfch = 1
        pkt.modu = "LORA"

        local freq = (125000 * read(0x08) / 2 ^ 11) + (125000 * read(0x07) / 2 ^ 3) + (125000 * read(0x06) * 2 ^ 5) -- pktFreq() -- in Hz

        pkt.freq = string.format("%0d.%03d", freq / 1000000, ((freq + 500) / 1000) % 1000)

        local rssi = read(0x1A)
        pkt.rssi = -157 + rssi
        -- pktRssi()

        local snr = read(0x19)
        if snr > 127 then
            snr = -(band(bnot(snr), 0xFF) + 1)
        end
        pkt.lsnr = snr / 4 -- pktLsnr()

        local mc1 = read(0x1D)
        local mc2 = read(0x1E)
        pkt.datr = getName(MC2, mc2, 0xF0) .. getName(MC1, mc1, 0xF0) -- pktDatr()

        pkt.codr = getName(MC1, mc1, 0x0E) -- pktCodr()

        pkt.data = pktData()

        pkt.size = #pkt.data
        -- message ok counter
        M.rxok = M.rxok + 1
        -- callback
        M.rxpk(pkt)
    end
end

local function setFreq(freqHz)
    --  local FRF_MSB=0x06
    --  local FRF_MID=0x07
    --  local FRF_LSB=0x08

    -- keep resolution for integer version
    -- frf = (freqHz*2^19)/32000000
    -- frf = (freqHz*2^14)/1000000
    -- frf = (freqHz/1000*2^14)/1000
    -- frf = (freqHz/1000*2^11)/125
    local frf = (freqHz / 1000 * 2 ^ 11) / 125
    local frfMsb = frf / 2 ^ 16
    local frfMid = frf / 2 ^ 8 % 256
    local frfLsb = frf % 256
    write(0x06, frfMsb)
    write(0x07, frfMid)
    write(0x08, frfLsb)
    -- print(string.format("%0d.%06d Mhz %02X %02X %02X ",freqHz/1000000,freqHz%1000000,frfMsb,frfMid,frfLsb))
end

local function old_setPower(power)
    local pac
    if power > 17 then
        pac = 0x8F -- 17dbm
    elseif power < -3 then
        pac = 0x20 -- -3dbm
    elseif power <= 12 then
        pac = 0x20 + power + 3 -- -3dbm .. 12dbm
    else
        pac = 0x80 + power - 2 -- 13dbm .. 16dbm
    end
    write(REG_PA_CONFIG, pac)
end

local function setOCP(mA)
    local ocpTrim = 27

    if (mA <= 120) then
        ocpTrim = (mA - 45) / 5
    elseif (mA <= 240) then
        ocpTrim = (mA + 30) / 10
    end

    write(REG_OCP, bor(0x20, band(0x1F, ocpTrim)))
end

local function setPower(level)
    --  PA BOOST
    if (level > 17) then
        if (level > 20) then
            level = 20
        end

        --  subtract 3 from level, so 18 - 20 maps to 15 - 17
        level = level - 3

        --  High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
        write(REG_PA_DAC, 0x87)
        setOCP(140)
    else
        if (level < 2) then
            level = 2
        end
        -- Default value PA_HF/LF or +17dBm
        write(REG_PA_DAC, 0x84)
        setOCP(100)
    end

    write(REG_PA_CONFIG, bor(PA_BOOST, (level - 2)))
end

local function setRate(sf, bw, cr, crc, iiq, power)
    --  local SF10=0xA0
    --  local SF11=0xB0
    --  local SF12=0xC0
    --  local PA_CONFIG=0x09
    --  local MODEM_CONFIG1=0x1D
    --  local MODEM_CONFIG2=0x1E
    --  local MODEM_CONFIG3=0x26
    --  local SYMB_TIMEOUT_LSB=0x1F
    --  local INVERT_IQ=0x33
    local mc1 = bor(bw, cr)
    local mc2 = bor(sf, crc)
    -- local mc3=0x04
    local mc3 = 0x00 -- no AGC
    if (sf == 0xB0 or sf == 0xC0) then
        mc3 = 0x08
    end -- MC2.SF11=0xB0, MC2.SF12=0xC0
    local stl = 0x08
    if (sf == 0xA0 or sf == 0xB0 or sf == 0xC0) then
        stl = 0x05
    end
    setPower(power)
    write(0x1D, mc1)
    write(0x1E, mc2)
    write(0x26, mc3)
    write(0x1F, stl)
    write(0x33, iiq)
end

local function setChannel(freq, sf, cr)
    --  local HOP_PERIOD=0x24
    --  local FIFO_ADDR_PTR=0x0D
    --  local FIFO_RX_BASE_AD=0x0F
    --  local CR4_5=0x02
    --  local CRC_ON=0x04
    if (cr == nil) then
        cr = "4/5"
    end
    setFreq(freq)
    setRate(sf, _bw, MC1[cr], 0x04, 0x27, 14) -- CR4/5=0x02, CRC_ON=0x04
    write(0x24, 0x00)
    write(0x0D, read(0x0F))
end

function M.setChannel(freq, sf, cr)
    _freq = freq
    _sf = MC2[sf]
    _cr = cr
    setChannel(_freq, _sf, _cr)
end

local function txBuffer(data)
    --  local FIFO_ADDR_PTR=0x0D
    --  local FIFO_TX_BASE_AD=0x0E
    --  local PAYLOAD_LENGTH=0x22
    --  local FIFO=0x00

    write(0x0D, read(0x0E))
    write(0x22, #data)
    write(0x00, data)
end

local state = 0
local cadSF = 0

local function transmitPkt(freq, sf, bw, cr, crc, iiq, powe, data)
    print("Transmit pkt")
    state = 3 -- tx
    -- PREPARATION TO SEND
    idle()

    setFreq(freq)
    setRate(sf, bw, cr, crc, iiq, powe)

    -- reset FIFO address and paload length
    write(REG_FIFO_TX_BASE_ADDR, 0)
    write(REG_FIFO_ADDR_PTR, 0)
    write(REG_PAYLOAD_LENGTH, 0)
    write(REG_FIFO, data)

    write(REG_PAYLOAD_LENGTH, #data)
    write(REG_DIO_MAPPING_1, 0x40) -- DIO_MAPPING_1=0x40
    gpio.mode(_dio0, gpio.INT)
    gpio.trig(
        _dio0,
        "up",
        function()
            -- clear TxDone
            write(REG_IRQ_FLAGS, 0xFF)
            print("TxDone", node.heap())
            idle()
            _scanner()
        end
    )
    write(REG_OP_MODE, bor(MODE_LONG_RANGE_MODE, MODE_TX))
    M.txnb = M.txnb + 1
end

local function dio1handler()
    --  local IRQ_FLAGS=0x12
    --  local DIO_MAPPING_1=0x40

    if state == 0 or state == 1 then -- CAD_DETECTED
        write(0x01, 0x86) -- RX_SINGLE
        write(0x12, 0xFF) -- clear interrupt flags
        state = 2
        write(0x40, 0x03) -- DIO0 RxDone, DIO1 RxTimeout, DIO3 None
        delay(256)
        local rssi = read(0x1B)
        if rssi < 40 then
            _scanner()
        end
    elseif state == 2 then -- RX_TIMEOUT
        local rssi = read(0x1B)
        M.rxto = M.rxto + 1
        _scanner()
    end
end

local function dio0handler()
    --  local OP_MODE=0x01
    --  local IRQ_FLAGS=0x12
    --  local MODEM_CONFIG3=0x26
    --  local DIO_MAPPING_1=0x40
    --  local OPMODE_CAD=0x07

    tmst = now()
    -- CadDone
    if state == 1 then
        if cadSF < 0xC0 then -- try next SF
            cadSF = cadSF + 0x10 -- next SF
            write(0x1E, cadSF) -- set next SF
            if cadSF == 0xB4 then
                write(0x26, 0x0C) -- ModemConfig3: LowDataRateOptimize on
            end
            if cadSF == 0xA4 then
                write(0x1F, 0x05) -- RegSymbolTimeoutLSB: SymbolTimeout 5
            end
            write(0x01, 0x87) -- set mode LoRa CAD
            write(0x12, 0xFF) -- clear interrupt flags
            delay(256)
            local rssi = read(0x1B)
            if rssi < 40 then
                _scanner()
            end
        else
            _scanner() -- restart scanner
        end
    elseif state == 0 then
        write(0x01, 0x87) -- set mode LoRa CAD
        write(0x12, 0xFF) -- clear interrupt flags
        delay(256)
        local rssi = read(0x1B)
        if rssi > 42 then
            state = 1 -- CAD
        else
            -- hop()
        end
    elseif state == 2 then
        local flags = read(0x12)
        if band(flags, 0x40) == 0x40 then
            -- RxDone
            rxDone() -- handle message received
            _scanner() -- restart scanner
        end
    end
end

local function allSf()
    --  local RegOpMode=0x01
    --  local OPMODE_SLEEP=0x00
    --  local OPMODE_RX=0x05
    --  local DIO_MAPPING_1=0x40

    write(0x01, 0x81) -- set mode LoRa standby
    write(0x39, 0x34) -- syncword LoRaWan
    setChannel(_freq, _sf, _cr) -- channel settings in LoRa mode

    cadSF = _sf + 0x04 -- reset SF hopper

    gpio.mode(_dio0, gpio.INT)
    gpio.trig(_dio0, "up", dio0handler)
    gpio.mode(_dio1, gpio.INT)
    gpio.trig(_dio1, "up", dio1handler)
    write(0x40, 0xA3) -- DIO0 CadDone, DIO1 None, DIO3 None

    -- start
    write(0x01, 0x87) -- set mode LoRa CAD
    state = 0 -- RSSI detect
    write(0x12, 0xFF) -- clear interrupt flags
    delay(256)
    local rssi = read(0x1B)
    if rssi > 42 then
        state = 1 -- CAD
    end
end

local function singleSf()
    write(0x01, 0x81) -- set mode LoRa standby
    write(0x39, 0x34) -- syncword LoRaWan
    setChannel(_freq, _sf, _cr) -- channel settings in LoRa mode
    gpio.mode(_dio0, gpio.INT)
    gpio.trig(
        _dio0,
        "up",
        function()
            tmst = now()
            rxDone()
            print("RX done", node.heap())
            write(0x12, 0xFF) -- clear interrupt flags
        end
    )
    write(0x40, 0x03) -- DIO0 RxDone, DIO1 RxTimeout, DIO3 None
    write(0x01, 0x85) -- set mode LoRa rxContinuous
end

function M.rxpk(pkg)
end

local txTimer = tmr.create()

function M.txpk(pkt)
    print("M.txpk")
    local freq = pkt.freq
    local sf = MC2[pkt.datr:sub(1, -6)]
    local bw = MC1[pkt.datr:sub(-5)]
    local cr = MC1[pkt.codr]
    -- local crc = 0x00 -- crc disabled...
    local crc = 0x04 -- crc enabled...
    local iiq = 0x27
    if pkt.ipol == true then
        iiq = 0x40
    end
    local powe = pkt.powe
    local data = pkt.data
    local size = string.len(pkt.data)
    -- encoder.fromBase64(padBase64(pkt.data)):sub(1, size)
    -- trigger transmitPkt 30ms before the message has to be sent.
    -- a large margin is necessary because the ESP's timing is 'not so precise'

    transmitPkt(freq, sf, bw, cr, crc, iiq, powe, data, size)
end

local function sxInit()
    --  local VERSION=0x42
    --  local OPMODE_SLEEP=0x00
    --  local SYNC_WORD=0x39
    --  local LNA=0x0C
    --  local MAX_PAYLOAD_LENGTH=0x23
    --  local PAYLOAD_LENGTH=0x22
    --  local PREAMBLE_LSB=0x21
    --  local PA_RAMP=0x0A
    --  local PA_DAC=0x5A
    --  local LNA_MAX_GAIN=0x23

    local version = read(0x42)
    if (version ~= 0x12) then
        print("Unknown radio: ", version)
    end
    print("version", version)
    write(0x01, 0x80)
    write(0x39, 0x34)
    write(0x0C, 0x23)
    write(0x22, 0x40)
    write(0x21, 0x08)
    write(0x0A, bor(band(read(0x0A), 0xF0), 0x08)) -- set PA ramp-up time 50 uSec
    write(0x5A, bor(read(0x5A), 0x04))
    write(0x37, 0x0A) -- detection threshold
end

local function init(nss, dio0, dio1, freq, sf, bw, cr)
    --
    node.setcpufreq(node.CPU160MHZ)
    -- setup SPI
    spi.setup(1, spi.MASTER, spi.CPOL_LOW, spi.CPHA_LOW, spi.DATABITS_8, 3)
    _nss = nss
    gpio.mode(_nss, gpio.OUTPUT)
    -- init radio
    sxInit()
    -- setup handlers
    _dio0 = dio0
    _dio1 = dio1

    _freq = freq
    _cr = cr
    _bw = MC1[bw]
    if (sf == "ALL") then
        _sf = MC2["SF7"]
        _scanner = allSf
        print("start allSF detector")
    else
        _sf = MC2[sf]
        _scanner = singleSf
        print("start singleSF detector on", sf)
    end

    _scanner()

    return M
end

return init