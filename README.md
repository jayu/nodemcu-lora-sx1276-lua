# NodeMCU LoRa module library

NodeMCU lua library for LoRa SX1276/SX1278 radio communication modules

The library is a based on the library from https://github.com/JaapBraam/LoRaWanGateway 

The improvements:
- fixed a bug with incorrect operations on message buffers which result in sending garbage instead of actual message
- enhanced power management to support 20dBm mode
  - implementation was inspired by https://github.com/sandeepmistry/arduino-LoRa/blob/master/src/LoRa.cpp

I'm not an expert in implementing such libraries and I don't understand most of these registers operations, but I made it work for me somehow 😃

Most of the credits goes to authors of the mentioned libraries who had to study SX1276 manual a lot 🧐

## Usage

### API

```lua
local radioLib = require('sx1276.lua')

local nssPin = 0
local dio0Pin = 1
local dio1Pin = 2

local frequency = 433.000 * 1000000 -- Frequency in Hz
local spreadingFactor = "SF7" -- "SF7" - "SF12" 
local bandWidth = "BW125" -- I think only this one is supported
local errorCorrection = "4/5" -- I believe these are supported: "4/5", "4/6", "4/7", "4/8"

local radioInterface = radioLib(
  nssPin,
  dio0Pin,
  dio1Pin,
  frequency,
  spreadingFactor,
  bandWidth,
  errorCorrection
)

-- Receiving

local function handleMessage(pkt)
  print("RX!")
  print("Data: ", pkt.data)
  print("RSSI: ", pkt.rssi)
  for k, v in pairs(pkt) do
      print(k, v)
  end
end

radioInterface.rxpk = handleMessage

-- Transmitting

local pkt = {
    freq = frequency,
    codr = errorCorrection,
    datr = spreadingFactor .. bandWidth,
    data = "Here goes your message",
    powe = 17 -- values 0..20
}

radio.txpk(pkt)

```

### Connections 

_Stolen from https://github.com/JaapBraam/LoRaWanGateway/blob/master/README.md#hardware_

<table>
<tr><th>ESP PIN</th><th>SX1276 PIN</th></tr>
<tr><td>D1 (GPIO5)</td><td>DIO0</td></tr>
<tr><td>D2 (GPIO4)</td><td>DIO1</td></tr>
<tr><td>D5 (GPIO14)</td><td>SCK</td></tr>
<tr><td>D6 (GPIO12)</td><td>MISO</td></tr>
<tr><td>D7 (GPIO13)</td><td>MOSI</td></tr>
<tr><td>D0 (GPIO16)</td><td>NSS</td></tr>
<tr><td>GND</td><td>GND</td></tr>
<tr><td>3.3V</td><td>VCC</td></tr>
</table>

## TODO (a.k.a. roadmap)

The library needs a lot of refactoring, feel free to open a PR!

- [ ] refactor global variables with register numbers into object with all registers
- [ ] refactor all register magic numbers to variables with register numbers
- [ ] remove code responsible for handling multiple spreading factors
- [ ] make the API more friendly
- [ ] implement support for low power mode - right now NodeMCU has to be ON all the time to handle incoming messages, but there is a way to use interrupts from SX1276 to wake up NodeMCU
