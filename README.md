# WaterMeter-MS-3.0i-RFM95

Data from the sensor to MQTT-gateway to MQTT to (Telegraf to InFluxdb to Grafinea)
Controller function via NODE.RED connected via MQTT.


It measures water flow from a Dwyer WMT2 water meter

Used a 0 to 100PSI pressure sensor (ebay), via an analog port

Support MCP9800 or Si7021 (humidity also) for air temp

Two relay outputs

3 Analog inputs

MAX6816 or analog filter for contact debouncing for water meter

Powered via an 8 to 28v AC or DC external power source

(Board is designed to fit in a Hammond 1554 case)

Processor is a Moteino R4 LoRa with a RFM95


https://www.dwyer-inst.com/Product/Flow/WaterMeters/SeriesWMT2

https://lowpowerlab.com/shop/product/99

