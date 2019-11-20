# TTGO LoRa32 ESP32 Oled board

This repository holds some sample code, using platformio IDE, for using this board peripherals:

 - Lora - Use the sx127x radio to connect to the TheThingsNetwork
 - BLE  - Esp32 Bluetooth 
 - WIFI - Native WIFI

The following examples are on this repository:

## TTN ABP (TTN_TTGOLora32 folder)

This code uses the Lora chip to connect to the Things Network, using ABP to join the network.

## TTN OTAA (TTN_TTGOLora32_OTAA folder)

This code also connects to TheThingsNetwork, but this time using Over The Air Activation.

## TTN NodeRed

The TTN Node red folder contains a node-red flow that by using the Node-Red TTN node, receives data from the above examples (either ABP or OTAA), and sends back information to the node regarding the signal uplink rssi strenght for that message.
If the TTGO board is range of a gateway with downlink capacity (all of them, except a few single channel gateways), then the board displays the received data (which is the signal strenght) on the OLED. 

## BLEBridge (TTN_TTGOLora32_BLEBridge folder)

The BLE Bridge is a simple example of bridging data from the BLE transceiver to the TheThingsNetwork Lorawan.

# WiFi Scanner 

Simple Wifi scanner that shows found WIFI networks on the Oled.
