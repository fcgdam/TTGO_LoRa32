# TTGO LoRa32 ESP32 Oled board

This repository holds some sample code, using platformio IDE, for using this board peripherals:

 Lora - Use the sx127x radio to connect to the TheThingsNetwork
 BLE  - Esp32 Bluetooth 
 WIFI - Native WIFI

The following examples are on this repository:

## TTN ABP

This code uses the Lora chip to connect to the Things Network, using ABP to join the network.

## TTN OTAA

This code also connects to TheThingsNetwork, but this time using Over The Air Activation.

## TTN NodeRed

The TTN Node rede folder contains a node-red flow that using thei Node-Red TTN node, receives data from the above examples, and sends back information regarding the receive uplink rssi to the nodes.
The TTGO board then displays the receiveid data on the OLED. 

## BLEBridge

The BLE Bridge is a simple example of bridging data from the BLE transceiver to the TheThingsNetwork Lorawan.

# WiFi Scanner - Arduino

Simple Wifi scanner that shows found networks on the Oled.
