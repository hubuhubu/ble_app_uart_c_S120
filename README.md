ble_app_uart_c
==============

Client example with 128 bit UUID service&amp;characteristics for S120

It's based on the ble_app_hrs_c. The HRS service was replaced by the NUS (UART) service.
This project can be used in conjunction with the UART serial port emulation example from SDK v8.0.0 (http://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v8.x.x/doc/8.0.0/s110/html/a00072.html).

Functionality:
- Scan for and connect to a peripheral that advertise with the 128bit UUID NUS service
- Do service discovery and notify the application if the NUS UUID service found
- Enable RX CCCD for notification 
- Forward data received from the peer device TX Characteristic to UART
- Forward data received on UART to the peer device RX Characteristic

Be noted that the Characteristic's names and UUID were copied from the original ble_app_uart so that the 2 examples matched.
It may not match with the description of the RX and TX characteristics (reversed)


Requirements


nRF51 SDK version 8.0.0

S120 v2.0 

nRF51-DK, or nRF51-Dongle, version 1.0.0 or later

The project may need modifications to work with other versions or other boards.



About this project

This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty.

However, in the hope that it still may be useful also for others than the ones we initially wrote it for, we've chosen to distribute it here on GitHub.

The application is built to be used with the official nRF51 SDK, that can be downloaded from https://www.nordicsemi.no, provided you have a product key for one of our kits.

Please post any questions about this project on https://devzone.nordicsemi.com.
