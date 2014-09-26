ble_app_uart_c
==============

Client example with 128 bit UUID service&amp;characteristics for S120

It's based on the ble_app_hrs_c. The HRS service was replaced by the NUS (UART) service.
The peer device (peripheral device) can be used conjunction with this example is the ble_app_uart available in SDK v6.0 and earlier. 

Be noted that the charactersitic's names and UUID were copied from the original ble_app_uart so that the 2 examples matched.
It may not match with the description of the RX and TX characteristics (reversed)

Requirements


nRF51 SDK version 6.1

S110 v7.0 and above. 

nRF51822 Development Kit version 2.1.0 or later

The project may need modifications to work with other versions or other boards.



About this project

This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty.

However, in the hope that it still may be useful also for others than the ones we initially wrote it for, we've chosen to distribute it here on GitHub.

The application is built to be used with the official nRF51 SDK, that can be downloaded from https://www.nordicsemi.no, provided you have a product key for one of our kits.

Please post any questions about this project on https://devzone.nordicsemi.com.
