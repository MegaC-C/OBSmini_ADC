OBSmini_ADC
###########

Overview
********

This code is written for the OBSmini with the aim to visualize the recieved echo signals as measured by the nRF52833. 
The results can be used to tune limits/timings etc.
Most code comes from official examples:


NFC/Powermanagement:

nrf/samples/nfc/system_off
 
SAADC/PPI:

https://github.com/NordicPlayground/nRF52-ADC-examples/tree/master/nrfx_saadc_multi_channel_ppi
 
BLE:

webinar: Developing Bluetooth Low Energy products using nRF Connect SDK
 
PWM:

nRF5_SDKv17.0.2/examples/peripheral/pwm_driver
 
DFU_OTA:

https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/working_with_nrf/nrf52/developing.html#fota-updates
 
MCUBoot:

https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/app_dev/bootloaders_and_dfu/index.html

https://github.com/hellesvik-nordic/samples_for_nrf_connect_sdk/tree/1111836cd720127c7f2b0dc0bec9f7ef496b8954/bootloader_samples
 
 
Building and Running
********************
 
with DFU_OTA:

the "merged.hex" is used to drag´n´drop flash (doesn't work at first flash, any "zephyr.bin" must be flashed first) and the "app_update.bin" is used for DFU.

without DFU_OTA:

the "zephyr.bin" is used to drag´n´drop flash (all these files are found in ...\myProjectFolder\build\zephyr)

