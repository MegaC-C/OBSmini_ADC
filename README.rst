OBSmini_ADC
###########

Overview
********

This code is written for the OBSmini with the aim to visualize the recieved echo signals as measured by the nRF52833. 
The results can be used to tune limits/timings etc.

Building and Running
********************
 
with DFU_OTA:

the "merged.hex" is used to drag´n´drop flash (doesn't work at first flash, any "zephyr.bin" must be flashed first) and the "app_update.bin" is used for DFU.


without DFU_OTA:

the "zephyr.bin" is used to drag´n´drop flash (all these files are found in .../myProjectFolder/build/zephyr)

