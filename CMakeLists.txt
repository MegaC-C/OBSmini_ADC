# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

###############################################################################################################
# here is the best place for adding mcuboot CONFIGs, as the root folder remains untouched
# only rsa-2048 key did work for my config, not clear why.
# folder with the custom key is added to project folder and defined below
# key generated via command in ncs/vX.X.X folder: python3 bootloader/mcuboot/scripts/imgtool.py keygen -t rsa-2048 -k priv.pem

set(mcuboot_CONFIG_BOOT_SIGNATURE_KEY_FILE \"${CMAKE_CURRENT_SOURCE_DIR}/custom_key_dir/OBSmini-rsa-2048.pem\")
set(mcuboot_CONFIG_DISABLE_FLASH_PATCH y)
###############################################################################################################

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(OBSmini_ADC)

target_sources(app PRIVATE src/main.c)
