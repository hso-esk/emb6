#!/bin/sh
# Building test on linux
scons --target=cs_luxsrv --mac=0xcafe &&
scons --target=cs_luxcli --mac=0xcafe &&
scons --target=if_lux --mac=0xcafe &&
scons --target=dc_lux --mac=0xcafe &&
scons --target=ds_lux --mac=0xcafe &&
scons --target=md_x86cli --mac=0xcafe &&
scons --target=md_x86srv --mac=0xcafe