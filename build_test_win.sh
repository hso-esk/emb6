#!/bin/sh
# Build test on windows
scons.bat --target=cs_atany900 --mac=0xcafe
scons.bat --target=cc_atany900 --mac=0xcafe
scons.bat --target=mq_atany900 --mac=0xcafe
scons.bat --target=if_atany900 --mac=0xcafe
scons.bat --target=mdc_atany900 --mac=0xcafe

scons.bat --target=cs_atany900pro5 --mac=0xcafe
scons.bat --target=ds_atany900pro5 --mac=0xcafe
scons.bat --target=md_atany900pro5_srv --mac=0xcafe
scons.bat --target=ud_atany900pro5_srv --mac=0xcafe
scons.bat --target=ap_atany900pro5_srv --mac=0xcafe

scons.bat --target=if_atany900basic --mac=0xcafe
scons.bat --target=cs_atany900basic --mac=0xcafe
scons.bat --target=as_atany900basic --mac=0xcafe
scons.bat --target=ac_atany900basic --mac=0xcafe
scons.bat --target=dc_atany900basic --mac=0xcafe
scons.bat --target=md_atany900basic_cli --mac=0xcafe
scons.bat --target=md_atany900basic_srv --mac=0xcafe
scons.bat --target=ud_atany900basic_cli --mac=0xcafe
scons.bat --target=ud_atany900basic_srv --mac=0xcafe

scons.bat --target=as_samd20xpro_rf212 --mac=0xcafe
scons.bat --target=cs_xpro_212b --mac=0xcafe
scons.bat --target=mq_xpro_212b --mac=0xcafe
scons.bat --target=md_samd20xpro_rf212b_cli --mac=0xcafe
scons.bat --target=md_samd20xpro_rf212_srv --mac=0xcafe
scons.bat --target=ds_samd20xpro_rf212b --mac=0xcafe
scons.bat --target=ud_samd20xpro_rf212 --mac=0xcafe
scons.bat --target=ud_samd20xpro_rf212_cli --mac=0xcafe

scons.bat --target=dc_stk3600 --mac=0xcafe
scons.bat --target=ds_stk --mac=0xcafe
scons.bat --target=ds12 --mac=0xcafe
scons.bat --target=dc12 --mac=0xcafe
scons.bat --target=ds11 --mac=0xcafe
scons.bat --target=dc11 --mac=0xcafe
scons.bat --target=us_stk3600 --mac=0xcafe
scons.bat --target=us_stk3600_cc112x --mac=0xcafe
scons.bat --target=uc_stk3600_cc112x --mac=0xcafe
scons.bat --target=us_stk3600_cc120x --mac=0xcafe
scons.bat --target=uc_stk3600_cc120x --mac=0xcafe
scons.bat --target=mdc_stk3600_cc112x --mac=0xcafe
scons.bat --target=mds_stk3600_cc112x --mac=0xcafe
scons.bat --target=mdc_stk3600_cc120x --mac=0xcafe
scons.bat --target=mds_stk3600_cc120x --mac=0xcafe
scons.bat --target=ua_stk_cc112x --mac=0xcafe
scons.bat --target=ua_stk_cc120x --mac=0xcafe
scons.bat --target=cs_stk3600 --mac=0xcafe
scons.bat --target=cs_StkCC120x --mac=0xcafe
scons.bat --target=cc_StkCC120x --mac=0xcafe
scons.bat --target=cs_StkCC112x --mac=0xcafe
scons.bat --target=cc_StkCC112x --mac=0xcafe
scons.bat --target=cs_moses --mac=0xcafe
scons.bat --target=uc_moses --mac=0xcafe
scons.bat --target=us_moses --mac=0xcafe
scons.bat --target=if_mosesstick --mac=0xcafe
scons.bat --target=uc_mosesstick --mac=0xcafe
scons.bat --target=mq_stk112x --mac=0xcafe
scons.bat --target=mq_stk120x --mac=0xcafe
scons.bat --target=mq_moses --mac=0xcafe
