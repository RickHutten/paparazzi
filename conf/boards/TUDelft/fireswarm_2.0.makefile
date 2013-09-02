# Hey Emacs, this is a -*- makefile -*-
#
# lisa_m_2.0.makefile
#
# http://paparazzi.enac.fr/wiki/Lisa/M
#

BOARD=TUDelft/fireswarm
BOARD_VERSION=2.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"
ifneq ($(NO_LUFTBOOT),1)
NO_LUFTBOOT=0
endif

ARCH=stm32
$(TARGET).ARCHDIR = $(ARCH)
# not needed?
$(TARGET).OOCD_INTERFACE=flossjtag
#$(TARGET).OOCD_INTERFACE=jtagkey-tiny
$(TARGET).LDSCRIPT=$(SRC_ARCH)/lisa-m.ld

# -----------------------------------------------------------------------

ifndef FLASH_MODE
FLASH_MODE = DFU
#FLASH_MODE = JTAG
#FLASH_MODE = SERIAL
endif

ifneq ($(NO_LUFTBOOT),1)
$(TARGET).CFLAGS+=-DLUFTBOOT
$(TARGET).LDFLAGS+=-Wl,-Ttext=0x8002000
endif

#
#
# some default values shared between different firmwares
#
#


#
# default LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= none
SYS_TIME_LED       ?= 3


#
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART3
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT ?= UART5

MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

GPS_PORT ?= UART1
GPS_BAUD ?= B38400


#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
#
ifndef ACTUATORS
ACTUATORS = actuators_pwm
endif


ifndef ADC_IR1
ADC_IR1      = 1
ADC_IR1_CHAN = 0
endif
ifndef ADC_IR2
ADC_IR2      = 2
ADC_IR2_CHAN = 1
endif
ifndef ADC_IR3
ADC_IR_TOP      = 3
ADC_IR_TOP_CHAN = 2
endif
ifndef ADC_IR_NB_SAMPLES
ADC_IR_NB_SAMPLES = 16
endif