import time
import digitalio
import board
import busio
import RPi.GPIO as io
import RFM98_test
import sys

RADIO_FREQ_MHZ = 433.0
CS = digitalio.DigitalInOut(board.D5)
RESET = digitalio.DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = RFM98_test.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)


sys.stdout = open('Registers.txt','w')
print('RF98_REG_00_FIFO:             ',rfm9x._read_u8(RFM98_test.RF98_REG_00_FIFO ))
print('RF98_REG_01_OP_MODE:          ',rfm9x._read_u8(RFM98_test.RF98_REG_01_OP_MODE ))
print('RF98_REG_02_BITRATE_MSB:      ',rfm9x._read_u8(RFM98_test.RF98_REG_02_BITRATE_MSB ))
print('RF98_REG_03_BITRATE_LSB:      ',rfm9x._read_u8(RFM98_test.RF98_REG_03_BITRATE_LSB ))
print('RF98_REG_04_F_DEV_MSB:        ',rfm9x._read_u8(RFM98_test.RF98_REG_04_F_DEV_MSB ))
print('RF98_REG_05_F_DEV_LSB:        ',rfm9x._read_u8(RFM98_test.RF98_REG_05_F_DEV_LSB ))
print('RF98_REG_06_FRF_MSB:          ',rfm9x._read_u8(RFM98_test.RF98_REG_06_FRF_MSB ))
print('RF98_REG_07_FRF_MID:          ',rfm9x._read_u8(RFM98_test.RF98_REG_07_FRF_MID ))
print('RF98_REG_08_FRF_LSB:          ',rfm9x._read_u8(RFM98_test.RF98_REG_08_FRF_LSB ))
print('RF98_REG_09_PA_CONFIG:        ',rfm9x._read_u8(RFM98_test.RF98_REG_09_PA_CONFIG ))
print('RF98_REG_0A_PA_RAMP:          ',rfm9x._read_u8(RFM98_test.RF98_REG_0A_PA_RAMP ))
print('RF98_REG_0B_OCP:              ',rfm9x._read_u8(RFM98_test.RF98_REG_0B_OCP ))
print('RF98_REG_0C_LNA:              ',rfm9x._read_u8(RFM98_test.RF98_REG_0C_LNA ))
print('RF98_REG_0D_RX_CONFIG:        ',rfm9x._read_u8(RFM98_test.RF98_REG_0D_RX_CONFIG ))
print('RF98_REG_0E_RSSI_CONFIG:      ',rfm9x._read_u8(RFM98_test.RF98_REG_0E_RSSI_CONFIG ))
print('RF98_REG_0F_RSSI_COLLISION:   ',rfm9x._read_u8(RFM98_test.RF98_REG_0F_RSSI_COLLISION ))
print('RF98_REG_10_RSSI_THRESH:      ',rfm9x._read_u8(RFM98_test.RF98_REG_10_RSSI_THRESH ))
print('RF98_REG_11_RSSI_VALUE:       ',rfm9x._read_u8(RFM98_test.RF98_REG_11_RSSI_VALUE ))
print('RF98_REG_12_RX_BW:            ',rfm9x._read_u8(RFM98_test.RF98_REG_12_RX_BW ))
print('RF98_REG_13_AFC_BW:           ',rfm9x._read_u8(RFM98_test.RF98_REG_13_AFC_BW ))
print('RF98_REG_17_RESERVED:         ',rfm9x._read_u8(RFM98_test.RF98_REG_17_RESERVED ))
print('RF98_REG_18_RESERVED:         ',rfm9x._read_u8(RFM98_test.RF98_REG_18_RESERVED ))
print('RF98_REG_19_RESERVED:         ',rfm9x._read_u8(RFM98_test.RF98_REG_19_RESERVED ))
print('RF98_REG_1A_AFC_FEI:          ',rfm9x._read_u8(RFM98_test.RF98_REG_1A_AFC_FEI ))
print('RF98_REG_1B_AFC_MSB:          ',rfm9x._read_u8(RFM98_test.RF98_REG_1B_AFC_MSB ))
print('RF98_REG_1C_AFC_LSB:          ',rfm9x._read_u8(RFM98_test.RF98_REG_1C_AFC_LSB ))
print('RF98_REG_1D_FEI_MSB:          ',rfm9x._read_u8(RFM98_test.RF98_REG_1D_FEI_MSB ))
print('RF98_REG_1E_FEI_LSB:          ',rfm9x._read_u8(RFM98_test.RF98_REG_1E_FEI_LSB ))
print('RF98_REG_1F_PREAMBLE_DETECT:  ',rfm9x._read_u8(RFM98_test.RF98_REG_1F_PREAMBLE_DETECT ))
print('RF98_REG_20_RX_TIMEOUT_1:     ',rfm9x._read_u8(RFM98_test.RF98_REG_20_RX_TIMEOUT_1 ))
print('RF98_REG_21_RX_TIMEOUT_2:     ',rfm9x._read_u8(RFM98_test.RF98_REG_21_RX_TIMEOUT_2 ))
print('RF98_REG_22_RX_TIMEOUT_3:     ',rfm9x._read_u8(RFM98_test.RF98_REG_22_RX_TIMEOUT_3 ))
print('RF98_REG_23_RX_DELAY:         ',rfm9x._read_u8(RFM98_test.RF98_REG_23_RX_DELAY ))
print('RF98_REG_24_OSC:              ',rfm9x._read_u8(RFM98_test.RF98_REG_24_OSC ))
print('RF98_REG_25_PREAMBLE_MSB:     ',rfm9x._read_u8(RFM98_test.RF98_REG_25_PREAMBLE_MSB ))
print('RF98_REG_26_PREAMBLE_LSB:     ',rfm9x._read_u8(RFM98_test.RF98_REG_26_PREAMBLE_LSB ))
print('RF98_REG_27_SYNC_CONFIG:      ',rfm9x._read_u8(RFM98_test.RF98_REG_27_SYNC_CONFIG ))
print('RF98_REG_28_SYNC_VALUE_1:     ',rfm9x._read_u8(RFM98_test.RF98_REG_28_SYNC_VALUE_1 ))
print('RF98_REG_29_SYNC_VALUE_2:     ',rfm9x._read_u8(RFM98_test.RF98_REG_29_SYNC_VALUE_2 ))
print('RF98_REG_2A_SYNC_VALUE_3:     ',rfm9x._read_u8(RFM98_test.RF98_REG_2A_SYNC_VALUE_3 ))
print('RF98_REG_2B_SYNC_VALUE_4:     ',rfm9x._read_u8(RFM98_test.RF98_REG_2B_SYNC_VALUE_4 ))
print('RF98_REG_2C_SYNC_VALUE_5:     ',rfm9x._read_u8(RFM98_test.RF98_REG_2C_SYNC_VALUE_5 ))
print('RF98_REG_2D_SYNC_VALUE_6:     ',rfm9x._read_u8(RFM98_test.RF98_REG_2D_SYNC_VALUE_6 ))
print('RF98_REG_2E_SYNC_VALUE_7:     ',rfm9x._read_u8(RFM98_test.RF98_REG_2E_SYNC_VALUE_7 ))
print('RF98_REG_2F_SYNC_VALUE_8:     ',rfm9x._read_u8(RFM98_test.RF98_REG_2F_SYNC_VALUE_8 ))
print('RF98_REG_30_PACKET_CONFIG_1:  ',rfm9x._read_u8(RFM98_test.RF98_REG_30_PACKET_CONFIG_1 ))
print('RF98_REG_31_PACKET_CONFIG_2:  ',rfm9x._read_u8(RFM98_test.RF98_REG_31_PACKET_CONFIG_2 ))
print('RF98_REG_32_PAYLOAD_LENGTH:   ',rfm9x._read_u8(RFM98_test.RF98_REG_32_PAYLOAD_LENGTH ))
print('RF98_REG_2C_SYNC_VALUE_5:     ',rfm9x._read_u8(RFM98_test.RF98_REG_2C_SYNC_VALUE_5 ))
print('RF98_REG_2D_SYNC_VALUE_6:     ',rfm9x._read_u8(RFM98_test.RF98_REG_2D_SYNC_VALUE_6 ))
print('RF98_REG_2E_SYNC_VALUE_7:     ',rfm9x._read_u8(RFM98_test.RF98_REG_2E_SYNC_VALUE_7 ))
print('RF98_REG_2F_SYNC_VALUE_8:     ',rfm9x._read_u8(RFM98_test.RF98_REG_2F_SYNC_VALUE_8 ))
print('RF98_REG_30_PACKET_CONFIG_1:  ',rfm9x._read_u8(RFM98_test.RF98_REG_30_PACKET_CONFIG_1 ))
print('RF98_REG_31_PACKET_CONFIG_2:  ',rfm9x._read_u8(RFM98_test.RF98_REG_31_PACKET_CONFIG_2 ))
print('RF98_REG_32_PAYLOAD_LENGTH:   ',rfm9x._read_u8(RFM98_test.RF98_REG_32_PAYLOAD_LENGTH ))
print('RF98_REG_33_NODE_ADRS:        ',rfm9x._read_u8(RFM98_test.RF98_REG_33_NODE_ADRS ))
print('RF98_REG_34_BROADCAST_ADRS:   ',rfm9x._read_u8(RFM98_test.RF98_REG_34_BROADCAST_ADRS ))
print('RF98_REG_35_FIFO_THRESH:      ',rfm9x._read_u8(RFM98_test.RF98_REG_35_FIFO_THRESH ))
print('RF98_REG_36_SEQ_CONFIG_1:     ',rfm9x._read_u8(RFM98_test.RF98_REG_36_SEQ_CONFIG_1 ))
print('RF98_REG_37_SEQ_CONFIG_2:     ',rfm9x._read_u8(RFM98_test.RF98_REG_37_SEQ_CONFIG_2 ))
print('RF98_REG_38_TIMER_RESOL:      ',rfm9x._read_u8(RFM98_test.RF98_REG_38_TIMER_RESOL ))
print('RF98_REG_39_TIMER_1_COEF:     ',rfm9x._read_u8(RFM98_test.RF98_REG_39_TIMER_1_COEF ))
print('RF98_REG_3A_TIMER_2_COEF:     ',rfm9x._read_u8(RFM98_test.RF98_REG_3A_TIMER_2_COEF ))
print('RF98_REG_3B_IMAGE_CAL:        ',rfm9x._read_u8(RFM98_test.RF98_REG_3B_IMAGE_CAL ))
print('RF98_REG_3C_TEMP:             ',rfm9x._read_u8(RFM98_test.RF98_REG_3C_TEMP ))
print('RF98_REG_3D_LOW_BAT:          ',rfm9x._read_u8(RFM98_test.RF98_REG_3D_LOW_BAT ))
print('RF98_REG_3E_IRQ_FLAGS_1:      ',rfm9x._read_u8(RFM98_test.RF98_REG_3E_IRQ_FLAGS_1 ))
print('RF98_REG_3F_IRQ_FLAGS_2:      ',rfm9x._read_u8(RFM98_test.RF98_REG_3F_IRQ_FLAGS_2 ))
print('RF98_REG_40_DIO_MAPPING_1:    ',rfm9x._read_u8(RFM98_test.RF98_REG_40_DIO_MAPPING_1 ))
print('RF98_REG_41_DIO_MAPPING_2:    ',rfm9x._read_u8(RFM98_test.RF98_REG_41_DIO_MAPPING_2 ))
print('RF98_REG_42_VERSION:          ',rfm9x._read_u8(RFM98_test.RF98_REG_42_VERSION ))
print('RF98_REG_44_PLL_HOP:          ',rfm9x._read_u8(RFM98_test.RF98_REG_44_PLL_HOP ))
print('RF98_REG_4B_TCXO:             ',rfm9x._read_u8(RFM98_test.RF98_REG_4B_TCXO ))
print('RF98_REG_4D_PA_DAC:           ',rfm9x._read_u8(RFM98_test.RF98_REG_4D_PA_DAC ))
print('RF98_REG_5B_FORMER_TEMP:      ',rfm9x._read_u8(RFM98_test.RF98_REG_5B_FORMER_TEMP ))
print('RF98_REG_5D_BITRATE_FRAC:     ',rfm9x._read_u8(RFM98_test.RF98_REG_5D_BITRATE_FRAC ))
print('RF98_REG_61_AGC_REF:          ',rfm9x._read_u8(RFM98_test.RF98_REG_61_AGC_REF ))
print('RF98_REG_62_AGC_THRESH_1:     ',rfm9x._read_u8(RFM98_test.RF98_REG_62_AGC_THRESH_1 ))
print('RF98_REG_63_AGC_THRESH_2:     ',rfm9x._read_u8(RFM98_test.RF98_REG_63_AGC_THRESH_2 ))
print('RF98_REG_64_AGC_THRESH_3:     ',rfm9x._read_u8(RFM98_test.RF98_REG_64_AGC_THRESH_3 ))
print('RF98_REG_70_PLL:              ',rfm9x._read_u8(RFM98_test.RF98_REG_70_PLL ))
print('RF98_PA_DAC_DISABLE:          ',rfm9x._read_u8(RFM98_test.RF98_PA_DAC_DISABLE ))
print('RF98_PA_DAC_ENABLE:           ',rfm9x._read_u8(RFM98_test.RF98_PA_DAC_ENABLE ))
print('_RH_RF95_FXOSC:               ',(RFM98_test._RH_RF95_FXOSC ))
print('_RH_RF95_FSTEP:               ',(RFM98_test._RH_RF95_FSTEP ))
print('_RH_BROADCAST_ADDRESS:        ',rfm9x._read_u8(RFM98_test._RH_BROADCAST_ADDRESS ))
print('SLEEP_MODE:                   ',(RFM98_test.SLEEP_MODE ))
print('STANDBY_MODE:                 ',(RFM98_test.STANDBY_MODE ))
print('FS_TX_MODE:                   ',(RFM98_test.FS_TX_MODE ))
print('TX_MODE:                      ',(RFM98_test.TX_MODE ))
print('FS_RX_MODE:                   ',(RFM98_test.FS_RX_MODE ))
print('RX_MODE:                      ',(RFM98_test.RX_MODE ))
sys.stdout.close()
