import time
import digitalio
import board
import busio
import RPi.GPIO as io
import adafruit_rfm9x

RADIO_FREQ_MHZ = 433.0
CS = digitalio.DigitalInOut(board.D5)
RESET = digitalio.DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# Initialze RFM radio with a more conservative baudrate
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)



print(rfm9x._read_u8(adafruit_rfm9x._RH_RF95_REG_00_FIFO))
