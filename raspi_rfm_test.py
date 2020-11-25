import time
import digitalio
import board
import busio
import adafruit_rfm9x


RADIO_FREQ_MHZ = 433.0
CS = digitalio.DigitalInOut(board.D5)
RESET = digitalio.DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
duration = 25

# Initialze RFM radio with a more conservative baudrate
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)


for i in range(duration):
	print('Sending packet of 8 bytes...')
		rfm9x.send(bytes([0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]))
		time.sleep(1)
