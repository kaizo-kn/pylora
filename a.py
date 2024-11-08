import RPi.GPIO as GPIO
import spidev
import time
from datetime import datetime

# Konfigurasi pin
LORA_RST_PIN = 17
LORA_CS_PIN = 27
LORA_IRQ_PIN = 22

# Register LoRa
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_IRQ_FLAGS = 0x12
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE_ADDR = 0x0E
REG_PAYLOAD_LENGTH = 0x22
REG_VERSION = 0x42

# Mode operasi
MODE_LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03

class LoRaTransmitter:
    def __init__(self):
        self.spi = None
        self.counter = 0

    def get_timestamp(self):
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def init(self):
        try:
            # Inisialisasi GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(LORA_RST_PIN, GPIO.OUT)
            GPIO.setup(LORA_CS_PIN, GPIO.OUT)
            GPIO.setup(LORA_IRQ_PIN, GPIO.IN)

            # Inisialisasi SPI
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz = 5000000
            self.spi.mode = 0

            # Reset dan cek chip
            self.reset()
            if self.read_register(REG_VERSION) != 0x12:
                return False

            # Konfigurasi dasar
            self.sleep()
            self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)
            
            # Set frekuensi 433MHz
            self.set_frequency(433.0)
            
            # Set power 17dBm
            self.set_tx_power(17)
            
            # Konfigurasi modem
            self.write_register(0x1D, 0x72)  # BW=125kHz, CR=4/5
            self.write_register(0x1E, 0x74)  # SF=7, CRC on
            self.write_register(0x26, 0x04)  # Low Data Rate Optimize

            self.standby()
            return True

        except Exception as e:
            print(f"[{self.get_timestamp()}] Error inisialisasi: {str(e)}")
            return False

    def reset(self):
        GPIO.output(LORA_RST_PIN, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(LORA_RST_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(LORA_RST_PIN, GPIO.HIGH)
        time.sleep(0.01)

    def write_register(self, address, value):
        GPIO.output(LORA_CS_PIN, GPIO.LOW)
        self.spi.xfer([address | 0x80, value])
        GPIO.output(LORA_CS_PIN, GPIO.HIGH)

    def read_register(self, address):
        GPIO.output(LORA_CS_PIN, GPIO.LOW)
        response = self.spi.xfer([address & 0x7F, 0])[1]
        GPIO.output(LORA_CS_PIN, GPIO.HIGH)
        return response

    def sleep(self):
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)

    def standby(self):
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)

    def set_frequency(self, freq_mhz):
        freq = int(freq_mhz * 16384 / 0.061035)
        self.write_register(REG_FRF_MSB, (freq >> 16) & 0xFF)
        self.write_register(REG_FRF_MID, (freq >> 8) & 0xFF)
        self.write_register(REG_FRF_LSB, freq & 0xFF)

    def set_tx_power(self, level):
        level = max(2, min(17, level))
        self.write_register(REG_PA_CONFIG, 0x80 | (level - 2))

    def transmit(self):
        print(f"[{self.get_timestamp()}] LoRa Transmitter aktif. Tekan Ctrl+C untuk berhenti.")
        try:
            while True:
                timestamp = self.get_timestamp()
                message = f"{self.counter} pesan terkirim dari Lora Transmitter"
                print(f"\n[{timestamp}] Mengirim: {message}")
                
                # Encode message to UTF-8
                if isinstance(message, str):
                    message = message.encode('utf-8')  # Encode to UTF-8 bytes
                
                # Set payload length based on the byte-length of the encoded message
                payload_length = len(message)
                self.write_register(REG_PAYLOAD_LENGTH, payload_length)
                
                # Send the message
                if self.send_message(message):
                    print(f"[{self.get_timestamp()}] Pesan terkirim!")
                else:
                    print(f"[{self.get_timestamp()}] Gagal mengirim")
                
                self.counter += 1
                time.sleep(1)  # Adjust sleep for interval between messages

        except KeyboardInterrupt:
            print(f"\n[{self.get_timestamp()}] Program dihentikan")



    def transmit(self):
        print(f"[{self.get_timestamp()}] LoRa Transmitter aktif. Tekan Ctrl+C untuk berhenti.")
        try:
            while True:
                timestamp = self.get_timestamp()
                message = f"{self.counter} pesan terkirim dari Lora Transmitter"
                print(f"\n[{timestamp}] Mengirim: {message}")
                
                if self.send_message(message):
                    print(f"[{self.get_timestamp()}] Pesan terkirim!")
                else:
                    print(f"[{self.get_timestamp()}] Gagal mengirim")
                
                self.counter += 1
                time.sleep(1)

        except KeyboardInterrupt:
            print(f"\n[{self.get_timestamp()}] Program dihentikan")

    def close(self):
        if self.spi:
            self.spi.close()
        GPIO.cleanup()

def main():
    lora = LoRaTransmitter()
    try:
        if lora.init():
            print(f"[{lora.get_timestamp()}] LoRa Transmitter berhasil diinisialisasi.")
            lora.transmit()
        else:
            print(f"[{lora.get_timestamp()}] Gagal menginisialisasi LoRa")
    finally:
        lora.close()

if __name__ == "__main__":
    main()