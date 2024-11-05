import RPi.GPIO as GPIO
import spidev
import time
import os

# Konfigurasi pin
LORA_RST_PIN = 17      # GPIO pin untuk reset
LORA_CS_PIN = 27       # GPIO pin untuk chip select
LORA_IRQ_PIN = 22      # GPIO pin untuk interrupt

# Register LoRa
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_FIFO_RX_BASE_ADDR = 0x0F
REG_FIFO_ADDR_PTR = 0x0D
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_SYNC_WORD = 0x39

# Mode operasi LoRa
MODE_LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_RX_CONTINUOUS = 0x05

class LoRaReceiver:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.spi = None

    def init(self):
        # Inisialisasi GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(LORA_RST_PIN, GPIO.OUT)
        GPIO.setup(LORA_CS_PIN, GPIO.OUT)
        GPIO.setup(LORA_IRQ_PIN, GPIO.IN)

        # Inisialisasi SPI
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 500000
        self.spi.mode = 0

        # Reset LoRa
        self.reset()

        # Set ke mode standby
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)

        # Set frekuensi (433 MHz)
        self.set_frequency(433.0)

        # Set base address untuk menerima data
        self.write_register(REG_FIFO_RX_BASE_ADDR, 0)
        self.write_register(REG_FIFO_ADDR_PTR, 0)

        # Set ke mode RX continuous
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)

        print("LoRa Receiver berhasil diinisialisasi dan siap menerima pesan")

    def reset(self):
        GPIO.output(LORA_RST_PIN, GPIO.HIGH)
        time.sleep(0.01)
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

    def set_frequency(self, freq_mhz):
        freq = int(freq_mhz * 16384)
        self.write_register(REG_FRF_MSB, (freq >> 16) & 0xFF)
        self.write_register(REG_FRF_MID, (freq >> 8) & 0xFF)
        self.write_register(REG_FRF_LSB, freq & 0xFF)

    def receive(self):
        # Wait for the IRQ pin to be HIGH indicating data is ready to read
        while GPIO.input(LORA_IRQ_PIN) == GPIO.LOW:
            time.sleep(0.01)  # Sleep a bit to avoid busy-waiting

        # Check IRQ flags to see if data is ready
        irq_flags = self.read_register(REG_IRQ_FLAGS)
        print(f"IRQ flags: {irq_flags}")  # Keep this for debugging

        if irq_flags & 0x40:  # RxDone flag
            # Set pointer FIFO to the received address
            self.write_register(REG_FIFO_ADDR_PTR, self.read_register(REG_FIFO_RX_BASE_ADDR))
            # Number of bytes received
            packet_length = self.read_register(REG_RX_NB_BYTES)
            print(f"Received packet length: {packet_length}")  # Add this line

            # Read data from FIFO
            message = []
            for _ in range(packet_length):
                message.append(self.read_register(REG_FIFO))
            # Clear RxDone flag
            self.write_register(REG_IRQ_FLAGS, 0xFF)
            return bytes(message).decode('utf-8', 'ignore')

        # Check for CRC errors
        if irq_flags & 0x20:  # Check for CRC error flag
            print("CRC error detected")

        return None

    def close(self):
        if self.spi:
            self.spi.close()
        GPIO.cleanup()

def main():
    try:
        # Buat instance LoRaReceiver
        lora = LoRaReceiver(verbose=True)
        # Inisialisasi LoRa
        lora.init()

        print("Menunggu pesan...")
        while True:
            # Terima pesan
            message = lora.receive()
            if message:
                print(f"Pesan diterima: {message}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh user")
    finally:
        print("\nMembersihkan...")
        lora.close()

if __name__ == "__main__":
    main()
