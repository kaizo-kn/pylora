import RPi.GPIO as GPIO
import spidev
import time
from datetime import datetime
import threading

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
REG_IRQ_FLAGS = 0x12
REG_FIFO_ADDR_PTR = 0x0D
REG_PAYLOAD_LENGTH = 0x22

# Mode operasi LoRa
MODE_LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_RX_CONTINUOUS = 0x05

class LoRaReceiver:
    def __init__(self):
        self.spi = None
        self.received_messages = []  # List untuk menyimpan semua pesan
        self.running = True

    def init(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(LORA_RST_PIN, GPIO.OUT)
        GPIO.setup(LORA_CS_PIN, GPIO.OUT)
        GPIO.setup(LORA_IRQ_PIN, GPIO.IN)

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 5000000
        self.spi.mode = 0

        self.reset()
        self.sleep()
        self.standby()
        self.set_frequency(433.0)

        print("LoRa Receiver berhasil diinisialisasi.")
        threading.Thread(target=self.listen_for_messages, daemon=True).start()

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

    def listen_for_messages(self):
        self.standby()
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)
        print("Menunggu pesan...")

        while self.running:
            if GPIO.input(LORA_IRQ_PIN) == 1:  # Check if IRQ is high (message received)
                self.handle_interrupt()

            time.sleep(0.1)  # Shorter sleep for faster response to interrupts

    def handle_interrupt(self):
        print("Interrupt terdeteksi!")

        # Clear interrupt flags and check the RxDone flag
        irq_flags = self.read_register(REG_IRQ_FLAGS)
        if irq_flags & 0x40:  # Check if RxDone flag is set
            # Clear all IRQ flags
            self.write_register(REG_IRQ_FLAGS, 0xFF)

            # Get the number of bytes received
            payload_length = self.read_register(0x13)  # REG_RX_NB_BYTES
            
            # Reset FIFO address pointer to the start of the message
            self.write_register(REG_FIFO_ADDR_PTR, 0)

            # Read the message from FIFO
            message = bytearray()
            for _ in range(payload_length):
                byte_received = self.read_register(REG_FIFO)
                message.append(byte_received)

            # Display the latest message only
            try:
                message_str = message.decode('utf-8', errors='strict')
            except UnicodeDecodeError:
                message_str = "<Invalid characters received>"

            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            print(f"[{timestamp}] Pesan diterima: {message_str} (Panjang: {len(message)} bytes)")

            # Optionally store the latest message only
            self.received_messages = [message]

        else:
            print("No valid message detected.")




    def close(self):
        self.running = False
        if self.spi:
            self.spi.close()
        GPIO.cleanup()

def main():
    lora = None
    try:
        lora = LoRaReceiver()
        lora.init()
        while True:
            time.sleep(1)  # Menjaga program tetap berjalan
    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh user")
    except Exception as e:
        print(f"Terjadi kesalahan: {str(e)}")
    finally:
        if lora:
            print("\nMembersihkan...")
            lora.close()

if __name__ == "__main__":
    main()