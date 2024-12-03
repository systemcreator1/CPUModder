import os
import subprocess
import RPi.GPIO as GPIO
import time

# GPIO Pin Assignments for SPI
SPI_MOSI = 10  # GPIO10
SPI_MISO = 9   # GPIO9
SPI_SCLK = 11  # GPIO11
SPI_CS = 8     # GPIO8 (Chip Select or Reset)

# CPU Configuration
CPU_TYPE = "cortex-m3"  # Modify based on your CPU
SOURCE_FILE = "source.c"  # High-level source code file
OUTPUT_BINARY = "firmware.bin"  # Compiled binary
CLOCK_SPEED = 0.001  # Adjust SPI clock speed (in seconds)

# GPIO Setup
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_CS], GPIO.OUT)
    GPIO.output(SPI_CS, GPIO.HIGH)  # Default CS High

# Compile Source Code to Binary
def compile_code(source_file, output_binary, cpu_type="cortex-m3"):
    """
    Compiles source code into a machine-readable binary for the target CPU.
    """
    try:
        # GCC Command
        gcc_cmd = [
            "arm-none-eabi-gcc",
            "-mcpu=" + cpu_type,
            "-mthumb",
            "-o", "firmware.elf",
            source_file
        ]
        # Objcopy Command
        objcopy_cmd = [
            "arm-none-eabi-objcopy",
            "-O", "binary",
            "firmware.elf",
            output_binary
        ]
        
        print("Compiling source code...")
        subprocess.run(gcc_cmd, check=True)
        print("Generating binary...")
        subprocess.run(objcopy_cmd, check=True)
        print(f"Binary generated: {output_binary}")
    except subprocess.CalledProcessError as e:
        print("Compilation error:", e)
        exit(1)

# Flash Binary via SPI
def spi_write(data):
    """
    Sends data to the target CPU using SPI protocol.
    """
    GPIO.output(SPI_CS, GPIO.LOW)  # Select the chip
    for byte in data:
        for bit in range(8):
            GPIO.output(SPI_MOSI, (byte >> (7 - bit)) & 1)
            GPIO.output(SPI_SCLK, GPIO.HIGH)
            time.sleep(CLOCK_SPEED)  # SPI clock speed
            GPIO.output(SPI_SCLK, GPIO.LOW)
    GPIO.output(SPI_CS, GPIO.HIGH)  # Deselect the chip

def flash_firmware(firmware_path):
    """
    Reads and flashes the firmware binary to the CPU.
    """
    try:
        print("Loading firmware...")
        with open(firmware_path, "rb") as firmware:
            binary_data = firmware.read()
        print(f"Flashing {len(binary_data)} bytes...")
        spi_write(binary_data)
        print("Firmware flashed successfully.")
    except Exception as e:
        print("Error during flashing:", e)

# All-in-One Function
def reprogram_cpu(source_file, output_binary, cpu_type="cortex-m3"):
    """
    Complete process: Compile code and flash firmware to the CPU.
    """
    try:
        setup_gpio()
        print("Step 1: Compiling Code...")
        compile_code(source_file, output_binary, cpu_type)
        print("Step 2: Flashing Firmware...")
        flash_firmware(output_binary)
        print("CPU reprogramming complete.")
    finally:
        GPIO.cleanup()

# Main Execution
if __name__ == "__main__":
    if not os.path.exists(SOURCE_FILE):
        print(f"Error: Source file {SOURCE_FILE} not found.")
    else:
        reprogram_cpu(SOURCE_FILE, OUTPUT_BINARY, CPU_TYPE)
