# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# ADS1115
# This code is designed to work with the ADS1115 I2C module with RaspberryPi
import smbus
import time

# Register and other configuration values:
ADS1x15_DEFAULT_ADDRESS        = 0x48
ADS1x15_POINTER_CONVERSION     = 0x00
ADS1x15_POINTER_CONFIG         = 0x01
ADS1x15_POINTER_LOW_THRESHOLD  = 0x02
ADS1x15_POINTER_HIGH_THRESHOLD = 0x03
ADS1x15_CONFIG_OS_SINGLE       = 0x8000
ADS1x15_CONFIG_MUX_OFFSET      = 12
# Maping of gain values to config register values.
ADS1x15_CONFIG_GAIN = {
    2/3: 0x0000,
    1:   0x0200,
    2:   0x0400,
    4:   0x0600,
    8:   0x0800,
    16:  0x0A00
}
ADS1x15_CONFIG_MODE_CONTINUOUS  = 0x0000
ADS1x15_CONFIG_MODE_SINGLE      = 0x0100
# Mapping of data/sample rate to config register values for ADS1015 (faster).
ADS1015_CONFIG_DR = {
    128:   0x0000,
    250:   0x0020,
    490:   0x0040,
    920:   0x0060,
    1600:  0x0080,
    2400:  0x00A0,
    3300:  0x00C0
}
# Mapping of data/sample rate to config register values for ADS1115 (slower).
ADS1115_CONFIG_DR = {
    8:    0x0000,
    16:   0x0020,
    32:   0x0040,
    64:   0x0060,
    128:  0x0080,
    250:  0x00A0,
    475:  0x00C0,
    860:  0x00E0
}
ADS1x15_CONFIG_COMP_WINDOW      = 0x0010
ADS1x15_CONFIG_COMP_ACTIVE_HIGH = 0x0008
ADS1x15_CONFIG_COMP_LATCHING    = 0x0004
ADS1x15_CONFIG_COMP_QUE = {
    1: 0x0000,
    2: 0x0001,
    4: 0x0002
}
ADS1x15_CONFIG_COMP_QUE_DISABLE = 0x0003

# ADS1115 address, 0x48
ADDR=ADS1x15_DEFAULT_ADDRESS

# Get I2C bus
bus = smbus.SMBus(1)

def ads_setup():
    # Select configuration register, 0x01
    #   0x8483   AINP = AIN0 and AINN = AIN1, +/- 2.048V
    #   Continuous conversion mode, 128SPS
    # b15=1     OS       1 : Start a single conversion (when in power-down state)
    # b14:12=000 MUX[2:0] AINP = AIN0 and AINN = AIN1
    # b11:9=010 PGA[2:0] FSR = +/- 2.048V (gain=2)
    # b8=0      MODE     0 : Continuous-conversion mode
    # b7:5=100  DR[2:0]  128 SPS
    # b4=0      COMP_MODE 0 : Traditional comparator (default)
    # b3=0      COMP_POL  0 : Active low (default)
    # b2=0      COMP_LAT  0 : Nonlatching comparator
    # b1:0=11   COMP_QUE  Disable comparator and set ALERT/RDY pin to high-impedance (default)
#    data = [0x84,0x83]
    data = [0xc2,0x83] #AINP = AIN0 and AIN N = GND; +/-4.096V
    bus.write_i2c_block_data(ADDR, ADS1x15_POINTER_CONFIG, data)

    time.sleep(0.5)

def ads_read():
    # Read data back from 0x00, 2 bytes
    # raw_adc MSB, raw_adc LSB
    data = bus.read_i2c_block_data(ADDR, ADS1x15_POINTER_CONVERSION, 2)

    # Convert the data
    raw_adc = data[0]*256 + data[1]

    if raw_adc > 32767:
        raw_adc -= 65535

    return raw_adc

def main():
    ads_setup()
    return ads_read()

if __name__ == "__main__":
    main()

