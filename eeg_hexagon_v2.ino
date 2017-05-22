#include <RF24_config.h>
#include <printf.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Wire.h>

byte nrf_cs = A0;
byte nrf_ce = A1;
byte mcp3912_cs = A2;
byte mcp3912_dr = A3;

byte led_bit = 1<<6;

void init_led()
{
  byte oldSREG = SREG;
  DDRB |= led_bit;
  SREG = oldSREG;  
}
void set_led(byte value)
{
  byte oldSREG = SREG;
  if(value) PORTB |= led_bit;
  else PORTB &= ~led_bit;
  SREG = oldSREG;  
}

#define MCP3912_CH0     0x00
#define MCP3912_CH1     0x01
#define MCP3912_CH2     0x02
#define MCP3912_CH3     0x03
#define MCP3912_MOD     0x08
#define MCP3912_PHASE   0x0A
#define MCP3912_GAIN    0x0B
#define MCP3912_STATCOM 0x0C
#define MCP3912_CONFIG0 0x0D
#define MCP3912_CONFIG1 0x0E
#define MCP3912_OFF0    0x0F
#define MCP3912_GC0     0x10
#define MCP3912_OFF1    0x11
#define MCP3912_GC1     0x12
#define MCP3912_OFF2    0x13
#define MCP3912_GC2     0x14
#define MCP3912_OFF3    0x15
#define MCP3912_GC3     0x16
#define MCP3912_LOCK    0x1F


/**
 * @brief CONFIG0 Register Bit-Field
 *
 * bit 23
 *      EN_OFFCAL: Enables the 24-bit digital offset error calibration.
 *          1 = Enabled. This mode does not add any group delay to the ADC data.
 *          0 = Disabled (DEFAULT)
 * bit 22
 *      EN_GAINCAL: Enables or disables the 24-bit digital gain error
 *                  calibration on all channels
 *          1 = Enabled. This mode adds a group delay on all channels of 24
 *              DMCLK periods. All data ready pulses are delayed by 24 DMCLK
 *              lock periods, compared to the mode with EN_GAINCAL = 0.
 *          0 = Disabled (DEFAULT)
 * bit 21-20
 *      DITHER<1:0>: Control for dithering circuit for idle tone?s cancellation
 *                  and improved THD on all channels
 *          11 = Dithering ON, Strength = Maximum (DEFAULT)
 *          10 = Dithering ON, Strength = Medium
 *          01 = Dithering ON, Strength = Minimum
 *          00 = Dithering turned OFF
 * bit 19-18
 *      BOOST<1:0>: Bias Current Selection for all ADCs (impacts achievable
 *                  maximum sampling speed, see Table 5-2 of MCP3912 Datasheet)
 *          11 = All channels have current x 2
 *          10 = All channels have current x 1 (Default)
 *          01 = All channels have current x 0.66
 *          00 = All channels have current x 0.5
 * bit 17-16
 *      PRE<1:0>: Analog Master Clock (AMCLK) Prescaler Value
 *          11 = AMCLK = MCLK/8
 *          10 = AMCLK = MCLK/4
 *          01 = AMCLK = MCLK/2
 *          00 = AMCLK = MCLK (Default)
 * bit 15-13
 *      OSR<2:0>: Oversampling Ratio for delta sigma A/D Conversion
 *              (ALL CHANNELS, fD/fS)
 *          111 = 4096 (fd = 244 sps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
 *          110 = 2048 (fd = 488 sps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
 *          101 = 1024 (fd = 976 sps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
 *          100 = 512 (fd = 1.953 ksps for MCLK = 4 MHz, fs = AMCLK = 1MHz)
 *          011 = 256 (fd = 3.90625 ksps for MCLK = 4 MHz, fs = AMCLK = 1 MHz) D
 *          010 = 128 (fd = 7.8125 ksps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
 *          001 = 64 (fd = 15.625 ksps for MCLK = 4 MHz, fs = AMCLK = 1MHz)
 *          000 = 32 (fd = 31.25 ksps for MCLK = 4 MHz, fs = AMCLK = 1MHz)
 * bit 12-8
 *      Unimplemented: Read as 0
 * bit 7-0
 *      VREFCAL<7:0>: Internal Voltage Temperature coefficient VREFCAL<7:0>
 *                    value. (See Section 5.6.3 ?Temperature compensation
 *                    (VREFCAL<7:0>)? in MCP documentation for complete
 *                    description).
 *
 */
typedef struct {

    union {
        uint32_t wholeRegister;

        struct {
            signed VREFCAL : 8;
            signed : 5;
            signed OSR : 3;
            signed PRE : 2;
            signed BOOST : 2;
            signed DITHER : 2;
            signed EN_GAINCAL : 1;
            signed EN_OFFCAL : 1;
            signed : 8;
        };
    };
} MCP391x_CONFIG0_REG;

MCP391x_CONFIG0_REG conf0_reg;


/**
 * @brief Status and Communication Register Bit-Field
 *
 * bit 23-22
 *      READ<1:0>: Address counter increment setting for Read Communication
 *          11 = Address counter auto-increments, and loops on the entire register map
 *          10 = Address counter auto-increments, and loops on register TYPES (DEFAULT)
 *          01 = Address counter auto-increments, and loops on register GROUPS
 *          00 = Address is not incremented, and continually reads the same
 *               single-register address
 * bit 21
 *      WRITE: Address counter increment setting for Write Communication
 *          1 = Address counter auto-increments and loops on writable part of
 *              the register map (DEFAULT)
 *          0 = Address is not incremented, and continually writes to the same
 *              single register address
 * bit 20
 *      DR_HIZ: Data Ready Pin Inactive State Control
 *          1 = The DR pin state is a logic high when data is NOT ready
 *          0 = The DR pin state is high-impedance when data is NOT ready (DEFAULT)
 * bit 19
 *      DR_LINK: Data Ready Link Control
 *          1 = Data Ready link enabled. Only one pulse is generated on the DR
 *              pin for all ADC channels corresponding to the data ready pulse
 *              of the most lagging ADC. (DEFAULT)
 *          0 = Data Ready link disabled. Each ADC produces its own data ready
 *              pulse on the DR pin.
 * bit 18
 *      WIDTH_CRC: Format for CRC-16 on communications
 *          1 = 32-bit (CRC-16 code is followed by zeros). This coding is
 *              compatible with CRC implementation in most 32-bit MCUs
 *              (including PIC32 MCUs).
 *          0 = 16 bit (default)
 * bit 17-16
 *      WIDTH_DATA<1:0>: ADC Data Format Settings for all ADCs
 *          11 = 32-bit with sign extension
 *          10 = 32-bit with zeros padding
 *          01 = 24-bit (default)
 *          00 = 16-bit (with rounding)
 * bit 15
 *      EN_CRCCOM: Enable CRC CRC-16 Checksum on Serial communications
 *          1 = CRC-16 Checksum is provided at the end of each communication
 *              sequence (therefore each communication is longer). The CRC-16
 *              Message is the complete communication sequence (see section
 *              Section 6.9).
 *          0 = Disabled
 * bit 14
 *      EN_INT: Enable for the CRCREG interrupt function
 *          1 = The interrupt flag for the CRCREG checksum verification is
 *              enabled. The Data Ready pin (DR) will become logic low and stays
 *               logic low if a CRCREG checksum error happens. This interrupt is
 *              cleared if the LOCK<7:0> value is made equal to the PASSWORD
 *              value (0xA5).
 *          0 = The interrupt flag for the CRCREG checksum verification is
 *              disabled. The CRCREG<15:0> bits are still calculated properly
 *              and can still be read in this mode. No interrupt is generated
 *              even when a CRCREG checksum error happens. (Default)
 * bit 13-12
 *      Reserved: Should be kept equal to 0 at all times
 * bit 11-4
 *      Unimplemented: Read as 0
 * bit 3-0
 *      DRSTATUS<3:0>: Data ready status bit for each individual ADC channel
 *          DRSTATUS<n> = 1 - Channel CHn data is not ready (DEFAULT)
 *          DRSTATUS<n> = 0 - Channel CHn data is ready. The status bit is set back
 *                    to '1' after reading the STATUSCOM register. The status
 *                    bit is not set back to '1' by the read of the corresponding
 *                    channel ADC data.
 *
 */
 
typedef struct {

    union {
        uint32_t wholeRegister;

        struct {
            unsigned DRSTATUS : 4;
            unsigned : 10;
            unsigned EN_INT : 1;
            unsigned EN_CRCCOM : 1;
            unsigned WIDTH_DATA : 2;
            unsigned WIDTH_CRC : 1;
            unsigned DR_LINK : 1;
            unsigned DR_HIZ : 1;
            unsigned WRITE : 1;
            unsigned READ : 2;
            unsigned : 8;
        };
    };
} MCP391x_STATUSCOM_REG;

MCP391x_STATUSCOM_REG statcom_reg;

/**
 * @brief CONFIG1 Address Register Bit-Field
 *
 * bit 23-20
 *      Unimplemented: Read as 0.
 * bit 19-16
 *      RESET<3:0>: Soft Reset mode setting for each individual ADC
 *          RESET<n> = 1 : Channel CHn in soft reset mode
 *          RESET<n> = 0 : Channel CHn not in soft reset mode
 * bit 15-12
 *      Unimplemented: Read as 0
 * bit 11-8
 *      SHUTDOWN<3:0>: Shutdown Mode setting for each individual ADC
 *          SHUTDOWN<n> = 1 : ADC Channel CHn in Shutdown
 *          SHUTDOWN<n> = 0 : ADC Channel CHn not in Shutdown
 * bit 7
 *      VREFEXT: Internal Voltage Reference selection bit
 *          1 = Internal Voltage Reference Disabled. An external reference
 *              voltage needs to be applied across the REFIN+/- pins. The analog
 *              power consumption (AI_DD) is slightly diminished in this mode
 *              since the internal voltage reference is placed in Shutdown mode.
 *          0 = Internal Reference enabled. For optimal accuracy, the REFIN+/OUT
 *              pin needs proper decoupling capacitors. REFIN- pin should be
 *              connected to A_GND, when in this mode.
 * bit 6
 *      CLKEXT: Internal Clock selection bit
 *          1 = MCLK is generated externally and should be provided on the OSC1
 *          pin. The oscillator is disabled and uses no current (Default)
 *          0 = Crystal oscillator enabled. A crystal must be placed between
 *              OSC1 and OSC2 with proper decoupling capacitors. The digital
 *              power consumption (DI_DD) is increased in this mode due to the
 *              oscillator.
 * bit 5-0
 *      Unimplemented: Read as 0
 *
 */
typedef struct {

    union {
        uint32_t wholeRegister;

        struct {
            signed : 6;
            signed CLKEXT : 1;
            signed VREFEXT : 1;
            signed SHUTDOWN : 4;
            signed : 4;
            signed RESET : 4;
            signed : 12;
        };
    };
} MCP391x_CONFIG1_REG;

MCP391x_CONFIG1_REG conf1_reg;

void mcp3912_write_reg(byte reg, long val24)
{
  digitalWrite(mcp3912_cs, 0);
  byte treg = ((0b01)<<6) | (reg<<1) | 0;
  byte transf[4];
  transf[0] = ((0b01)<<6) | (reg<<1) | 0;
  transf[1] = (val24>>16)&0xFF;
  transf[2] = (val24>>8)&0xFF;
  transf[3] = val24&0xFF;
  SPI.transfer(transf, 4);
//  SPI.transfer(treg);
//  SPI.transfer((val24>>16)&0xFF);
//  SPI.transfer((val24>>8)&0xFF);
//  SPI.transfer(val24&0xFF);
  digitalWrite(mcp3912_cs, 1);
}

long mcp3912_read_reg(byte reg)
{
  digitalWrite(mcp3912_cs, 0);
  byte treg = ((0b01)<<6) | (reg<<1) | 1;
//  SPI.transfer(treg);
  int vh = SPI.transfer16(treg<<8);
  int vl = SPI.transfer16(0);
//  byte b1 = 0;//SPI.transfer(0);
//  byte b2 = SPI.transfer(0);
//  byte b3 = SPI.transfer(0);
//  byte b4 = SPI.transfer(0);
  digitalWrite(mcp3912_cs, 1);
  return ((vh&0xFF)<<16) | vl;
//  return (b1<<24) | (b2<<16) | (b3<<8) | b4;
}

long mcp3912_read_reg32(byte reg)
{
  digitalWrite(mcp3912_cs, 0);
  byte treg = ((0b01)<<6) | (reg<<1) | 1;
  SPI.transfer(treg);
  int vh = SPI.transfer16(0);
  int vl = SPI.transfer16(0);
  digitalWrite(mcp3912_cs, 1);
  return (vh<<16) | vl;

/*  byte b1 = SPI.transfer(0);
  byte b2 = SPI.transfer(0);
  byte b3 = SPI.transfer(0);
  byte b4 = SPI.transfer(0);
  digitalWrite(mcp3912_cs, 1);
  return (b1<<24) | (b2<<16) | (b3<<8) | b4;*/
}

int mcp3912_read_reg16(byte reg)
{
  digitalWrite(mcp3912_cs, 0);
  byte treg = ((0b01)<<6) | (reg<<1) | 1;
  SPI.transfer(treg);
  int vv = SPI.transfer16(0);
  digitalWrite(mcp3912_cs, 1);
  return vv;
}

void mcp3912_init()
{
  pinMode(mcp3912_cs, OUTPUT);
  pinMode(mcp3912_dr, INPUT);
  digitalWrite(mcp3912_cs, 1);

  conf0_reg.BOOST = 0b11;
  conf0_reg.DITHER = 0b11;
  conf0_reg.EN_GAINCAL = 0;
  conf0_reg.EN_OFFCAL = 0;
  conf0_reg.OSR = 0b111; //OSR = 128
  // PRE 11:
  //111 -> 35 sps
  //110 -> 68 sps
  //101 -> 130 sps ZERO before reboot
  //100 -> 220 sps ZERO before reboot
  //011 -> 220 sps ZERO before reboot
  //010 -> 220 sps
  //001 -> 220 sps
  //000 -> 220 sps
  // PRE 00:
  //111 -> 220 sps
  //110 -> 240 sps
  conf0_reg.PRE = 0b00; // 0b11 = 8, 0b10 = 4, 0b01 = 2, 0 = 1
  conf0_reg.VREFCAL = 64;

  conf1_reg.CLKEXT = 0; //This should be set to zero for oscillator :: TESTING
  conf1_reg.RESET = 0b0000;
  conf1_reg.SHUTDOWN = 0b0000;
  conf1_reg.VREFEXT = 0;


  statcom_reg.WIDTH_DATA = 0b00; //16 bits
//  statcom_reg.WIDTH_DATA = 0b01; //24 bits
  statcom_reg.DR_HIZ = 1;
  statcom_reg.DR_LINK = 1;
  statcom_reg.EN_CRCCOM = 0;
  statcom_reg.EN_INT = 0;
  statcom_reg.READ = 0b11;
  statcom_reg.WIDTH_CRC = 0;
  statcom_reg.WRITE = 1;


  mcp3912_write_reg(MCP3912_STATCOM, statcom_reg.wholeRegister);
  mcp3912_write_reg(MCP3912_CONFIG0, conf0_reg.wholeRegister);
  mcp3912_write_reg(MCP3912_CONFIG1, conf1_reg.wholeRegister);
  
  return;
  

  long statcom, conf0, conf1, lock;
//  statcom = (0b10111000<<16) | (0b10<<8) | 0b0; //types group, auto inc write, dr high, dr linked, 16crc, 16 bit resolution
  statcom = (0b10111001<<16) | (0b0<<8) | 0b0; //types group, auto inc write, dr high, dr linked, 16crc, 16 bit resolution
//  conf0 = (0b00111110<<16) | (0b10100000<<8) | 0x50; //976 sps
  conf0 = (0b00111110<<16) | (0b10100000<<8) | 0x50; 
  conf1 = 0;
  lock = 0xA5<<16;
//  mcp3912_write_reg(MCP3912_LOCK, lock);
  mcp3912_write_reg(MCP3912_STATCOM, statcom);
  mcp3912_write_reg(MCP3912_CONFIG0, conf0);
  mcp3912_write_reg(MCP3912_CONFIG1, conf1);
}

int adc_ch0 = 0, adc_ch1 = 0, adc_ch2 = 0, adc_ch3 = 0;


void get_mcp_data()
{
  digitalWrite(mcp3912_cs, 0);
  byte treg = ((0b01)<<6) | (MCP3912_CH0<<1) | 1;
  SPI.transfer(treg);
  adc_ch0 = SPI.transfer16(0);
  adc_ch1 = SPI.transfer16(0);
  adc_ch2 = SPI.transfer16(0);
  adc_ch3 = SPI.transfer16(0);
  digitalWrite(mcp3912_cs, 1);
  return;
}

void get_mcp_data_t3()
{
  long v1 = mcp3912_read_reg16(MCP3912_CH0);
  long v2 = mcp3912_read_reg16(MCP3912_CH1);
  long v3 = mcp3912_read_reg16(MCP3912_CH2);
  long v4 = mcp3912_read_reg16(MCP3912_CH3);
  adc_ch0 = v1;//(v1>>8)&0xFFFF;
  adc_ch1 = v2;//(v2>>8)&0xFFFF;
  adc_ch2 = v3;//(v3>>8)&0xFFFF;
  adc_ch3 = v4;//(v4>>8)&0xFFFF;
}

void get_mcp_data_t2()
{
  long v1 = mcp3912_read_reg32(MCP3912_CH0);
  long v2 = mcp3912_read_reg32(MCP3912_CH1);
  long v3 = mcp3912_read_reg32(MCP3912_CH2);
  long v4 = mcp3912_read_reg32(MCP3912_CH3);
  adc_ch0 = v1;//(v1>>8)&0xFFFF;
  adc_ch1 = v2;//(v2>>8)&0xFFFF;
  adc_ch2 = v3;//(v3>>8)&0xFFFF;
  adc_ch3 = v4;//(v4>>8)&0xFFFF;
}

void get_mcp_data_t()
{
//  mcp3912_read_reg(MCP3912_STATCOM);
  digitalWrite(mcp3912_cs, 0);
  byte treg = ((0b01)<<6) | (MCP3912_CH0<<1) | 1;
  SPI.transfer(treg);
  byte b1 = SPI.transfer(0);
  byte b2 = SPI.transfer(0);
  byte b24 = 1;
  if(b24) SPI.transfer(0);
  adc_ch0 = (b1<<8) | b2;
  b1 = SPI.transfer(0);
  b2 = SPI.transfer(0);
  if(b24) SPI.transfer(0);
  adc_ch1 = (b1<<8) | b2;
  b1 = SPI.transfer(0);
  b2 = SPI.transfer(0);
  if(b24) SPI.transfer(0);
  adc_ch2 = (b1<<8) | b2;
  b1 = SPI.transfer(0);
  b2 = SPI.transfer(0);
  if(b24) SPI.transfer(0);
  adc_ch3 = (b1<<8) | b2;
  digitalWrite(mcp3912_cs, 1);
  return ;  
}

byte mcp_data_ready()
{
  long st = mcp3912_read_reg(MCP3912_STATCOM);
  byte str = st&0b01111;
  return (str == 0);
//  return (str != 0b1111);
}

/*================================
 *  MCP444X/446X Commands (USE IN GROUP) 
 *  7/8-Bit Quad I2C Digital POT with
 *  Nonvolatile Memory
 *================================
 */
#define   ADR_D_POT   0b0101111

#define   ADR_R     0b00001100
#define   ADR_W     0b00000000

#define   ADR_MEM_W0    0x02 // In EEPROM
#define   ADR_MEM_W1    0x03 // In EEPROM
#define   ADR_MEM_W3    0x08 // In EEPROM
#define   ADR_MEM_W4    0x09 // In EEPROM

#define   ADR_W0      0x00 // In RAM
#define   ADR_W1      0x01 // In RAM
#define   ADR_W2      0x06 // In RAM
#define   ADR_W3      0x07 // In RAM

#define   ADR_TCON0   0x04
#define   ADR_TCON1   0x0A

#define   ADR_STATUS    0x05

#define   REG_STATUS    0b0000010 // default all ON
/*                |||||||-WL3: 
1 = Wiper and TCON1 register bits R3HW, R3A, R3W, and R3B of Resistor Network 3 (Pot 3) are
“Locked” (Write Protected)
0 = Wiper and TCON1 of Resistor Network 3 (Pot 3) can be modified
 * 
                 ||||||--WL2: 
1 = Wiper and TCON1 register bits R2HW, R2A, R2W, and R2B of Resistor Network 2 (Pot 2) are
“Locked” (Write Protected)
0 = Wiper and TCON1 of Resistor Network 2 (Pot 2) can be modified
 * 
                |||||---EEWA: 
1 = An EEPROM Write cycle is currently occurring. Only serial commands to the Volatile memory
locations are allowed (addresses 00h, 01h, 04h, and 05h)
0 = An EEPROM Write cycle is NOT currently occurring
 * 
                 ||||----WL1: 
1 = Wiper and TCON0 register bits R1HW, R1A, R1W, and R1B of Resistor Network 1 (Pot 1) are
“Locked” (Write Protected)
0 = Wiper and TCON0 of Resistor Network 1 (Pot 1) can be modified
 * 
                  |||----WL0: 
1 = Wiper and TCON0 register bits R0HW, R0A, R0W, and R0B of Resistor Network 0 (Pot 0) are
“Locked” (Write Protected)
0 = Wiper and TCON0 of Resistor Network 0 (Pot 0) can be modified 
                   ||----Forced to “1”
                  |----WP:
1 = EEPROM memory is Write Protected
0 = EEPROM memory can be written
*/

#define   REG_TCON0   0b00000000  
/*                ||||||||-R1HW:0 - “shutdown” 
 *                |||||||--R1A: 1 - Connected
 *                ||||||---R1W: 1 - Connected
 *                |||||----R1B: 1 - Connected
 *                ||||-----R0HW:0 - “shutdown” 
 *                |||------R0A: 1 - Connected
 *                ||-------R0W: 1 - Connected
 *                |--------R0B: 1 - Connected
*/

#define   REG_TCON1   0b00000000  
/*                ||||||||-R3HW:0 - “shutdown” 
 *                |||||||--R3A: 1 - Connected
 *                ||||||---R3W: 1 - Connected
 *                |||||----R3B: 1 - Connected
 *                ||||-----R2HW:0 - “shutdown” 
 *                |||------R2A: 1 - Connected
 *                ||-------R2W: 1 - Connected
 *                |--------R2B: 1 - Connected
*/

int Init_D_POT_All()
{
  Wire.begin();
  
  Wire.beginTransmission(ADR_D_POT);
  Wire.write((ADR_STATUS<<4) | ADR_W);
  Wire.write(REG_STATUS);
  Wire.endTransmission();
  
  Wire.beginTransmission(ADR_D_POT);
  Wire.write((ADR_TCON0<<4) | ADR_W);
  Wire.write(REG_TCON0);
  Wire.endTransmission();
  
  Wire.beginTransmission(ADR_D_POT);
  Wire.write((ADR_TCON1<<4) | ADR_W);
  Wire.write(REG_TCON0);
  Wire.endTransmission();
  
  return 0;
}

float Set_D_POT_kOhm(int num, float res)
{
  int r = res*127/50;
  byte r_hb = (r&(1<<8)) != 0;
  if (num == 0)
  {
    Wire.beginTransmission( ADR_D_POT );
    Wire.write( (ADR_W0<<4) | ADR_W | r_hb);
    Wire.write( r&0xFF );
    Wire.endTransmission();
  }
  else if (num == 1)
  {
    Wire.beginTransmission( ADR_D_POT );
    Wire.write( (ADR_W1<<4) | ADR_W | r_hb);
    Wire.write( r&0xFF );
    Wire.endTransmission();
  }
  else if (num == 2)
  {
    Wire.beginTransmission( ADR_D_POT );
    Wire.write( (ADR_W2<<4) | ADR_W | r_hb );
    Wire.write( r&0xFF );
    Wire.endTransmission();
  }
  else if (num == 3)
  {
    Wire.beginTransmission( ADR_D_POT );
    Wire.write( (ADR_W3<<4) | ADR_W | r_hb );
    Wire.write( r&0xFF );
    Wire.endTransmission();
  }
  else return -1;
    
  return (float)r*50.0/127.0;
}


float MEM_D_POT_kOhm(int num)
{  
  if (num == 0)
  {
    Wire.beginTransmission( ADR_D_POT );
    Wire.write((ADR_W0<<4) | ADR_R);
  }
  else if (num == 1)
  {
    Wire.beginTransmission( ADR_D_POT );
    Wire.write((ADR_W1<<4) | ADR_R);
  }
  else if (num == 2)
  {
    Wire.beginTransmission( ADR_D_POT );
    Wire.write((ADR_W2<<4) | ADR_R);
  }
  else if (num == 3)
  {
    Wire.beginTransmission( ADR_D_POT );
    Wire.write((ADR_W3<<4) | ADR_R);
  }
  else return -1;
  Wire.endTransmission();
  
  Wire.requestFrom(ADR_D_POT, 2);

  int data = 0;
  if (Wire.available()) 
    data = Wire.read();  
  if (Wire.available()) 
    data = (data << 8) | Wire.read();
  else return -1;
  
  return (float)data*50.0/127.0;
}

RF24 rf(nrf_ce, nrf_cs);

void setup() {

//  SPI.begin();
//  SPI.beginTransaction(SPISettings(1000, MSBFIRST, SPI_MODE0));
  
  init_led();
  set_led(1);
  delay(100);
  set_led(0);
  delay(500);

  Init_D_POT_All();
  Set_D_POT_kOhm(0, 0);
  Set_D_POT_kOhm(1, 0);
  Set_D_POT_kOhm(2, 0);
  Set_D_POT_kOhm(3, 0);  

  while(!rf.begin())
  {
    for(int x = 0; x < 2; x++)
    {
      set_led(1);
      delay(500);
      set_led(0);
      delay(500);    
    }
//    delay(2000);
  }
//  pinMode(pulse_led, OUTPUT);
  if(!rf.setDataRate(RF24_1MBPS))
  {
    for(int x = 0; x < 5; x++)
    {
      set_led(1);
      delay(500);
      set_led(0);
      delay(500);    
    }
    delay(2000);
  }

  rf.setAddressWidth(3);
//  rf.enableDynamicPayloads();
  rf.setChannel(53);
  rf.setRetries(0, 0);
  uint8_t pipe_tx[8] = {'e', 'e', 't', 'x', '0', '0', '0', '0'};
//  byte pipe[8] = "0000000";
  rf.setPALevel(RF24_PA_MAX);
  rf.setCRCLength(RF24_CRC_DISABLED);
  rf.openWritingPipe(pipe_tx);

  mcp3912_init();
  // put your setup code here, to run once:

}

byte out_pack[32];
byte op = 0;
byte cnt = 0;

void difpack_16_to_14(int prev_val, int pack_val, byte *bh, byte *bl)
{
  int dif = pack_val - prev_val;
  byte sign = 0;
  if(dif < 0)
  {
    dif = -dif;
    sign = 1;
  }
  byte dl = dif&0xFF;
  byte dh = dif>>8;
  if((dh>>4) == 0) //from -4096 to 4096
    dh = dh&0b01111 | (sign<<5);
  else
  {
    int dif_s = dif>>2;
    dl = dif_s&0xFF;
    dh = dif_s>>8;
    dh = dh | (sign<<5);
  }
  dh = dh&0b00111111;
  *bh = dh;
  *bl = dl;
}

int skip_cnt = 0;

void loop() {
  cnt++;
  byte data_cnt = 0;
  int prev_adc_0, prev_adc_1, prev_adc_2, prev_adc_3;
  while(data_cnt < 4)
  {
/*    long mcs = micros();
    while(0)
    {
      if(micros() > mcs + 50) break;
      if(micros() - 50 > mcs) break;
    }*/
//    delay(1);
    while(mcp_data_ready() < 1) ;
//    if(data_cnt != 3)
//      rf.write(out_pack, 32);
    prev_adc_0 = adc_ch0;
    prev_adc_1 = adc_ch1;
    prev_adc_2 = adc_ch2;
    prev_adc_3 = adc_ch3;
    get_mcp_data();
    skip_cnt++;
//    if(skip_cnt < 3) continue;
    skip_cnt = 0;
    byte n = data_cnt;
    data_cnt++;
    if(data_cnt == 2)
    {
      set_led(cnt > 128);
    }
    if(n == 3)
    {
      difpack_16_to_14(prev_adc_0, adc_ch0, out_pack+n*8, out_pack+n*8+1);
      difpack_16_to_14(prev_adc_1, adc_ch1, out_pack+n*8+2, out_pack+n*8+3);
      difpack_16_to_14(prev_adc_2, adc_ch2, out_pack+n*8+4, out_pack+n*8+5);
      difpack_16_to_14(prev_adc_3, adc_ch3, out_pack+n*8+6, out_pack+n*8+7);
      byte control_byte = cnt;
      out_pack[n*8] |= control_byte&0b11000000;
      out_pack[n*8+2] |= (control_byte<<2)&0b11000000;
      out_pack[n*8+4] |= (control_byte<<4)&0b11000000;
      out_pack[n*8+6] |= (control_byte<<6)&0b11000000;
    }
    else
    {
      out_pack[n*8+0] = adc_ch0>>8;
      out_pack[n*8+1] = adc_ch0&0xFF;
      out_pack[n*8+2] = adc_ch1>>8;
      out_pack[n*8+3] = adc_ch1&0xFF;
      out_pack[n*8+4] = adc_ch2>>8;
      out_pack[n*8+5] = adc_ch2&0xFF;
      out_pack[n*8+6] = adc_ch3>>8;
      out_pack[n*8+7] = adc_ch3&0xFF;
    }
  }
/*  for(int n = 0; n < 4; n++)
  {
    delay(1);
    get_mcp_data();
    out_pack[n*8+0] = adc_ch0>>8;
    out_pack[n*8+1] = adc_ch0&0xFF;
    out_pack[n*8+2] = adc_ch1>>8;
    out_pack[n*8+3] = adc_ch1&0xFF;
    out_pack[n*8+4] = adc_ch2>>8;
    out_pack[n*8+5] = adc_ch2&0xFF;
    out_pack[n*8+6] = adc_ch3>>8;
    out_pack[n*8+7] = adc_ch3&0xFF;
  }*/
  rf.write(out_pack, 32);
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
//  set_led(0);
//  delay(5);
//  set_led(1);
//  delay(2);
}
