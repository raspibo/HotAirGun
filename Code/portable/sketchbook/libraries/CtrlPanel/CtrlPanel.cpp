
#include "CtrlPanel.h"

  /*
     Libreary result of remix of:
     -  CtrlPanelLib High Performance i2c LCD driver for MCP23008 & MCP23017
     hacked by Sam C. Lin / http://www.lincomatic.com
     from
     LiquidTWI by Matt Falcon (FalconFour) / http://falconfour.com
     logic gleaned from Adafruit RGB LCD Shield library
     Panelolu2 support by Tony Lock / http://blog.think3dprint3d.com
     enhancements by Nick Sayer / https://github.com/nsayer


     - Function add for a specific controller Raspibo

    Work in progress for a better solution in open source code
  */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#if defined (__AVR_ATtiny84__) || defined(__AVR_ATtiny85__) || (__AVR_ATtiny2313__)
#include "TinyWireM.h"
#define Wire TinyWireM
#else
#include <Wire.h>
#endif
#if defined(ARDUINO) && (ARDUINO >= 100) //scl
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


  //MCP23017 - Adafruit RGB LCD Shield
  // bit pattern for the burstBits function is
  //
  //  B7 B6 B5 B4 B3 B2 B1 B0 A7 A6 A5 A4 A3 A2 A1 A0 - MCP23017
  //  RS RW EN D4 D5 D6 D7 BL Io2Io1P3 P2 P1 EB EA EC
  //  15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
#define M17_BIT_RS 0x8000
#define M17_BIT_RW 0x4000
#define M17_BIT_EN 0x2000
#define M17_BIT_D4 0x1000
#define M17_BIT_D5 0x0800
#define M17_BIT_D6 0x0400
#define M17_BIT_D7 0x0200
#define M17_BIT_BL 0x0100
#define M17_BIT_P5 0x0080
#define M17_BIT_P4 0x0040		//Optional Led5 Or Buzzer
#define M17_BIT_P3 0x0020
#define M17_BIT_P2 0x0010		//Optional Led4
#define M17_BIT_P1 0x0008
#define M17_BIT_EB 0x0004		//Optional Led3
#define M17_BIT_EA 0x0002		//Optional Led2
#define M17_BIT_EC 0x0001		//Optional Led1

  static inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
    Wire.write((uint8_t)x);
#else
    Wire.send(x);
#endif
  }

  static inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
    return Wire.read();
#else
    return Wire.receive();
#endif
  }



  // When the display powers up, it is configured as follows:
  //
  // 1. Display clear
  // 2. Function set:
  //    DL = 0; 4-bit interface data
  //    N = 0; 1-line display
  //    F = 0; 5x8 dot character font
  // 3. Display on/off control:
  //    D = 0; Display off
  //    C = 0; Cursor off
  //    B = 0; Blinking off
  // 4. Entry mode set:
  //    I/D = 	1; Increment by 1
  //    S = 0; No shift
  //
  // Note, however, that resetting the Arduino doesn't reset the LCD, so we
  // can't assume that its in that state when a sketch starts (and the
  // CtrlPanelLib constructor is called). This is why we save the init commands
  // for when the sketch calls begin(), except configuring the expander, which
  // is required by any setup.

  CtrlPanelLib::CtrlPanelLib(uint8_t i2cAddr, uint8_t detectDevice, uint8_t backlightInverted) {
    // if detectDevice != 0, set _deviceDetected to 2 to flag that we should
    // scan for it in begin()
#ifdef DETECT_DEVICE
    _deviceDetected = detectDevice ? 2 : 1;
#endif
    _backlightInverted = backlightInverted;
    //  if (i2cAddr > 7) i2cAddr = 7;
    _i2cAddr = i2cAddr; // transfer this function call's number into our internal class state
    _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS; // in case they forget to call begin() at least we have something
    
  }

  void CtrlPanelLib::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
    uint8_t RegValue = 0;
    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
    delay(50);
    Wire.begin();
    uint8_t result;

#ifdef PANEL_CFG_ENC									// now set up input/output pins
    RegValue = RegValue | 0x07;
#endif
#ifdef PANEL_CFG_5Key
    RegValue = RegValue | 0xF8;
#else //ifdef PANEL_CFG_BUZZ
    RegValue = RegValue | 0xA8 ;
#endif
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_IODIRA);
    wiresend(RegValue);
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
      if (_deviceDetected == 2) {
        _deviceDetected = 0;
        return;
      }
    }
#endif

    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);	// set the button pullups
    wiresend(MCP23017_GPPUA);
    wiresend(RegValue);
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
      if (_deviceDetected == 2) {
        _deviceDetected = 0;
        return;
      }
    }
#endif

    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);	//Enable Interrupt
    wiresend(MCP23017_GPINTENA);
    wiresend(RegValue);
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
      if (_deviceDetected == 2) {
        _deviceDetected = 0;
        return;
      }
    }
#endif

    RegValue = 0;

#ifdef PANEL_CFG_ENC									// Setup Imterrupt MODE 1 Edge detect 0 On chnge state 
    RegValue = RegValue | 0x01;
#endif
#ifdef PANEL_CFG_5Key
    RegValue = RegValue | 0xF8;
#else //ifdef PANEL_CFG_BUZZ
    RegValue = RegValue | 0xA8 ;
#endif
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_INTCONA);
    wiresend(RegValue);
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
      if (_deviceDetected == 2) {
        _deviceDetected = 0;
        return;
      }
    }
#endif

    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);	//Register value for interrupt on edge detect
    wiresend(MCP23017_DEFVALA);
    wiresend(RegValue);
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
      if (_deviceDetected == 2) {
        _deviceDetected = 0;
        return;
      }
    }
#endif

    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);	//Register value for interrupt on edge detect
    wiresend(MCP23017_IOCONA);
    wiresend(0x40);
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
      if (_deviceDetected == 2) {
        _deviceDetected = 0;
        return;
      }
    }
#endif
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);	//Register value for interrupt on edge detect
    wiresend(MCP23017_IOCONB);
    wiresend(0x40);
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
      if (_deviceDetected == 2) {
        _deviceDetected = 0;
        return;
      }
    }
#endif

    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_IODIRB);
    wiresend(0x00); // all pins output
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
      if (_deviceDetected == 2) {
        _deviceDetected = 0;
        return;
      }
    }
#endif

#ifdef DETECT_DEVICE
    // If we haven't failed by now, then we pass
    if (_deviceDetected == 2) _deviceDetected = 1;
#endif

    if (lines > 1) {
      _displayfunction |= LCD_2LINE;
    }
    _numlines = lines;
    _currline = 0;

    // for some 1 line displays you can select a 10 pixel high font
    if ((dotsize != 0) && (lines == 1)) {
      _displayfunction |= LCD_5x10DOTS;
    }

    //put the LCD into 4 bit mode
    // start with a non-standard command to make it realize we're speaking 4-bit here
    // per LCD datasheet, first command is a single 4-bit burst, 0011.
    //-----
    //  we cannot assume that the LCD panel is powered at the same time as
    //  the arduino, so we have to perform a software reset as per page 45
    //  of the HD44780 datasheet - (kch)
    //-----
    _backlightBits = 0; // all backlight LED's on


    for (uint8_t i = 0; i < 3; i++) {
      burstBits8b((M17_BIT_EN | M17_BIT_D5 | M17_BIT_D4) >> 8);
      burstBits8b((M17_BIT_D5 | M17_BIT_D4) >> 8);
    }
    burstBits8b((M17_BIT_EN | M17_BIT_D5) >> 8);
    burstBits8b(M17_BIT_D5 >> 8);

    delay(5); // this shouldn't be necessary, but sometimes 16MHz is stupid-fast.

    command(LCD_FUNCTIONSET | _displayfunction); // then send 0010NF00 (N=lines, F=font)
    delay(5); // for safe keeping...
    command(LCD_FUNCTIONSET | _displayfunction); // ... twice.
    delay(5); // done!

    // turn on the LCD with our defaults. since these libs seem to use personal preference, I like a cursor.
    _displaycontrol = (LCD_DISPLAYON | LCD_BACKLIGHT);
    display();
    // clear it off
    clear();

    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    // set the entry mode
    command(LCD_ENTRYMODESET | _displaymode);
  }

  /********** high level commands, for the user! */
  void CtrlPanelLib::clear()
  {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
    delayMicroseconds(2000);  // this command takes a long time!
  }

  void CtrlPanelLib::home()
  {
    command(LCD_RETURNHOME);  // set cursor position to zero
    delayMicroseconds(2000);  // this command takes a long time!
  }

  void CtrlPanelLib::setCursor(uint8_t col, uint8_t row)
  {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if ( row > _numlines ) row = _numlines - 1;    // we count rows starting w/0
    command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
  }

  // Turn the display on/off (quickly)
  void CtrlPanelLib::noDisplay() {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    _displaycontrol &= ~LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
  }
  void CtrlPanelLib::display() {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    _displaycontrol |= LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
  }

  // Turns the underline cursor on/off
  void CtrlPanelLib::noCursor() {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    _displaycontrol &= ~LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
  }
  void CtrlPanelLib::cursor() {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    _displaycontrol |= LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
  }

  // Turn on and off the blinking cursor
  void CtrlPanelLib::noBlink() {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    _displaycontrol &= ~LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
  }
  void CtrlPanelLib::blink() {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    _displaycontrol |= LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
  }

  // These commands scroll the display without changing the RAM
  void CtrlPanelLib::scrollDisplayLeft(void) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
  }
  void CtrlPanelLib::scrollDisplayRight(void) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
  }

  // This is for text that flows Left to Right
  void CtrlPanelLib::leftToRight(void) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    _displaymode |= LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | _displaymode);
  }

  // This is for text that flows Right to Left
  void CtrlPanelLib::rightToLeft(void) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    _displaymode &= ~LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | _displaymode);
  }

  // This will 'right justify' text from the cursor
  void CtrlPanelLib::autoscroll(void) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    _displaymode |= LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | _displaymode);
  }

  // This will 'left justify' text from the cursor
  void CtrlPanelLib::noAutoscroll(void) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | _displaymode);
  }

  // Allows us to fill the first 8 CGRAM locations
  // with custom characters
  void CtrlPanelLib::createChar(uint8_t location, uint8_t charmap[]) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    location &= 0x7; // we only have 8 locations 0-7
    command(LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 8; i++) {
      write(charmap[i]);
    }
  }

  /*********** mid level commands, for sending data/cmds */
  inline void CtrlPanelLib::command(uint8_t value) {
    send(value, LOW);
  }
#if defined(ARDUINO) && (ARDUINO >= 100) //scl
  inline size_t CtrlPanelLib::write(uint8_t value) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return 1;
#endif
    send(value, HIGH);
    return 1;
  }
#else
  inline void CtrlPanelLib::write(uint8_t value) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    send(value, HIGH);
  }
#endif

  /************ low level data pushing commands **********/
  //#ifdef MCP23017
  uint8_t CtrlPanelLib::readButtons(void) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return 0;
#endif
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_GPIOA);
    Wire.endTransmission();

    Wire.requestFrom(MCP23017_ADDRESS | _i2cAddr, 1);
    return ~wirerecv() & ALL_BUTTON_BITS;
  }
  //#endif // MCP23017

  // Allows to set the backlight, if the LCD backpack is used
  void CtrlPanelLib::setBacklight(uint8_t status) {
#ifdef DETECT_DEVICE
    if (!_deviceDetected) return;
#endif
    if (_backlightInverted) status = !status;
    _backlightBits = M17_BIT_BL;
    if (!status ) _backlightBits &= ~M17_BIT_BL;
    burstBits16(_backlightBits);
  }

  // write either command or data, burst it to the expander over I2C.
  void CtrlPanelLib::send(uint8_t value, uint8_t mode) {
    // BURST SPEED, OH MY GOD
    // the (now High Speed!) I/O expander pinout
    //  B7 B6 B5 B4 B3 B2 B1 B0 A7 A6 A5 A4 A3 A2 A1 A0 - MCP23017
    //  15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
    //  RS RW EN D4 D5 D6 D7 BL P5 P4 P3 P3 P1 EC EB EA

    // n.b. RW bit stays LOW to write
    uint8_t buf = _backlightBits >> 8;
    // send high 4 bits
    if (value & 0x10) buf |= M17_BIT_D4 >> 8;
    if (value & 0x20) buf |= M17_BIT_D5 >> 8;
    if (value & 0x40) buf |= M17_BIT_D6 >> 8;
    if (value & 0x80) buf |= M17_BIT_D7 >> 8;

    if (mode) buf |= (M17_BIT_RS | M17_BIT_EN) >> 8; // RS+EN
    else buf |= M17_BIT_EN >> 8; // EN

    burstBits8b(buf);

    // resend w/ EN turned off
    buf &= ~(M17_BIT_EN >> 8);
    burstBits8b(buf);

    // send low 4 bits
    buf = _backlightBits >> 8;
    // send high 4 bits
    if (value & 0x01) buf |= M17_BIT_D4 >> 8;
    if (value & 0x02) buf |= M17_BIT_D5 >> 8;
    if (value & 0x04) buf |= M17_BIT_D6 >> 8;
    if (value & 0x08) buf |= M17_BIT_D7 >> 8;

    if (mode) buf |= (M17_BIT_RS | M17_BIT_EN) >> 8; // RS+EN
    else buf |= M17_BIT_EN >> 8; // EN

    burstBits8b(buf);

    // resend w/ EN turned off
    buf &= ~(M17_BIT_EN >> 8);
    burstBits8b(buf);

  }


  // value byte order is BA
  void CtrlPanelLib::burstBits16(uint16_t value) {
    // we use this to burst bits to the GPIO chip whenever we need to. avoids repetitive code.
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_GPIOA);
    wiresend(value & 0xFF); // send A bits
    wiresend(value >> 8);   // send B bits
    while (Wire.endTransmission());
  }

  /*
     void CtrlPanelLib::burstBits8a(uint8_t value) {
    // we use this to burst bits to the GPIO chip whenever we need to. avoids repetitive code.
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_GPIOA);
    wiresend(value); // last bits are crunched, we're done.
    while(Wire.endTransmission());
    }
  */
  void CtrlPanelLib::burstBits8b(uint8_t value) {
    // we use this to burst bits to the GPIO chip whenever we need to. avoids repetitive code.
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_GPIOB);
    wiresend(value); // last bits are crunched, we're done.
    while (Wire.endTransmission());
  }

  //direct access to the registers for interrupt setting and reading, also the tone function using buzzer pin
  uint8_t CtrlPanelLib::readRegister(uint8_t reg) {
    // read a register
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(reg);
    Wire.endTransmission();

    Wire.requestFrom(MCP23017_ADDRESS | _i2cAddr, 1);
    return wirerecv();
  }

  //set registers
  void CtrlPanelLib::setRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(reg);
    wiresend(value);
    Wire.endTransmission();
  }

  void CtrlPanelLib::WriteLed(uint8_t Led, uint8_t value){
    uint8_t RegValue;
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_GPIOA);
    Wire.endTransmission();
    Wire.requestFrom(MCP23017_ADDRESS | _i2cAddr, 1);
    RegValue = wirerecv();
    bitWrite(RegValue, Led, value);
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_GPIOA);
    wiresend(RegValue);
    Wire.endTransmission();
 }

 
  // void CtrlPanelLib::digitalWrite(uint8_t pin, uint8_t d) {
    // uint8_t gpio;
    // uint8_t bit = bitForPin(pin);


    // // read the current GPIO output latches
    // uint8_t regAddr = regForPin(pin, MCP23017_OLATA, MCP23017_OLATB);
    // gpio = readRegister(regAddr);

    // // set the pin and direction
    // bitWrite(gpio, bit, d);

    // // write the new GPIO
    // regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
    // setRegister(regAddr, gpio);
  // }
 
 
 
//#ifdef BUZZ_A0 || BUZZ_A4 
  //cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms)
  //note: a 100Khz TWI/I2C bus on a 16Mhz Arduino will max out at around 1500Hz freq
  void CtrlPanelLib::buzz(long duration, uint16_t freq) {
    int currentRegister = 0;

    // read gpio register
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_GPIOA);
    Wire.endTransmission();
    Wire.requestFrom(MCP23017_ADDRESS | _i2cAddr, 1);
    currentRegister = wirerecv();

    duration *= 1000; //convert from ms to us
    unsigned long cycletime = 1000000UL / freq; // period in us
    unsigned long cycles = (unsigned long)duration / cycletime;
    unsigned long ontime;
    while (cycles-- > 0)
    {
      ontime = micros();
      Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
      wiresend(MCP23017_GPIOA);
      wiresend(currentRegister |= Buzzer);
      while (Wire.endTransmission());
      while ((long)(ontime + (cycletime / 2) - micros()) > 0);
      Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
      wiresend(MCP23017_GPIOA);
      wiresend(currentRegister &= ~Buzzer);
      while (Wire.endTransmission());
      while ((long)(ontime + cycletime - micros()) > 0);
    }
  }

//#endif


  /**
     Helper to update a single bit of an A/B register.
     - Reads the current register value
     - Writes the new register value
  */
  void CtrlPanelLib::updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
    uint8_t regValue;
    uint8_t regAddr = regForPin(pin, portAaddr, portBaddr);
    uint8_t bit = bitForPin(pin);
    regValue = readRegister(regAddr);

    // set the value for the particular bit
    bitWrite(regValue, bit, pValue);

    setRegister(regAddr, regValue);
  }

  /**
   *  * Bit number associated to a give Pin
   *   */
  uint8_t CtrlPanelLib::bitForPin(uint8_t pin) {
    return pin % 8;
  }

  /**
   *  * Register address, port dependent, for a given PIN
   *   */
  uint8_t CtrlPanelLib::regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr) {
    return (pin < 8) ? portAaddr : portBaddr;
  }

  /**
   *  * Sets the pin mode to either INPUT or OUTPUT
   *   */
  void CtrlPanelLib::pinMode(uint8_t p, uint8_t d) {
    updateRegisterBit(p, (d == INPUT), MCP23017_IODIRA, MCP23017_IODIRB);
  }

  void CtrlPanelLib::digitalWrite(uint8_t pin, uint8_t d) {
    uint8_t gpio;
    uint8_t bit = bitForPin(pin);


    // read the current GPIO output latches
    uint8_t regAddr = regForPin(pin, MCP23017_OLATA, MCP23017_OLATB);
    gpio = readRegister(regAddr);

    // set the pin and direction
    bitWrite(gpio, bit, d);

    // write the new GPIO
    regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
    setRegister(regAddr, gpio);
  }

  void CtrlPanelLib::pullUp(uint8_t p, uint8_t d) {
    updateRegisterBit(p, d, MCP23017_GPPUA, MCP23017_GPPUB);
  }

  uint8_t CtrlPanelLib::digitalRead(uint8_t pin) {
    uint8_t bit = bitForPin(pin);
    uint8_t regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
    return (readRegister(regAddr) >> bit) & 0x1;
  }

  /**
     Configures the interrupt system. both port A and B are assigned the same configuration.
     Mirroring will OR both INTA and INTB pins.
     Opendrain will set the INT pin to value or open drain.
     polarity will set LOW or HIGH on interrupt.
     Default values after Power On Reset are: (false,flase, LOW)
     If you are connecting the INTA/B pin to arduino 2/3, you should configure the interupt handling as FALLING with
     the default configuration.
  */
  void CtrlPanelLib::setupInterrupts(uint8_t mirroring, uint8_t openDrain, uint8_t polarity) {
    // configure the port A
    uint8_t ioconfValue = readRegister(MCP23017_IOCONA);
    bitWrite(ioconfValue, 6, mirroring);
    bitWrite(ioconfValue, 2, openDrain);
    bitWrite(ioconfValue, 1, polarity);
    setRegister(MCP23017_IOCONA, ioconfValue);

    // Configure the port B
    ioconfValue = readRegister(MCP23017_IOCONB);
    bitWrite(ioconfValue, 6, mirroring);
    bitWrite(ioconfValue, 2, openDrain);
    bitWrite(ioconfValue, 1, polarity);
    setRegister(MCP23017_IOCONB, ioconfValue);
  }

  /**
     Set's up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.

     Note that the interrupt condition finishes when you read the information about the port / value
     that caused the interrupt or you read the port itself. Check the datasheet can be confusing.

  */
  void CtrlPanelLib::setupInterruptPin(uint8_t pin, uint8_t mode) {

    // set the pin interrupt control (0 means change, 1 means compare against given value);
    updateRegisterBit(pin, (mode != CHANGE), MCP23017_INTCONA, MCP23017_INTCONB);
    // if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt

    // In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
    // In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
    updateRegisterBit(pin, (mode == FALLING), MCP23017_DEFVALA, MCP23017_DEFVALB);

    // enable the pin for interrupt
    updateRegisterBit(pin, HIGH, MCP23017_GPINTENA, MCP23017_GPINTENB);

  }

  uint8_t CtrlPanelLib::getLastInterruptPin() {
    uint8_t intf;

    // try port A
    intf = readRegister(MCP23017_INTFA);
    for (int i = 0; i < 8; i++) if (bitRead(intf, i)) return i;

    // try port B
    intf = readRegister(MCP23017_INTFB);
    for (int i = 0; i < 8; i++) if (bitRead(intf, i)) return i + 8;

    return MCP23017_INT_ERR;

  }
  uint8_t CtrlPanelLib::getLastInterruptPinValue() {
    uint8_t intPin = getLastInterruptPin();
    if (intPin != MCP23017_INT_ERR) {
      uint8_t intcapreg = regForPin(intPin, MCP23017_INTCAPA, MCP23017_INTCAPB);
      uint8_t bit = bitForPin(intPin);
      return (readRegister(intcapreg) >> bit) & (0x01);
    }

    return MCP23017_INT_ERR;
  }

  // //#endif //MCP23017

  // //Setup encoder function:
  // //en_a, en_b encoder pin for rotation MCP23017 PIN A1 and A2
  // //en_c encoder click MCP23017 PIN A0
  // void CtrlPanelLib::setupEncoder(uint8_t en_a, uint8_t en_b, uint8_t en_c)  {
    // //Define Encoder rotation pin
    // pinMode(en_a, INPUT);
    // pullUp(en_a, HIGH);
    // setupInterruptPin(en_a, CHANGE);
    // pinMode(en_b, INPUT);
    // pullUp(en_b, HIGH);
    // setupInterruptPin(en_b, CHANGE);
    // //Define click pin
    // pinMode(en_c, INPUT);
    // pullUp(en_c, HIGH);
    // setupInterruptPin(en_c, CHANGE);

    // setupInterrupts(true, false, LOW);
  // }

  // //Setup button enabling interrupt handling
  // void CtrlPanelLib::setIntBtn(uint8_t pin)  {
    // pinMode(pin, INPUT);
    // pullUp(pin, HIGH);
    // setupInterruptPin(pin, FALLING);
  // }

  // //Setup 5  buttons in cross disposition and interrupt handling
  // void CtrlPanelLib::setIntCross(uint8_t btn[], uint8_t btnNum )  {
    // for (uint8_t i = 0; i < btnNum; i++)
    // {
      // pinMode(btn[i], INPUT);
      // pullUp(btn[i], HIGH);
      // setupInterruptPin(btn[i], FALLING);
    // }
  // }

