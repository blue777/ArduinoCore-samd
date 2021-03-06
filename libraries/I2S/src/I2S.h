/*
  Copyright (c) 2016 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _I2S_H_INCLUDED
#define _I2S_H_INCLUDED

#include <Arduino.h>

#include "utility/I2SDoubleBuffer.h"
#include <Adafruit_ZeroDMA.h>

typedef enum {
  I2S_PHILIPS_MODE,
  I2S_RIGHT_JUSTIFIED_MODE,
  I2S_LEFT_JUSTIFIED_MODE
} i2s_mode_t;


#ifndef I2S_DEVICE_CLK
#define I2S_DEVICE_CLK       I2S_DEVICE
#endif

#ifndef I2S_DEVICE_SER
#define I2S_DEVICE_SER       I2S_DEVICE
#endif


class I2SClass : public Stream
{
public:
  // the device index and pins must map to the "COM" pads in Table 6-1 of the datasheet 
  I2SClass(uint8_t deviceSerIndex=I2S_DEVICE_SER, uint8_t deviceClkIndex=I2S_DEVICE_CLK, uint8_t clockGenerator=I2S_CLOCK_GENERATOR, uint8_t sdPin=PIN_I2S_SD, uint8_t sckPin=PIN_I2S_SCK, uint8_t fsPin=PIN_I2S_FS);

  int begin(int mode, long sampleRate, int bitsPerData, int bitsPerSlot, bool driveClock);
  // the SCK and FS pins are driven as outputs using the sample rate
  int begin(int mode, long sampleRate, int bitsPerSample);
  // the SCK and FS pins are inputs, other side controls sample rate
  int begin(int mode, int bitsPerSample);
  void end();

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  // from Print
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buffer, size_t size);

  virtual int availableForWrite();

  int read(void* buffer, size_t size);

  size_t write(int);
  size_t write(int32_t);
  size_t write(const void *buffer, size_t size);
  uint8_t*	write_buffer_lock();
  void		write_buffer_release( size_t length );

  void onTransmit(void(*)(void));
  void onReceive(void(*)(void));

private:

  void enableClock(int divider);
  void disableClock();

  void enableTransmitter();
  void enableReceiver();

  void onTransferComplete(void);

  static void onDmaTransferComplete(Adafruit_ZeroDMA * dma);

private:
  class I2S_DMA : public Adafruit_ZeroDMA
  {
  public:
    I2S_DMA( I2SClass * i2s ) :
      _i2s(i2s)
    {
    }

  public:
    I2SClass* _i2s;
  };

  typedef enum {
    I2S_STATE_IDLE,
    I2S_STATE_TRANSMITTER,
    I2S_STATE_RECEIVER
  } i2s_state_t;

  static int _beginCount;

  uint8_t _deviceClkIndex;
  uint8_t _deviceSerIndex;
  uint8_t _clockGenerator;
  uint8_t _sdPin;
  uint8_t _sckPin;
  uint8_t _fsPin;

  i2s_state_t		_state;
  uint8_t			  _bitsPerData;
  uint8_t			  _bitsPerSlot;
  I2S_DMA           _dma;
  DmacDescriptor*   _dmaDesc;
  I2SDoubleBuffer	_doubleBuffer;
  int8_t			_dmaChannel;
  volatile bool		_dmaTransferInProgress;

  void (*_onTransmit)(void);
  void (*_onReceive)(void);
};

// "I2S" is already defined by the CMSIS device, undefine it so the I2SClass
// instance can be called I2S
#undef I2S

#if I2S_INTERFACES_COUNT > 0
extern I2SClass I2S;
#else
#error "I2S is not supported on your board!"
#endif

#endif
