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

#include <Arduino.h>
#include <wiring_private.h>


#if defined(__SAMD51__)

#include "utility/SAMD51_I2SDevice.h"

static I2SDevice_SAMD51 i2sd(*I2S);

#else

#include "utility/SAMD21_I2SDevice.h"

static I2SDevice_SAMD21G18x i2sd(*I2S);

#endif

#include "I2S.h"

int I2SClass::_beginCount = 0;

I2SClass::I2SClass(uint8_t deviceSerIndex, uint8_t deviceClkIndex, uint8_t clockGenerator, uint8_t sdPin, uint8_t sckPin, uint8_t fsPin) :
  _deviceSerIndex(deviceSerIndex),
  _deviceClkIndex(deviceClkIndex),
  _clockGenerator(clockGenerator),
  _sdPin(sdPin),
  _sckPin(sckPin),
  _fsPin(fsPin),
  _dma(this),
  _state(I2S_STATE_IDLE),
  _dmaChannel(-1),
  _bitsPerSample(0),
  _dmaTransferInProgress(false),

  _onTransmit(NULL),
  _onReceive(NULL),
  _dmaDesc(NULL)
{
}

int I2SClass::begin(int mode, long sampleRate, int bitsPerSample)
{
  // master mode (driving clock and frame select pins - output)
  return begin(mode, sampleRate, bitsPerSample, true);
}

int I2SClass::begin(int mode, int bitsPerSample)
{
  // slave mode (not driving clock and frame select pin - input)
  return begin(mode, 0, bitsPerSample, false);
}

int I2SClass::begin(int mode, long sampleRate, int bitsPerSample, bool driveClock)
{
  if (_state != I2S_STATE_IDLE) {
    return 0;
  }

  switch (mode) {
    case I2S_PHILIPS_MODE:
    case I2S_RIGHT_JUSTIFIED_MODE:
    case I2S_LEFT_JUSTIFIED_MODE:
      break;

    default:
      // invalid mode
      return 0;
  }

  switch (bitsPerSample) {
    case 8:
    case 16:
    case 32:
      _bitsPerSample = bitsPerSample;
      break;

    default:
      // invalid bits per sample
      return 0;
  }

  // try to allocate a DMA channel
  if( DMA_STATUS_OK == _dma.allocate() )
  {
    // no DMA channel available
    _dmaChannel = _dma.getChannel();
  }

  if (_dmaChannel < 0) {
    // no DMA channel available
    return 0;
  }

  if (_beginCount == 0) {
    // enable the I2S interface
#if defined(__SAMD51__)
    MCLK->APBDMASK.reg |= MCLK_APBDMASK_I2S;
#else
    PM->APBCMASK.reg |= PM_APBCMASK_I2S;
#endif

    // reset the device
    i2sd.reset();
  }

  _beginCount++;

  if (driveClock) {
    // set up clock
    enableClock(sampleRate * 2 * bitsPerSample);

    i2sd.setSerialClockSelectMasterClockDiv(_deviceClkIndex);
    i2sd.setFrameSyncSelectSerialClockDiv(_deviceClkIndex);
  } else {
    // use input signal from SCK and FS pins
    i2sd.setSerialClockSelectPin(_deviceClkIndex);
    i2sd.setFrameSyncSelectPin(_deviceClkIndex);
  }

  // disable device before continuing
  i2sd.disable();

  switch( mode )
  {
  case I2S_RIGHT_JUSTIFIED_MODE:
    i2sd.setSlotAdjustedRight(_deviceSerIndex);
    i2sd.set0BitDelay(_deviceClkIndex);
    break;

  case I2S_LEFT_JUSTIFIED_MODE:
    i2sd.setSlotAdjustedLeft(_deviceSerIndex);
    i2sd.set0BitDelay(_deviceClkIndex);
    break;

  case I2S_PHILIPS_MODE:
  default:
    i2sd.setSlotAdjustedLeft(_deviceSerIndex);
    i2sd.set1BitDelay(_deviceClkIndex);
    break;
  }

  i2sd.setTxUnderunMode(_deviceSerIndex, true);

  i2sd.setNumberOfSlots(_deviceClkIndex, 1);
  i2sd.setSlotSize(_deviceClkIndex, bitsPerSample);
  i2sd.setDataSize(_deviceSerIndex, bitsPerSample);

  pinPeripheral(_sckPin, PIO_COM);
  pinPeripheral(_fsPin, PIO_COM);

  i2sd.setClockUnit(_deviceSerIndex,_deviceClkIndex);

  pinPeripheral(_sdPin, PIO_COM);

  // done configure enable
  i2sd.enable();

  _doubleBuffer.reset();

  return 1;
}

void I2SClass::end()
{
  if (_dmaChannel > -1) {
    while( DMA_STATUS_BUSY == _dma.free() )
    {
      delay(1);
    }

    _dmaDesc  = NULL;
  }

  _state = I2S_STATE_IDLE;
  _dmaTransferInProgress = false;

  i2sd.disableSerializer(_deviceSerIndex);
  i2sd.disableClockUnit(_deviceClkIndex);

  // set the pins back to input mode
  pinMode(_sdPin, INPUT);
  pinMode(_fsPin, INPUT);
  pinMode(_sckPin, INPUT);

  disableClock();

  _beginCount--;

  if (_beginCount == 0) {
    i2sd.disable();

    // disable the I2S interface
#if defined(__SAMD51__)
  MCLK->APBDMASK.reg &= ~(MCLK_APBDMASK_I2S);
#else
    PM->APBCMASK.reg &= ~PM_APBCMASK_I2S;
#endif
  }
}

int I2SClass::available()
{
  if (_state != I2S_STATE_RECEIVER) {
    enableReceiver();
  }

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);

  // disable interrupts,
  __disable_irq();

  size_t writable = _doubleBuffer.availableForWrite();

  if (_dmaTransferInProgress == false && (0 < writable) ) {
    // no DMA transfer in progress, start a receive process
    _dmaTransferInProgress = true;

   _dma.changeDescriptor(
      _dmaDesc,
      i2sd.data(_deviceSerIndex),                // move data from here
      _doubleBuffer.write_buffer_lock(),
      writable / (_bitsPerSample >> 3) );

    _dma.startJob();
  }

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return _doubleBuffer.availableForRead();
}

union i2s_sample_t {
  uint8_t b8;
  int16_t b16;
  int32_t b32;
};

int I2SClass::read()
{
  i2s_sample_t sample;

  sample.b32 = 0;

  read(&sample, _bitsPerSample / 8);

  if (_bitsPerSample == 32) {
    return sample.b32;
  } else if (_bitsPerSample == 16) {
    return sample.b16;
  } else if (_bitsPerSample == 8) {
    return sample.b8;
  } else {
    return 0;
  }
}

int I2SClass::peek()
{
  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);
  i2s_sample_t sample;

  sample.b32 = 0;

  // disable interrupts,
  __disable_irq();

  _doubleBuffer.peek(&sample, _bitsPerSample / 8);

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  if (_bitsPerSample == 32) {
    return sample.b32;
  } else if (_bitsPerSample == 16) {
    return sample.b16;
  } else if (_bitsPerSample == 8) {
    return sample.b8;
  } else {
    return 0;
  }
}

void I2SClass::flush()
{
  // do nothing, writes are DMA triggered
}

size_t I2SClass::write(uint8_t data)
{
  return write((int32_t)data);
}

size_t I2SClass::write(const uint8_t *buffer, size_t size)
{
  return write((const void*)buffer, size);
}

int I2SClass::availableForWrite()
{
  return _doubleBuffer.availableForWrite();
}

uint8_t*	I2SClass::write_buffer_lock()
{
  return  _doubleBuffer.write_buffer_lock();
}

void		I2SClass::write_buffer_release( size_t length )
{
  _doubleBuffer.write_buffer_release(length);

  if (_state != I2S_STATE_TRANSMITTER) {
    enableTransmitter();
  }

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);

  // disable interrupts,
  __disable_irq();

  size_t  avail = _doubleBuffer.availableForRead();
  if (_dmaTransferInProgress == false && (0 < avail) ) {
    // no DMA transfer in progress, start a transmit process
    _dmaTransferInProgress = true;

    _dma.changeDescriptor(
      _dmaDesc,
      _doubleBuffer.read_buffer_lock(),                // move data from here
      i2sd.data(_deviceSerIndex),
      avail / (_bitsPerSample >> 3) );

    _dma.startJob();
  }

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }
}



int I2SClass::read(void* buffer, size_t size)
{
  if (_state != I2S_STATE_RECEIVER) {
    enableReceiver();
  }

  int read = _doubleBuffer.read(buffer, size);

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);

  // disable interrupts,
  __disable_irq();
  size_t  writable  = _doubleBuffer.availableForWrite();
  if (_dmaTransferInProgress == false &&  (0 < writable)) {
    // no DMA transfer in progress, start a receive process
    _dmaTransferInProgress = true;

    _dma.changeDescriptor(
      _dmaDesc,
      i2sd.data(_deviceSerIndex),                // move data from here
      _doubleBuffer.write_buffer_lock(),
      writable / (_bitsPerSample >> 3) );

    _dma.startJob();
  }

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return read;
}

size_t I2SClass::write(int sample)
{
  return write((int32_t)sample);
}

size_t I2SClass::write(int32_t sample)
{
  if (_state != I2S_STATE_TRANSMITTER) {
    enableTransmitter();
  }

  // this is a blocking write
  while(!i2sd.txReady(_deviceSerIndex));

  i2sd.writeData(_deviceSerIndex, sample);

  i2sd.clearTxReady(_deviceSerIndex);

  return 1;
}

size_t I2SClass::write(const void *buffer, size_t size)
{
  if (_state != I2S_STATE_TRANSMITTER) {
    enableTransmitter();
  }

  size_t written = _doubleBuffer.write(buffer, size);

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);

  // disable interrupts,
  __disable_irq();

  size_t  avail = _doubleBuffer.availableForRead();
  if (_dmaTransferInProgress == false && (0 < avail) ) {
    // no DMA transfer in progress, start a transmit process
    _dmaTransferInProgress = true;

    _dma.changeDescriptor(
      _dmaDesc,
      _doubleBuffer.read_buffer_lock(),                // move data from here
      i2sd.data(_deviceSerIndex),
      avail / (_bitsPerSample >> 3) );

    _dma.startJob();
  }

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return written;
}

void I2SClass::onTransmit(void(*function)(void))
{
  _onTransmit = function;
}

void I2SClass::onReceive(void(*function)(void))
{
  _onReceive = function;
}

void I2SClass::enableClock(int divider)
{
  int div = SystemCoreClock / divider;
  int src = GCLK_GENCTRL_SRC_DFLL48M_Val;

  if (div > 255) {
    // divider is too big, use 8 MHz oscillator instead
    div = 8000000 / divider;
    src = GCLK_GENCTRL_SRC_OSC8M_Val;
  }

  // configure the clock divider
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENDIV.bit.ID = _clockGenerator;
  GCLK->GENDIV.bit.DIV = div;

  // use the DFLL as the source
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENCTRL.bit.ID = _clockGenerator;
  GCLK->GENCTRL.bit.SRC = src;
  GCLK->GENCTRL.bit.IDC = 1;
  GCLK->GENCTRL.bit.GENEN = 1;

  // enable
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.bit.ID = i2sd.glckId(_deviceClkIndex);
  GCLK->CLKCTRL.bit.GEN = _clockGenerator;
  GCLK->CLKCTRL.bit.CLKEN = 1;

  while (GCLK->STATUS.bit.SYNCBUSY);
}

void I2SClass::disableClock()
{
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENCTRL.bit.ID = _clockGenerator;
  GCLK->GENCTRL.bit.SRC = GCLK_GENCTRL_SRC_DFLL48M_Val;
  GCLK->GENCTRL.bit.IDC = 1;
  GCLK->GENCTRL.bit.GENEN = 0;

  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.bit.ID = i2sd.glckId(_deviceClkIndex);
  GCLK->CLKCTRL.bit.GEN = _clockGenerator;
  GCLK->CLKCTRL.bit.CLKEN = 0;

  while (GCLK->STATUS.bit.SYNCBUSY);
}

void I2SClass::enableTransmitter()
{
  i2sd.setTxMode(_deviceSerIndex);
  i2sd.enableClockUnit(_deviceClkIndex);
  i2sd.enableSerializer(_deviceSerIndex);

  _dma.setCallback( I2SClass::onDmaTransferComplete );
  _dma.setTrigger(i2sd.dmaTriggerSource(_deviceSerIndex));
  _dma.setAction(DMA_TRIGGER_ACTON_BEAT);

  _dmaDesc = _dma.addDescriptor(
    _doubleBuffer.read_buffer_lock(),                // move data from here
    i2sd.data(_deviceSerIndex),
    _doubleBuffer.availableForRead() / (_bitsPerSample >> 3),            // this many...
    (enum dma_beat_size)(_bitsPerSample >> 4),  //  DMA_BEAT_SIZE_WORD,                  // bytes/hword/words
    true,                                // increment source addr?
    false,                                // increment dest addr?
    DMA_ADDRESS_INCREMENT_STEP_SIZE_1, 
    DMA_STEPSEL_SRC);

  _state = I2S_STATE_TRANSMITTER;
}

void I2SClass::enableReceiver()
{
  i2sd.setRxMode(_deviceSerIndex);
  i2sd.enableClockUnit(_deviceClkIndex);
  i2sd.enableSerializer(_deviceSerIndex);

  _dma.setCallback( I2SClass::onDmaTransferComplete );
  _dma.setTrigger(i2sd.dmaTriggerSource(_deviceSerIndex));
  _dma.setAction(DMA_TRIGGER_ACTON_BEAT);

  _dmaDesc = _dma.addDescriptor(
      i2sd.data(_deviceSerIndex),                // move data from here
      _doubleBuffer.write_buffer_lock(),
      _doubleBuffer.availableForWrite() / (_bitsPerSample >> 3),   // this many...
      (enum dma_beat_size)(_bitsPerSample >> 4),  // DMA_BEAT_SIZE_WORD,                  // bytes/hword/words
      false,                               // increment source addr?
      true);                               // increment dest addr?

  _state = I2S_STATE_RECEIVER;
}

void I2SClass::onTransferComplete(void)
{
  if (_state == I2S_STATE_TRANSMITTER) {
    // transmit complete
    _doubleBuffer.read_buffer_release();

    size_t  avail = _doubleBuffer.availableForRead();
    if ( 0 < avail ) {
      // output is available to transfer, start the DMA process for the current buffer

      _dma.changeDescriptor(
        _dmaDesc,
        _doubleBuffer.read_buffer_lock(),                // move data from here
        i2sd.data(_deviceSerIndex),
        avail / (_bitsPerSample >> 3) );

      _dma.startJob();
    } else {
      // no user data buffered to send
      _dmaTransferInProgress = false;
    }

    // call the users transmit callback if provided
    if (_onTransmit) {
      _onTransmit();
    }
  } else {
    // receive complete
    _doubleBuffer.write_buffer_release(_doubleBuffer.availableForWrite());

    size_t  avail = _doubleBuffer.availableForWrite();
    if ( 0< avail ) {
      // the user has read all the current input, start the DMA process to fill it again
     _dma.changeDescriptor(
        _dmaDesc, 
        i2sd.data(_deviceSerIndex),                // move data from here
        _doubleBuffer.write_buffer_lock(),
        avail / (_bitsPerSample >> 3) );

      _dma.startJob();
    } else {
      // user has not read current data, no free buffer to transfer into
      _dmaTransferInProgress = false;
    }

    // call the users receveive callback if provided
    if (_onReceive) {
      _onReceive();
    }
  }
}

void I2SClass::onDmaTransferComplete(Adafruit_ZeroDMA * dma)
{
  I2SClass*	i2s	= ((I2S_DMA*)dma)->_i2s;
  if( i2s->_dmaChannel == dma->getChannel() )
  {
    i2s->onTransferComplete();
  }
}

/*
#if I2S_INTERFACES_COUNT > 0
I2SClass I2S;
#endif
*/

