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

#include <string.h>

#include "I2SDoubleBuffer.h"

I2SDoubleBuffer::I2SDoubleBuffer()
{
  reset();
}

I2SDoubleBuffer::~I2SDoubleBuffer()
{
}

void I2SDoubleBuffer::reset()
{
  _readIndex  = 0;
  _readOffset = 0;
  _writeIndex = 0;
  _writeOffset = 0;
  
  for( int i = 0; i < (sizeof(_length)/sizeof(_length[0])); i++ )
  {
    _length[i] = 0;
  }
}

size_t I2SDoubleBuffer::availableForWrite()
{
  return I2S_BUFFER_SIZE - _length[_writeIndex] - _writeOffset;
}

size_t I2SDoubleBuffer::write(const void *buffer, size_t size)
{
  size_t space = availableForWrite();

  if (size > space) {
    size = space;
  }

  if (size == 0) {
    return 0;
  }

  memcpy(&_buffer[_writeIndex][_writeOffset], buffer, size);

  _writeOffset += size;
  if( I2S_BUFFER_SIZE <= _writeOffset )
  {
    _length[_writeIndex]  = _writeOffset;

    _writeIndex = (_writeIndex + 1) % I2S_BUFFER_NUM;
    _writeOffset  = 0;
  }

  return size;
}

uint8_t*  I2SDoubleBuffer::write_buffer_lock()
{
  return  &_buffer[_writeIndex][_writeOffset];
}

void   I2SDoubleBuffer::write_buffer_release( size_t length )
{
  _length[_writeIndex]  = _writeOffset + length;

  _writeIndex = (_writeIndex + 1) % I2S_BUFFER_NUM;
  _writeOffset  = 0;
}





size_t I2SDoubleBuffer::availableForRead()
{
  return _length[_readIndex] - _readOffset;
}

size_t I2SDoubleBuffer::read(void *buffer, size_t size)
{
  size_t avail = availableForRead();

  if (size > avail) {
    size = avail;
  }

  if (size == 0) {
    return 0;
  }

  memcpy(buffer, &_buffer[_readIndex][_readOffset], size);
  _readOffset += size;
  if( _length[_readIndex] <= _readOffset )
  {
    _length[_readIndex] = 0;

    _readIndex = (_readIndex + 1) % I2S_BUFFER_NUM;
    _readOffset = 0;
  }

  return size;
}

size_t I2SDoubleBuffer::peek(void *buffer, size_t size)
{
  size_t avail = availableForRead();

  if (size > avail) {
    size = avail;
  }

  if (size == 0) {
    return 0;
  }

  memcpy(buffer, &_buffer[_readIndex][_readOffset], size);
  return size;
}

uint8_t* I2SDoubleBuffer::read_buffer_lock()
{
  return &_buffer[_readIndex][_readOffset];
}

void  I2SDoubleBuffer::read_buffer_release()
{
  _length[_readIndex] = 0;

  _readIndex  = (_readIndex + 1) % I2S_BUFFER_NUM;
  _readOffset = 0;
}
