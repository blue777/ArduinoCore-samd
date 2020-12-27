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

#ifndef _I2S_DOUBLE_BUFFER_H_INCLUDED
#define _I2S_DOUBLE_BUFFER_H_INCLUDED

#include <stddef.h>
#include <stdint.h>

#define I2S_BUFFER_SIZE   8192
#define I2S_BUFFER_NUM    2

class I2SDoubleBuffer
{
public:
  I2SDoubleBuffer();
  virtual ~I2SDoubleBuffer();

  void reset();

  size_t availableForWrite();
  size_t write(const void *buffer, size_t size);
  uint8_t*  write_buffer_lock();
  void   write_buffer_release( size_t length );
  
  size_t read(void *buffer, size_t size);
  size_t peek(void *buffer, size_t size);
  size_t availableForRead();
  uint8_t*  read_buffer_lock();
  void   read_buffer_release();

private:
  uint8_t       _buffer[I2S_BUFFER_NUM][I2S_BUFFER_SIZE];
  volatile int  _length[I2S_BUFFER_NUM];
  volatile int _readIndex; 
  volatile int _readOffset;
  volatile int _writeIndex; 
  volatile int _writeOffset;
};

#endif
