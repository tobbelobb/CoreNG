/*
  Copyright (c) 2011 Arduino.  All right reserved.

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

#ifndef _StolenUSART_CLASS_
#define _StolenUSART_CLASS_

#include "HardwareSerial.h"
#include "UARTClass.h"
#include "RingBuffer.h"
#include "Core.h"
#include "../../libraries/SharedSpi/SharedSpi.h"

#if SAM4E || SAME70
#include "component/usart.h"
#else
#include "component/component_usart.h"
#endif

class StolenUSARTClass : public HardwareSerial
{
  public:
	typedef void (*InterruptCallbackFn)(StolenUSARTClass*);

    StolenUSARTClass(Usart* pUsart, IRQn_Type dwIrq, uint32_t dwId, RingBuffer* pRx_buffer, RingBuffer* pTx_buffer);

    void begin(const uint32_t dwBaudRate);
    void end(void);
    int available(void);
    int availableForWrite(void);
    int peek(void);
    int read(void);
    void flush(void);
    size_t write(const uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    using Print::write; // pull in write(str) and write(buf, size) from Print
    size_t canWrite( void ) const override;	//***** DC42 added for Duet

    void setInterruptPriority(uint32_t priority);
    uint32_t getInterruptPriority();

    void IrqHandler(void);

    operator bool() { return true; }; // UART always active

    InterruptCallbackFn SetInterruptCallback(InterruptCallbackFn f);

  protected:
    void init(const uint32_t dwBaudRate, const uint32_t config);

    RingBuffer * const _rx_buffer;
    RingBuffer * const _tx_buffer;

    Usart* const _pUsart;
    const IRQn_Type _dwIrq;
    const uint32_t _dwId;
    size_t numInterruptBytesMatched;
    InterruptCallbackFn interruptCallback;
    static constexpr uint8_t interruptSeq[2] = { 0xF0, 0x0F };
};

#endif // _StolenUSART_CLASS_
