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

#include <cstdlib>
#include <cstring>

#include "asf.h"
#include "StolenUSARTClass.h"
#include "WMath.h"
#include "../../libraries/SharedSpi/SharedSpi.h"

// Constructors ////////////////////////////////////////////////////////////////

StolenUSARTClass::StolenUSARTClass(Usart *pUsart, IRQn_Type dwIrq, uint32_t dwId, RingBuffer *pRx_buffer, RingBuffer *pTx_buffer)
	: _rx_buffer(pRx_buffer), _tx_buffer(pTx_buffer), _pUsart(pUsart), _dwIrq(dwIrq), _dwId(dwId),
	  numInterruptBytesMatched(0), interruptCallback(nullptr)
{
}

// Public Methods //////////////////////////////////////////////////////////////

void StolenUSARTClass::begin(const uint32_t dwBaudRate)
{
	usartUartSetupResult retVal = uartOnSspiPinsInit(dwBaudRate);
	switch(retVal)
	{
		case usartUartSetupResult::success:
			//reply.copy("Ok, everything should be done.");
			break;
		case usartUartSetupResult::uartSetupAlready:
			//reply.copy("UART was already configured on the shared SPI pins.");
			break;
		case usartUartSetupResult::spiSetupAlready:
			//reply.copy("SPI was already configured on the shared SPI pins.");
			break;
		case usartUartSetupResult::error:
			//reply.copy("uartOnSspiPinsInit() breaked error.");
			break;
		default:
			//reply.printf("Error: usartUartSetupResult %u not handled.", (uint8_t)retVal);
			break;
	}

  // Make sure both ring buffers are initialized back to empty.
  _rx_buffer->_iHead = _rx_buffer->_iTail = 0;
  _tx_buffer->_iHead = _tx_buffer->_iTail = 0;

  // Configure interrupts
  _pUsart->US_IDR = 0xFFFFFFFF;
  _pUsart->US_IER = US_IER_RXRDY | US_IER_OVRE | US_IER_FRAME;

  // Enable UART interrupt in NVIC
  NVIC_EnableIRQ(_dwIrq);
}

void StolenUSARTClass::end( void )
{
  // Clear any received data
  _rx_buffer->_iHead = _rx_buffer->_iTail;

  // Wait for any outstanding data to be sent
  flush();

  // Disable UART interrupt in NVIC
  NVIC_DisableIRQ( _dwIrq );
  pmc_disable_periph_clk( _dwId );
}

void StolenUSARTClass::setInterruptPriority(uint32_t priority)
{
  NVIC_SetPriority(_dwIrq, priority & 0x0F);
}

uint32_t StolenUSARTClass::getInterruptPriority()
{
  return NVIC_GetPriority(_dwIrq);
}

int StolenUSARTClass::available( void )
{
  return (SERIAL_BUFFER_SIZE + _rx_buffer->_iHead - _rx_buffer->_iTail) % SERIAL_BUFFER_SIZE;
}

int StolenUSARTClass::availableForWrite(void)
{
  size_t head = _tx_buffer->_iHead;
  size_t tail = _tx_buffer->_iTail;
  if (head >= tail)
  {
	  return SERIAL_BUFFER_SIZE - 1 - head + tail;
  }
  return tail - head - 1;
}

int StolenUSARTClass::peek( void )
{
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
  {
    return -1;
  }

  return _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
}

int StolenUSARTClass::read( void )
{
  // if the head isn't ahead of the tail, we don't have any characters
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
  {
    return -1;
  }

  uint8_t uc = _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
  _rx_buffer->_iTail = (_rx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
  return uc;
}

void StolenUSARTClass::flush( void )
{
  while (_tx_buffer->_iHead != _tx_buffer->_iTail); //wait for transmit data to be sent
  // Wait for transmission to complete
  while ((_pUsart->US_CSR & US_CSR_TXRDY) > 0)
   ;
}

size_t StolenUSARTClass::write( const uint8_t uc_data )
{
  // Is the hardware currently busy?
  if (!((_pUsart->US_CSR & US_CSR_TXRDY) > 0) || _tx_buffer->_iTail != _tx_buffer->_iHead)
  {
    // If busy we buffer
    const size_t hn = (_tx_buffer->_iHead + 1) % SERIAL_BUFFER_SIZE;
    while (_tx_buffer->_iTail == hn)
      ; // Spin locks if we're about to overwrite the buffer. This continues once the data is sent

    _tx_buffer->_aucBuffer[_tx_buffer->_iHead] = uc_data;
    _tx_buffer->_iHead = hn;
    // Make sure TX interrupt is enabled
    _pUsart->US_IER = US_IER_TXRDY;
  }
  else
  {
     // Bypass buffering and send character directly
	 _pUsart->US_THR = US_THR_TXCHR(uc_data);
  }
  return 1;
}

size_t StolenUSARTClass::write(const uint8_t *buffer, size_t size)
{
	size_t ret = size;
	while (size != 0)
	{
		size_t written = _tx_buffer->storeBlock(buffer, size);
		buffer += written;
		size -= written;
	    _pUsart->US_IER = US_IER_TXRDY;
	}
	return ret;
}

size_t StolenUSARTClass::canWrite() const
{
	return _tx_buffer->roomLeft();		// we may also be able to write 1 more byte direct to the UART, but this is close enough
}

void StolenUSARTClass::IrqHandler()
{
  const uint32_t status = _pUsart->US_CSR;

  // Did we receive data?
  if ((status & US_CSR_RXRDY) != 0)
  {
	  const uint8_t c = _pUsart->US_RHR & US_RHR_RXCHR_Msk;
	  if (c == interruptSeq[numInterruptBytesMatched])
	  {
		  ++numInterruptBytesMatched;
		  if (numInterruptBytesMatched == ARRAY_SIZE(interruptSeq))
		  {
			  numInterruptBytesMatched = 0;
			  if (interruptCallback != nullptr)
			  {
				  interruptCallback(this);
			  }
		  }
	  }
	  else
	  {
		  numInterruptBytesMatched = 0;
	  }
	  _rx_buffer->store_char(c);
  }

  // Do we need to keep sending data?
  if ((status & US_CSR_TXRDY) != 0)
  {
    if (_tx_buffer->_iTail != _tx_buffer->_iHead)
    {
      _pUsart->US_THR = _tx_buffer->_aucBuffer[_tx_buffer->_iTail];
      _tx_buffer->_iTail = (_tx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
    }
    else
    {
      // Mask off transmit interrupt so we don't get it anymore
      _pUsart->US_IDR = US_IDR_TXRDY;
    }
  }

  // Acknowledge errors
  if ((status & (US_CSR_OVRE | US_CSR_FRAME)) != 0)
  {
    _pUsart->US_CR = US_CR_RSTSTA;
    _rx_buffer->store_char(0x7F);				// store a DEL character so that the receiving process knows there has been an error
  }
}

StolenUSARTClass::InterruptCallbackFn StolenUSARTClass::SetInterruptCallback(InterruptCallbackFn f)
{
	InterruptCallbackFn ret = interruptCallback;
	interruptCallback = f;
	return ret;
}

// End
