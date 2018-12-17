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
#include "USARTClass.h"
# include "uart_serial.h"
#include "variant.h"

// Constructors ////////////////////////////////////////////////////////////////

USARTClass::USARTClass( Usart* pUsart, IRQn_Type dwIrq, uint32_t dwId, RingBuffer* pRx_buffer, RingBuffer* pTx_buffer )
  : UARTClass((Uart*)pUsart, dwIrq, dwId, pRx_buffer, pTx_buffer)
{
  // In case anyone needs USART specific functionality in the future
  _pUsart=pUsart;
}

// Public Methods //////////////////////////////////////////////////////////////

void USARTClass::begin(const uint32_t dwBaudRate)
{
  begin(dwBaudRate, Mode_8N1);
}

void USARTClass::begin(const uint32_t dwBaudRate, const UARTModes config)
{
  uint32_t modeReg = static_cast<uint32_t>(config);
  modeReg |= US_MR_USART_MODE_NORMAL | US_MR_USCLKS_MCK | US_MR_CHMODE_NORMAL;
  init(dwBaudRate, modeReg);
}

void USARTClass::begin(const uint32_t dwBaudRate, const USARTModes config)
{
  uint32_t modeReg = static_cast<uint32_t>(config);
  modeReg |= US_MR_USART_MODE_NORMAL | US_MR_USCLKS_MCK | US_MR_CHMODE_NORMAL;
  init(dwBaudRate, modeReg);
}

void USARTClass::begin(const uint32_t dwBaudRate, const int dummy)
{
	ConfigurePin(g_APinDescription[APIN_USART_SSPI_MOSI]);
	ConfigurePin(g_APinDescription[APIN_USART_SSPI_MISO]);

	_rx_buffer->_iHead = _rx_buffer->_iTail = 0;
	_tx_buffer->_iHead = _tx_buffer->_iTail = 0;

	_pUart->UART_IDR = 0xFFFFFFFF;
	_pUart->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;

	sam_usart_opt_t usart_settings = {
		dwBaudRate,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL
	};
	pmc_enable_periph_clk(ID_USART0);
	if(usart_init_rs232(USART0, &usart_settings,
				SystemPeripheralClock()))
	{
		return;
	}

	NVIC_EnableIRQ(_dwIrq);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
}

//void USARTClass::end( void )
//{
//  // Clear any received data
//  _rx_buffer->_iHead = _rx_buffer->_iTail;
//
//  // Wait for any outstanding data to be sent
//  flush();
//
//  // Disable UART interrupt in NVIC
//  NVIC_DisableIRQ( _dwIrq );
//
//  pmc_disable_periph_clk( _dwId );
//}

// End
