/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2023 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
**************************************************************************/
#define __TALCPU_COM_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include "tal.h"

#include "neorv32_rte.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static TAL_COM_DCB *DCBArray[TAL_COM_PORT_MAX];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  RX_IRQHandler                                                        */
/*                                                                       */
/*  This is the generic RX IRQ handler.                                  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void RX_IRQHandler (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error;
   TAL_COM_HW     *pHW    = &pDCB->HW;
   neorv32_uart_t *pUARTx = (neorv32_uart_t*)pHW->dBaseAddress;
   uint8_t         bData;

   /* Get RX data */
   bData = (uint8_t)pUARTx->DATA & 0x000000FF;

   /* clear/ack pending FIRQ */
   neorv32_cpu_csr_write(CSR_MIP, ~(1<<pHW->dRXFirqPending));

   /* If we have no overflow... */
   if (TAL_FALSE == pDCB->bRxOverflow)
   {
      /* ... put it into the ring buffer */
      Error = tal_MISCRingAdd(&pDCB->RxRing, &bData);
      if (TAL_OK == Error)
      {
         /* Signal counting semaphore */
         OS_SemaSignalFromInt(&pDCB->RxRdySema);
      }
      else
      {
         /* Ups, overflow */
         pDCB->bRxOverflow = TAL_OK;
      }
   }

} /* RX_IRQHandler */

/*************************************************************************/
/*  TX_IRQHandler                                                        */
/*                                                                       */
/*  This is the generic TX IRQ handler.                                  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void TX_IRQHandler (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error;
   TAL_COM_HW     *pHW    = &pDCB->HW;
   neorv32_uart_t *pUARTx = (neorv32_uart_t*)pHW->dBaseAddress;
   uint8_t         bData;

   if (1 == pHW->bTxFirqEnabled)
   {
      /* Read Data from the ring buffer */
      Error = tal_MISCRingGet(&pDCB->TxRing, &bData);
      if (Error != TAL_OK)
      {
         /* Ups, no data available, disable interrupt */
         pHW->bTxFirqEnabled = 0;
         neorv32_cpu_csr_clr(CSR_MIE, 1 << pHW->dTXFirqEnable);

         /* clear/ack pending FIRQ */
         neorv32_cpu_csr_write(CSR_MIP, ~(1<<pHW->dTXFirqPending));
      }
      else
      {
         /* Send data */
         pUARTx->DATA = (uint32_t)bData;

         /* clear/ack pending FIRQ */
         neorv32_cpu_csr_write(CSR_MIP, ~(1<<pHW->dTXFirqPending));
      }
   } /* end "TX interrupt */

} /* TX_IRQHandler */

/*************************************************************************/
/*  UART0_RX_IRQHandler                                                  */
/*                                                                       */
/*  This is the UART0 RX IRQ handler.                                    */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void UART0_RX_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   RX_IRQHandler(DCBArray[TAL_COM_PORT_1]);
   TAL_CPU_IRQ_EXIT();
} /* UART0_RX_IRQHandler */

/*************************************************************************/
/*  UART0_TX_IRQHandler                                                  */
/*                                                                       */
/*  This is the UART0 TX IRQ handler.                                    */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void UART0_TX_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   TX_IRQHandler(DCBArray[TAL_COM_PORT_1]);
   TAL_CPU_IRQ_EXIT();
} /* UART0_RX_IRQHandler */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  cpu_COMInit                                                          */
/*                                                                       */
/*  Prepare the hardware for use by the Open function later. Set the HW  */
/*  information depending of ePort and "enable" the COM port.            */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMInit (TAL_COM_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_ERR_COM_PORT_RANGE;
   TAL_COM_HW  *pHW    = &pDCB->HW;

   switch (pDCB->ePort)
   {
      case TAL_COM_PORT_1:
      {
         Error = tal_BoardEnableCOM1();
         if (TAL_OK == Error)
         {
            DCBArray[TAL_COM_PORT_1] = pDCB;

            pHW->dBaseAddress   = NEORV32_UART0_BASE;
            pHW->dRXFirqEnable  = UART0_RX_FIRQ_ENABLE;
            pHW->dTXFirqEnable  = UART0_TX_FIRQ_ENABLE;
            pHW->dRXFirqPending = UART0_RX_FIRQ_PENDING;
            pHW->dTXFirqPending = UART0_TX_FIRQ_PENDING;
            pHW->bTxFirqEnabled = 0;
         }
         break;
      } /* TAL_COM_PORT_1 */

      default:
      {
         /* Do nothing */
         break;
      }
   } /* end switch (pDCB->ePort) */

   return(Error);
} /* cpu_COMInit */

/*************************************************************************/
/*  cpu_COMIoctl                                                         */
/*                                                                       */
/*  Call a IOCTL function.                                               */
/*                                                                       */
/*  In    : pDCB, wNum, pParam                                           */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMIoctl (TAL_COM_DCB *pDCB, TAL_COM_IOCTL eFunc, uint32_t *pParam)
{
   (void)pDCB;
   (void)eFunc;
   (void)pParam;

   return(TAL_ERROR);
} /* cpu_COMIoctl */

/*************************************************************************/
/*  cpu_COMOpen                                                          */
/*                                                                       */
/*  Open the COM port.                                                   */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMOpen (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error = TAL_ERROR;
   TAL_COM_HW     *pHW    = &pDCB->HW;
   neorv32_uart_t *pUARTx = (neorv32_uart_t*)pHW->dBaseAddress;

   uint32_t ctrl     = 0;
   uint32_t baudrate = 0;
   uint32_t irq_mask;

   /*
    * Check parameter first
    */

   /* Check word length */
   switch (pDCB->Settings.eLength)
   {
      case TAL_COM_LENGTH_8:
      {
         /* Do nothung here */
         break;
      }

      default:
      {
         Error = TAL_ERR_COM_LENGTH;
         goto COMOpenEnd;  /*lint !e801*/
         break;   /*lint !e527*/
      }
   } /* switch (pDCB->Settings.eLength) */

   /* Check parity settings */
   switch (pDCB->Settings.eParity)
   {
      case TAL_COM_PARITY_NONE:
      {
         /* Do nothing */
         break;
      }

      default:
      {
         Error = TAL_ERR_COM_PARITY;
         goto COMOpenEnd;  /*lint !e801*/
         break;   /*lint !e527*/
      }
   } /* switch (pDCB->Settings.eParity) */

   /* Check stop bit settings */
   switch (pDCB->Settings.eStop)
   {
      case TAL_COM_STOP_1_0:
      {
         /* Do nothing here */
         break;
      }

      default:
      {
         Error = TAL_ERR_COM_STOP;
         goto COMOpenEnd;  /*lint !e801*/
         break;   /*lint !e527*/
      }
   } /* switch (pDCB->Settings.eStop) */

   /* Check baud rate */
   if (pDCB->Settings.dBaudrate != 0)
   {
      baudrate = pDCB->Settings.dBaudrate;
   }
   else
   {
      Error = TAL_ERR_COM_BAUDRATE;
      goto COMOpenEnd;  /*lint !e801*/
   }

   Error = TAL_OK;


   /* Enable and configure UART */
   irq_mask = (1<<UART_CTRL_IRQ_RX_NEMPTY) | (1<<UART_CTRL_IRQ_TX_EMPTY);
   neorv32_uart_setup(pUARTx, baudrate, irq_mask);

   /* Install RX and TX interrupt handler */
   if (TAL_COM_PORT_1 == pDCB->ePort)
   {
      neorv32_rte_handler_install(UART0_RX_RTE_ID, UART0_RX_IRQHandler);
      neorv32_rte_handler_install(UART0_TX_RTE_ID, UART0_TX_IRQHandler);
   }
   else
   {
//      neorv32_rte_handler_install(UART1_RX_RTE_ID, UART1_RX_IRQHandler);
//      neorv32_rte_handler_install(UART1_TX_RTE_ID, UART1_TX_IRQHandler);
   }

   /* Enable RX FIRQ channel */
   neorv32_cpu_csr_set(CSR_MIE, 1 << pHW->dRXFirqEnable);


COMOpenEnd:
   return(Error);
} /* cpu_COMOpen */

/*************************************************************************/
/*  cpu_COMClose                                                         */
/*                                                                       */
/*  Close the COM port.                                                  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMClose (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error = TAL_OK;
   TAL_COM_HW     *pHW    = &pDCB->HW;
   neorv32_uart_t *pUARTx = (neorv32_uart_t*)pHW->dBaseAddress;

   /* Disable UART and RX and TX interrupt */
   neorv32_uart_disable(pUARTx);

   neorv32_cpu_csr_clr(CSR_MIE, (1 << pHW->dRXFirqEnable));
   neorv32_cpu_csr_clr(CSR_MIE, (1 << pHW->dTXFirqEnable));

   pUARTx->CTRL = 0;

   return(Error);
} /* cpu_COMClose */

/*************************************************************************/
/*  cpu_COMStartTx                                                       */
/*                                                                       */
/*  Send the data from the ring buffer if the TX interrupt is disabled.  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMStartTx (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error = TAL_OK;
   TAL_COM_HW     *pHW    = &pDCB->HW;
   neorv32_uart_t *pUARTx = (neorv32_uart_t*)pHW->dBaseAddress;
   uint8_t         bData;

   TAL_CPU_DISABLE_ALL_INTS();
   if (1 == pHW->bTxFirqEnabled)
   {
      /* TX interrupt is enabled, do nothing */
   }
   else
   {
      /* Get data from the ring buffer */
      Error = tal_MISCRingGet(&pDCB->TxRing, &bData);
      if (TAL_OK == Error)
      {
         /* Send data */
         pUARTx->DATA = (uint32_t)bData;

         /* Enable TX interrupt */
         neorv32_cpu_csr_set(CSR_MIE, 1 << pHW->dTXFirqEnable);

         pHW->bTxFirqEnabled = 1;
      }
   }
   TAL_CPU_ENABLE_ALL_INTS();

   return(Error);
} /* cpu_COMStartTx */

/*************************************************************************/
/*  cpu_COMTxIsRunning                                                   */
/*                                                                       */
/*  Check if TX is still running.                                        */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / TAL_ERROR                                           */
/*************************************************************************/
TAL_RESULT cpu_COMTxIsRunning (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error = TAL_OK;
   TAL_COM_HW     *pHW    = &pDCB->HW;
   neorv32_uart_t *pUARTx = (neorv32_uart_t*)pHW->dBaseAddress;

   TAL_CPU_DISABLE_ALL_INTS();
   if (1 == pHW->bTxFirqEnabled)
   {
      /* TX is still running */
      Error = TAL_OK;
   }
   else
   {
      /* TX is not running */
      Error = TAL_ERROR;
   }
   TAL_CPU_ENABLE_ALL_INTS();

   return(Error);
} /* cpu_COMTxIsRunning */

/*************************************************************************/
/*  cpu_COMSendStringASS                                                 */
/*                                                                       */
/*  Send the assertion string.                                           */
/*                                                                       */
/*  In    : pDCB, pString                                                */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void cpu_COMSendStringASS (TAL_COM_DCB *pDCB, char *pString)
{
   (void)pDCB;
   (void)pString;

} /* cpu_COMSendStringASS */

/*** EOF ***/
