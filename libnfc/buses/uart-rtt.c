/*-
 * Free/Libre Near Field Communication (NFC) library
 *
 * Libnfc historical contributors:
 * Copyright (C) 2009      Roel Verdult
 * Copyright (C) 2009-2013 Romuald Conty
 * Copyright (C) 2010-2012 Romain Tartière
 * Copyright (C) 2010-2013 Philippe Teuwen
 * Copyright (C) 2012-2013 Ludovic Rousseau
 * See AUTHORS file for a more comprehensive list of contributors.
 * Additional contributors of this file:
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

/**
 * @file uart.c
 * @brief UART driver
 */

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif // HAVE_CONFIG_H

#include "uart.h"
#include <rtthread.h>
#include "drv_usart.h"
//#include <sys/ioctl.h>
//#include <sys/select.h>
//#include <sys/stat.h>
//#include <sys/time.h>
//#include <sys/types.h>
#include <ctype.h>
//#include <dirent.h>
#include <errno.h>
//#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
//#include <termios.h>
//#include <unistd.h>
#include <stdlib.h>

#include <nfc/nfc.h>
#include "nfc-internal.h"

#define LOG_GROUP    NFC_LOG_GROUP_COM
#define LOG_CATEGORY "libnfc.bus.uart"

#ifdef RTT_LIBNFC

#else
#ifndef _WIN32
// Needed by sleep() under Unix
#  include <unistd.h>
#  include <time.h>
#  define msleep(x) do { \
    struct timespec xsleep; \
    xsleep.tv_sec = x / 1000; \
    xsleep.tv_nsec = (x - xsleep.tv_sec * 1000) * 1000 * 1000; \
    nanosleep(&xsleep, NULL); \
  } while (0)
#else
// Needed by Sleep() under Windows
#  include <winbase.h>
#  define msleep Sleep
#endif
#endif //RTT_LIBNFC
	  
	  
#  if defined(__APPLE__)
const char *serial_ports_device_radix[] = { "tty.SLAB_USBtoUART", "tty.usbserial", "tty.usbmodem", NULL };
#  elif defined (__FreeBSD__) || defined (__OpenBSD__) || defined(__FreeBSD_kernel__)
const char *serial_ports_device_radix[] = { "cuaU", "cuau", NULL };
#  elif defined (__NetBSD__)
const char *serial_ports_device_radix[] = { "tty0", "ttyC", "ttyS", "ttyU", "ttyY", NULL };
#  elif defined (__linux__) || defined (__CYGWIN__)
const char *serial_ports_device_radix[] = { "ttyUSB", "ttyS", "ttyACM", "ttyAMA", "ttyO", NULL };
#  elif defined RTT_LIBNFC
const char *serial_ports_device_radix[] = { "uart0", "uart1", "uart2", "uart3", "uart4", NULL };
#  else
#    error "Can't determine serial string for your system"
#  endif

// As of 2015/Feb/22, Cygwin does not handle FIONREAD on physical serial devices.
// We'll use TIOCINQ instead which is pretty much the same.
#ifdef __CYGWIN__
#  include <sys/termios.h>
#  define FIONREAD TIOCINQ
#endif

// Work-around to claim uart interface using the c_iflag (software input processing) from the termios struct
#  define CCLAIMED 0x80000000

#ifndef RTT_LIBNFC
struct serial_port_unix {
  int 			fd; 			// Serial port file descriptor
  struct termios 	termios_backup; 	// Terminal info before using the port
  struct termios 	termios_new; 		// Terminal info during the transaction
};
#endif
#define UART_DATA( X ) ((struct serial_port_unix *) X)

void uart_close_ext(const serial_port sp, const bool restore_termios);

#ifdef RTT_LIBNFC
#include <rtthread.h>
#define malloc (void*)rt_malloc
#define free rt_free
#endif

static struct rt_ringbuffer *client_rx_fifo = RT_NULL;


static rt_err_t rx_ind(rt_device_t dev, rt_size_t size)
{
	uint8_t ch;
    rt_size_t i;

    for (i = 0; i < size; i++)
    {
        /* read a char */
        if (rt_device_read(dev, 0, &ch, 1))
        {
             rt_ringbuffer_put_force(client_rx_fifo, &ch, 1);
        }
    }
	

	return RT_EOK;
}

serial_port
uart_open(const char *pcPortName)
{
	
	rt_err_t open_result = RT_EOK;
//  struct serial_port_unix *sp = malloc(sizeof(struct serial_port_unix));
//
//  if (sp == 0)
//    return INVALID_SERIAL_PORT;
//	return CLAIMED_SERIAL_PORT;
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "uart_open %s ", pcPortName);

	rt_device_t uart_device = rt_device_find(pcPortName);
	if(uart_device != RT_NULL)
	{
		/* using DMA mode first */
        open_result = rt_device_open(uart_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);
        /* using interrupt mode when DMA mode not supported */
        if (open_result == -RT_EIO)
        {
            open_result = rt_device_open(uart_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
        }
        RT_ASSERT(open_result == RT_EOK);

        rt_device_set_rx_indicate(uart_device, rx_ind);
		client_rx_fifo = rt_ringbuffer_create(128);
		return uart_device;
	}

  return INVALID_SERIAL_PORT;
}

void
uart_flush_input(serial_port sp, bool wait)
{
  // flush commands may seem to be without effect
  // if asked too quickly after previous event, cf comments below
  // therefore a "wait" argument allows now to wait before flushing
  // I believe that now the byte-eater part is not required anymore --Phil
  if (wait) {
//    msleep(50); // 50 ms
	  rt_thread_mdelay(50);
  }

 
  log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "uart_flush_input");

}

void
uart_set_speed(serial_port sp, const uint32_t uiPortSpeed)
{
  log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Serial port speed requested to be set to %d baud.", uiPortSpeed);

  struct serial_configure conf;
  conf.baud_rate = uiPortSpeed;
  conf.data_bits = DATA_BITS_8;
  conf.stop_bits = STOP_BITS_1;
  conf.parity = PARITY_NONE;
  rt_device_control(sp,RT_DEVICE_CTRL_CONFIG,&conf);

}

uint32_t
uart_get_speed(serial_port sp)
{
  uint32_t uiPortSpeed = 115200;

  return uiPortSpeed;
}

void
uart_close_ext(const serial_port sp, const bool restore_termios)
{
	log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Serial port Close.");
	rt_device_close(sp);
}

void
uart_close(const serial_port sp)
{
  uart_close_ext(sp, true);
}

/**
 * @brief Receive data from UART and copy data to \a pbtRx
 *
 * @return 0 on success, otherwise driver error code
 */
int
uart_receive(serial_port sp, uint8_t *pbtRx, const size_t szRx, void *abort_p, int timeout)
{
  int iAbortFd = abort_p ? *((int *)abort_p) : 0;
  int received_bytes_count = 0;
  int available_bytes_count = 0;
  const int expected_bytes_count = (int)szRx;
  int res;
  
	rt_thread_mdelay(timeout);
  
	rt_size_t len = rt_ringbuffer_get(client_rx_fifo,pbtRx,szRx);
//  memcpy(pbtRx,read_buffer,read_size);
  log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "uart_recv %d bytes, excepted:  %d",len, szRx);
  LOG_HEX(LOG_GROUP, "RX", pbtRx, len);

//  fd_set rfds;
//  do {
//select:
//    // Reset file descriptor
//    FD_ZERO(&rfds);
//    FD_SET(UART_DATA(sp)->fd, &rfds);
//
//    if (iAbortFd) {
//      FD_SET(iAbortFd, &rfds);
//    }
//
//    struct timeval timeout_tv;
//    if (timeout > 0) {
//      timeout_tv.tv_sec = (timeout / 1000);
//      timeout_tv.tv_usec = ((timeout % 1000) * 1000);
//    }
//
//    res = select(MAX(UART_DATA(sp)->fd, iAbortFd) + 1, &rfds, NULL, NULL, timeout ? &timeout_tv : NULL);
//
//    if ((res < 0) && (EINTR == errno)) {
//      // The system call was interupted by a signal and a signal handler was
//      // run.  Restart the interupted system call.
//      goto select;
//    }
//
//    // Read error
//    if (res < 0) {
//      log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "Error: %s", strerror(errno));
//      return NFC_EIO;
//    }
//    // Read time-out
//    if (res == 0) {
//      log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "%s", "Timeout!");
//      return NFC_ETIMEOUT;
//    }
//
//    if (FD_ISSET(iAbortFd, &rfds)) {
//      // Abort requested
//      log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "%s", "Abort!");
//      close(iAbortFd);
//      return NFC_EOPABORTED;
//    }
//
//    // Retrieve the count of the incoming bytes
//    res = ioctl(UART_DATA(sp)->fd, FIONREAD, &available_bytes_count);
//    if (res != 0) {
//      return NFC_EIO;
//    }
//    // There is something available, read the data
//    res = read(UART_DATA(sp)->fd, pbtRx + received_bytes_count, MIN(available_bytes_count, (expected_bytes_count - received_bytes_count)));
//    // Stop if the OS has some troubles reading the data
//    if (res <= 0) {
//      return NFC_EIO;
//    }
//    received_bytes_count += res;
//
//  } while (expected_bytes_count > received_bytes_count);

  return NFC_SUCCESS;
}

/**
 * @brief Send \a pbtTx content to UART
 *
 * @return 0 on success, otherwise a driver error is returned
 */
int
uart_send(serial_port sp, const uint8_t *pbtTx, const size_t szTx, int timeout)
{
  (void) timeout;
  log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_DEBUG, "uart_send %d bytes", szTx);
  LOG_HEX(LOG_GROUP, "TX", pbtTx, szTx);
  
//  uint8_t test_buf[] = {0x00,0x00 ,0xFF ,0x05 ,0xFB ,0xD4 ,0x14 ,0x01 ,0x14 ,0x01 ,0x02 ,0x00};
//    rt_device_write(sp,0,test_buf,sizeof(test_buf));
  rt_device_write(sp,0,pbtTx,szTx);
  

  
//  if ((int) szTx == write(UART_DATA(sp)->fd, pbtTx, szTx))
    return NFC_SUCCESS;
//  else
//    return NFC_EIO;
}

char **
uart_list_ports(void)
{
char **res = malloc(2* sizeof(char *));
  if (!res) {
    perror("malloc");
    return res;
  }
	res[0] = malloc(8);
  	strcpy(res[0],"uart2");
	res[1] = NULL;

  return (char **)res;
}
