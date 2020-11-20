# RT-Thread building script for component

from building import *

cwd = GetCurrentDir()
src = Glob('libnfc/*.c') 
src += ["libnfc/buses/uart-rtt.c"]
src += ["libnfc/drivers/pn532_uart.c"]
src += ["libnfc/chips/pn53x.c"]

CPPPATH = [cwd,cwd+"/include", cwd+"/libnfc", cwd+"/utils", cwd+"/libnfc/buses", cwd+"/libnfc/drivers"]

CPPDEFINES = ["RTT_LIBNFC","DRIVER_PN532_UART_ENABLED","NFC_BUFSIZE_CONNSTRING=32"]

group = DefineGroup('libnfc', src, depend = ['PKG_USING_LIBNFC' ], CPPPATH = CPPPATH, CPPDEFINES = CPPDEFINES)

Return('group')
