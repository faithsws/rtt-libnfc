# RT-Thread building script for component

from building import *

cwd = GetCurrentDir()
src = Glob('libnfc/*.c') 
CPPPATH = [cwd,cwd+"/include"]
CPPDEFINES = []

group = DefineGroup('libnfc', src, depend = ['PKG_USING_LIBNFC' ], CPPPATH = CPPPATH, CPPDEFINES = CPPDEFINES)

Return('group')
