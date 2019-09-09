import sys
import os
import re

env = Environment()
env['WIN32'] = (sys.platform.startswith('win'))
env['UNIX'] = (sys.platform.startswith('linux') or sys.platform.startswith('darwin'))
env['MSVC'] = (env['CC'] == 'cl')
env['MINGW'] = False
env['APPLE'] = False
env['XCODE'] = False
env['CYGWIN'] = False
localEnv = env

