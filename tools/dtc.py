#
# Copyright (c) 2006-2023, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
# Change Logs:
# Date           Author       Notes
# 2023-05-10     GuEe-GUI     the first version
#

import os
import re
import sys

from building import *

def dtc(RTT_ROOT, opt):
    os.system('scons -C ' + os.path.join(RTT_ROOT, 'tools', 'dtc'))
    return os.path.join(RTT_ROOT, 'tools', 'dtc', opt)

def dts_to_dtb(RTT_ROOT, dts_list):
    dtc_cmd = dtc(RTT_ROOT, 'dtc')
    path = GetCurrentDir() + '/'
    for dts in dts_list:
        dtb = path + dts.replace('.dts', '.dtb')
        dts = path + dts
        if not os.path.exists(dtb) or os.path.getmtime(dtb) < os.path.getmtime(dts):
            os.system("\"{}\" -I dts -O dtb -@ -A {} -o {}".format(dtc_cmd, dts, dtb))
