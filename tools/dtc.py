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
        dtb = dts.replace('.dts', '.dtb')
        if not os.path.exists(path + dtb) or os.path.getmtime(path + dtb) < os.path.getmtime(path + dts):
            tmp_dts = dts + '.tmp'
            Preprocessing(dts, None, output = tmp_dts, CPPPATH=[RTT_ROOT + '/components/drivers/include'])
            os.system("\"{}\" -I dts -O dtb -@ -A {} -o {}".format(dtc_cmd, path + tmp_dts, path + dtb))
            os.remove(path + tmp_dts);
