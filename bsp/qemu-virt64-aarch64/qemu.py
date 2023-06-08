#!/usr/bin/python
# -*- coding: utf-8 -*-
import os,sys

opt=sys.argv

graphic_cfg=""" \
	-serial stdio \
	-device virtio-gpu-device,xres=800,yres=600 \
	-device virtio-keyboard-device \
	-device virtio-mouse-device \
	-device virtio-tablet-device \
"""

q_gic=2
q_dumpdtb=""
q_smp=4
q_mem=128
q_graphic="-nographic"
q_debug=""
q_bootargs="console=ttyAMA0 earlycon root=block0 rootfstype=elm"
q_sd="sd.bin"
q_flash="flash.bin"

def is_opt(key, inkey):
	if str("-"+key) == inkey:
		return True
	return False

if sys.platform.startswith('win'):
	if not os.path.exists(q_sd):
		os.system("qemu-img create -f raw {} 64M".format(q_sd))
	if not os.path.exists(q_flash):
		os.system("qemu-img create -f raw {} 64M".format(q_flash))
else:
	if not os.path.exists(q_sd):
		os.system("dd if=/dev/zero of={} bs=1024 count=65536".format(q_sd))
	if not os.path.exists(q_flash):
		os.system("dd if=/dev/zero of={} bs=1024 count=65536".format(q_flash))

for i in range(len(opt)):
	if i == 0:
		continue
	inkey=opt[i]

	if is_opt("gic", inkey): q_gic = int(opt[i+1])
	if is_opt("dumpdtb", inkey): q_dumpdtb = str(",dumpdtb=" + opt[i+1])
	if is_opt("smp", inkey): q_smp = int(opt[i+1])
	if is_opt("mem", inkey): q_mem = int(opt[i+1])
	if is_opt("debug", inkey): q_debug = "-S -s"
	if is_opt("bootargs", inkey): q_debug = opt[i+1]
	if is_opt("graphic", inkey): q_graphic = graphic_cfg
	if is_opt("sd", inkey): q_sd = opt[i+1]
	if is_opt("flash", inkey): q_flash = opt[i+1]

if q_smp > 8:
	q_gic = 3

os.system("""
qemu-system-aarch64 \
	-M virt,gic-version={}{} \
	-cpu max \
	-smp {} \
	-m {} \
	-kernel rtthread.bin \
	-append "{}" \
	{} \
	{} \
	-drive if=none,file={},format=raw,id=blk0 \
		-device virtio-blk-device,drive=blk0 \
	-netdev user,id=net0 \
		-device virtio-net-device,netdev=net0 \
	-device virtio-rng-device \
	-device intel-hda \
	-device hda-duplex \
	-drive file={},format=raw,if=pflash,index=1 \
	-device virtio-serial-device \
		-chardev socket,host=127.0.0.1,port=4321,server=on,wait=off,telnet=on,id=console0 \
		-device virtserialport,chardev=console0
""".format(q_gic, q_dumpdtb, q_smp, q_mem, q_bootargs, q_graphic, q_debug, q_sd, q_flash))

if len(q_dumpdtb) != 0:
	dtb=q_dumpdtb.split('=')[-1]
	os.system("dtc -I dtb -O dts {} -o {}".format(dtb, dtb.replace(".dtb", ".dts")))
