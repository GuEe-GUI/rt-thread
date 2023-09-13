#!/usr/bin/python
# -*- coding: utf-8 -*-
import os, sys

# WSL?
is_windows = sys.platform.startswith('win') or \
	os.popen('which qemu-system-aarch64 | xargs file').read().find('PE') >= 0 or \
	(os.system("readlink `which qemu-system-aarch64` > /dev/null") == 0 and \
	os.popen('readlink `which qemu-system-aarch64` | xargs file').read().find('PE') >= 0)

class QEMU_VERSION:
	def __init__(self):
		cmd = os.popen("qemu-system-aarch64 --version").readlines()[0]
		version = cmd[cmd.find("version ") + 8: -1].split('.')

		self.major = version[0]
		self.minor = version[1]
		self.revision = version[2]
	# ==
	def __eq__(self, version_in):
		version = version_in.split('.')
		return self.major == version[0] and self.minor == version[1] and self.revision == version[2]
	# >=
	def __ge__(self, version_in):
		return self.__gt__(version_in) or self.__eq__(version_in)
	# >
	def __gt__(self, version_in):
		version = version_in.split('.')
		return self.major > version[0] or \
			(self.major == version[0] and self.minor > version[1]) or \
			(self.major == version[0] and self.minor == version[1] and self.revision > version[2])
	# <=
	def __le__(self, version_in):
		return self.__lt__(version_in) or self.__eq__(version_in)
	# <
	def __lt__(self, version_in):
		return not self.__ge__(version_in)
	# !=
	def __ne__(self, version_in):
		return not self.__eq__(version_in)

qemu_version = QEMU_VERSION()

opt = sys.argv

graphic_cfg = """ \
	-serial stdio -device ramfb \
	-device virtio-gpu-device \
	-device virtio-keyboard-device \
	-device virtio-mouse-device \
	-device virtio-tablet-device \
"""

smmu_cfg = ""
iommu_cfg = ""
plugged_mem_cfg = ""
virtfs_cfg = ""
ufs_cfg = ""

q_gic = 2
q_dumpdtb = ""
q_el = 1
q_smp = 4
q_mem = 128
q_graphic = "-nographic"
q_debug = ""
q_bootargs = "console=ttyAMA0 earlycon cma=8M coherent_pool=2M root=vda0 rootfstype=elm rootwait rw"
q_initrd = ""
q_block = "block"
q_net = "user"
q_ssh = 12055
q_scsi = "scsi"
q_flash = "flash"
q_emmc = "emmc"
q_nvme = "nvme"
q_plugged_mem = 0
q_iommu = "smmu"
q_sound = "hda"
q_usbd = "usbd"
q_gl = None
q_9p = ""
q_ufs = "ufs"

def is_opt(key, inkey):
	if str("-" + key) == inkey:
		return True
	return False

for i in range(len(opt)):
	if i == 0:
		continue
	inkey = opt[i]

	if is_opt("gic", inkey): q_gic = int(opt[i + 1])
	if is_opt("dumpdtb", inkey): q_dumpdtb = str(",dumpdtb=" + opt[i + 1])
	if is_opt("el", inkey): q_el = int(opt[i + 1])
	if is_opt("smp", inkey): q_smp = int(opt[i + 1])
	if is_opt("mem", inkey): q_mem = int(opt[i + 1])
	if is_opt("debug", inkey): q_debug = "-S -s"
	if is_opt("bootargs", inkey): q_bootargs = opt[i + 1]
	if is_opt("initrd", inkey): q_initrd = str("-initrd " + opt[i + 1])
	if is_opt("graphic", inkey): q_graphic = graphic_cfg
	if is_opt("block", inkey): q_block = opt[i + 1]
	if is_opt("tap", inkey): q_net = "tap,ifname=tap0"
	if is_opt("ssh", inkey): q_ssh = int(opt[i + 1])
	if is_opt("flash", inkey): q_flash = opt[i + 1]
	if is_opt("emmc", inkey): q_emmc = opt[i + 1]
	if is_opt("nvme", inkey): q_nvme = opt[i + 1]
	if is_opt("plugged-mem", inkey): q_plugged_mem = int(opt[i + 1])
	if is_opt("iommu", inkey): q_iommu = opt[i + 1]
	if is_opt("sound", inkey): q_sound = opt[i + 1]
	if is_opt("usbd", inkey): q_usbd = opt[i + 1]
	if is_opt("gl", inkey): q_gl = "-device virtio-gpu-gl-device -display gtk,gl=on "
	if is_opt("9p", inkey): q_9p = opt[i + 1]
	if is_opt("ufs", inkey): q_ufs = opt[i + 1]

# SMP
if q_smp > 8:
	q_gic = 3

# Exception Level
if q_el == 1:
	q_el = ""
elif q_el == 2:
	q_el = ",virtualization=on"
	if q_gic == 3:
		q_gic = "max"
elif q_el == 3:
	q_el = ",secure=on"
else:
	print("Error: Invalid -el {}".format(q_el))
	exit(-1)

# IOMMU
if q_iommu == "smmu":
	smmu_cfg = "iommu=smmuv3,"
elif q_iommu == "virtio":
	iommu_cfg = "-device virtio-iommu-device,primary-bus=pcie.0 "
else:
	print("Error: Invalid -iommu {}".format(q_iommu))
	exit(-1)

# Display
#	--enable-opengl --enable-virglrenderer
if q_graphic != "-nographic":
	if is_windows:
		q_bootargs += " ramfb.buffer_size=1"
elif q_gl != None:
	print("Error: GL should in graphic mode")
	exit(-1)

# Sound
if q_sound == "hda":
	q_sound = "-device intel-hda -device hda-duplex "
elif q_sound == "virtio":
	q_sound = "-device virtio-sound-pci,audiodev=vsnd -audiodev alsa,id=vsnd "
else:
	print("Error: Invalid -sound {}".format(q_sound))
	exit(-1)

# Net
#	--enable-slirp
if q_net.find("user") >= 0:
	q_net += ",hostfwd=tcp::{}-:22".format(q_ssh)
else:
	if not is_windows:
		q_net += ",script=no,downscript=no"
	print("Warning: SSH not set in TAP")

# Storage
#	pflash have pflash0 and pflash1, pflash0 is used for BootROMs such as UEFI
#	if we load file to pflash0, QEMU will boot from it, so we only use pflash1.
#	Well, we can R/W in pflash0 by CFI driver, but the data will lost after QEMU exits.
#
#	partitions (not support in Windows, Maybe WSL2):
#		modprobe nbd max_part=12
#		qemu-nbd --connect=/dev/nbdX ABC.qcow2
#		fdisk /dev/nbdX
#		...
#		qemu-nbd --disconnect /dev/nbdX
disk_list = [q_block, q_scsi, q_flash, q_emmc, q_nvme, q_usbd]

if qemu_version >= '8.2.0':
	disk_list += [q_ufs]
	ufs_cfg = """ \
		-drive if=none,file={}.qcow2,format=qcow2,id=ufs \
			-device ufs,serial=deadbeef \
			-device ufs-lu,drive=ufs \
	""".format(q_ufs)

for disk in disk_list:
	disk += ".qcow2"
	if not os.path.exists(disk):
		os.system("qemu-img create -f qcow2 {} 64M".format(disk))

# Share File System
#	--enable-virtfs
if len(q_9p) > 0:
	virtfs_cfg = """ \
		-fsdev local,security_model=passthrough,id=fsdev0,path={} \
			-device virtio-9p-device,fsdev=fsdev0,mount_tag=hostshare \
	""".format(q_9p)

# Plugged Memory
if is_windows:
	if q_plugged_mem > 0:
		print("Warning: virtio-mem is not supported in MS Windows")
else:
	plugged_mem = "plugged-mem.bin"

	if q_plugged_mem == 0:
		q_plugged_mem = 64

	if not os.path.exists(plugged_mem) or os.path.getsize(plugged_mem) != q_plugged_mem * 1024 * 1024:
		os.system("qemu-img create -f raw {} {}M".format(plugged_mem, q_plugged_mem))

	plugged_mem_cfg = """ \
		-device virtio-mem,memdev=plugged-mem \
			-object memory-backend-file,mem-path={},id=plugged-mem,share=on,size={} \
	""".format(plugged_mem, os.path.getsize(plugged_mem))

os.system("""
qemu-system-aarch64 \
	-M virt,acpi=on,{}its=on,gic-version={}{}{} \
	-cpu max \
	-smp {} \
	-m {} \
	-kernel rtthread.bin \
	-append "{}" \
	-device vmcoreinfo \
	{} \
	{} \
	{} \
	-drive if=none,file={}.qcow2,format=qcow2,id=blk0 \
		-device virtio-blk-device,drive=blk0 \
	-netdev {},id=net0 \
		-device virtio-net-device,netdev=net0,speed=800000 \
	-device virtio-rng-device \
	-device virtio-balloon-device \
	-device virtio-scsi-pci \
		-device scsi-hd,channel=0,scsi-id=0,lun=2,drive=scsi0 \
		-drive file={}.qcow2,format=qcow2,if=none,id=scsi0 \
	{} \
	{} \
	-device virtio-crypto-device,cryptodev=vcrypto \
		-object cryptodev-backend-builtin,id=vcrypto \
	-device virtio-serial-device \
		-chardev socket,host=127.0.0.1,port=4321,server=on,wait=off,telnet=on,id=console0 \
		-device virtserialport,chardev=console0 \
	{} \
	{} \
	-drive if=pflash,file={}.qcow2,format=qcow2,index=1 \
	-device pci-serial,chardev=console1 \
		-chardev socket,host=127.0.0.1,port=4322,server=on,wait=off,telnet=on,id=console1 \
	-device sdhci-pci -device sd-card,drive=emmc \
		-drive if=none,file={}.qcow2,format=qcow2,id=emmc \
	-device nvme,serial=deadbeef,drive=nvme \
		-drive if=none,file={}.qcow2,format=qcow2,id=nvme \
	-device i6300esb -watchdog-action reset \
	-device qemu-xhci \
		-device usb-storage,drive=usbd \
			-drive if=none,file={}.qcow2,format=qcow2,id=usbd \
	{} \
	-device edu,dma_mask=0xffffffff
""".format(smmu_cfg, q_gic, q_dumpdtb, q_el, q_smp, q_mem, q_bootargs, q_initrd,
	q_graphic, q_debug, q_block, q_net, q_scsi, virtfs_cfg, plugged_mem_cfg,
	iommu_cfg, q_sound, q_flash, q_emmc, q_nvme, q_usbd, ufs_cfg))

if len(q_dumpdtb) != 0:
	dtb = q_dumpdtb.split('=')[-1]
	os.system("dtc -I dtb -O dts -@ -A {} -o {}".format(dtb, dtb.replace(".dtb", ".dts")))
