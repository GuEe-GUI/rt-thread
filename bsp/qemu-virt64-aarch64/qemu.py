#!/usr/bin/python
# -*- coding: utf-8 -*-
import os, sys

if os.getenv('RTT_ROOT'):
	RTT_ROOT = os.getenv('RTT_ROOT')
else:
	RTT_ROOT = os.path.join(os.getcwd(), '..', '..')

sys.path.append(RTT_ROOT + '/tools')

from building import *
import dtc

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
q_plugged_mem = ""
q_iommu = "smmu"
q_sound = "hda"
q_gl = None
q_9p = ""
q_ufs = "ufs"
q_dtbo = []
q_dtb = ""

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
	if is_opt("plugged-mem", inkey): q_plugged_mem = opt[i + 1]
	if is_opt("iommu", inkey): q_iommu = opt[i + 1]
	if is_opt("sound", inkey): q_sound = opt[i + 1]
	if is_opt("gl", inkey): q_gl = "-device virtio-gpu-gl-device -display gtk,gl=on "
	if is_opt("9p", inkey): q_9p = opt[i + 1]
	if is_opt("ufs", inkey): q_ufs = opt[i + 1]
	if is_opt("dtb", inkey): q_dtb = "-dtb " + opt[i + 1]
	if is_opt("dtbo", inkey): q_dtbo += [opt[i + 1]]

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
	iommu_cfg = "-device virtio-iommu-device,disable-legacy=on,primary-bus=pcie.0 "
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
	if qemu_version < '8.2.0':
		print("Error: VirtIO Sound is not in this version: {}".format(qemu_version))
		exit(-1)
	q_sound = "-device virtio-sound-pci,audiodev=vsnd -audiodev alsa,id=vsnd "
else:
	print("Error: Invalid -sound {}".format(q_sound))
	exit(-1)

# Net
#	--enable-slirp
#	Enable TAP in example (not support in Windows, Maybe WSL2):
#		ETH0=`/sbin/route -n | awk '$1 == "0.0.0.0" {print $NF}'`
#		ip link add br0 type bridge
#		ip link set br0 up
#		echo 1 | tee /proc/sys/net/ipv4/ip_forward > /dev/null
#		ip link set ${ETH0} up
#		ip link set ${ETH0} master br0
#		dhclient br0
#		ip tuntap add dev tap0 mode tap
#		ip link set dev tap0 up
#		ip link set tap0 master br0
#
#	Disable TAP in example (not support in Windows, Maybe WSL2):
#		ip link set tap0 down
#		ip tuntap del dev tap0 mod tap
#		echo 0 | tee /proc/sys/net/ipv4/ip_forward > /dev/null
#		ip link set br0 down
#		ip link del br0 type bridge
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
disk_list = [q_block, q_scsi, q_flash, q_emmc, q_nvme]

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
	p9_tag = "hostshare"
	virtfs_cfg = """ \
		-fsdev local,security_model=passthrough,id=fsdev0,path={} \
			-device virtio-9p-device,fsdev=fsdev0,mount_tag={} \
	""".format(q_9p, p9_tag)
	q_bootargs += " 9p.tag={} 9p.mount=host".format(p9_tag)

# Plugged Memory
if len(q_plugged_mem) > 0:
	if is_windows:
		print("Warning: virtio-mem is not supported in MS Windows")
	else:
		plugged_mem_cfg = """ \
			-device virtio-mem,id=plugged-mem0,memdev=plugged-mem-pool0,node=0,requested-size={} \
				-object memory-backend-ram,id=plugged-mem-pool0,size={} \
		""".format(q_plugged_mem, q_plugged_mem)

# Note
#	Hot-plug / Hot-unplug in QEMU monitor:
#		(qemu) chardev-add socket,host=127.0.0.1,port=4323,server=on,wait=off,telnet=on,id=console2
#		(qemu) device_add virtserialport,chardev=console2,name=org.rt-thread.port,id=port2
#		(qemu) device_del port2
#		(qemu) chardev-remove console2
#
#	VirtIO version disable legacy to set version >= 1.0:
#		-global virtio-mmio.force-legacy=false
#
#	VirtIO used virtqueue packed (version >= 1.0)
#		-device virtio-XYZ-device,packed=on
#
#	VirtIO used PCI/PCIe bus (version >= 1.0)
#		-device virtio-XYZ-pci,disable-legacy=on

cmd_base = """
qemu-system-aarch64 \
	-M virt,acpi=on,{}its=on,gic-version={}{}{} \
	-cpu max \
	-smp {} \
	-m {} \
	-kernel rtthread.bin \
	-append "{}" \
	{} \
	{} \
	-device vmcoreinfo \
	{} \
	{} \
	-drive if=none,file={}.qcow2,format=qcow2,id=blk0 \
		-device virtio-blk-device,drive=blk0 \
	-netdev {},id=net0 \
		-device virtio-net-device,netdev=net0,speed=800000 \
	-device virtio-rng-device \
	-device virtio-balloon-device \
	-device virtio-scsi-pci,disable-legacy=on \
		-device scsi-hd,channel=0,scsi-id=0,lun=2,drive=scsi0 \
		-drive file={}.qcow2,format=qcow2,if=none,id=scsi0 \
	{} \
	{} \
	-device virtio-crypto-device,cryptodev=vcrypto0 \
		-object cryptodev-backend-builtin,id=vcrypto0 \
	-device virtio-serial-device \
		-chardev socket,host=127.0.0.1,port=4321,server=on,wait=off,telnet=on,id=console0 \
		-device virtserialport,chardev=console0,name=org.rt-thread.console \
	{} \
	{} \
	-drive if=pflash,file={}.qcow2,format=qcow2,index=1 \
	-device pci-serial,chardev=console1 \
		-chardev socket,host=127.0.0.1,port=4322,server=on,wait=off,telnet=on,id=console1 \
	-device sdhci-pci -device sd-card,drive=emmc0 \
		-drive if=none,file={}.qcow2,format=qcow2,id=emmc0 \
	-device nvme,id=nvme-ctrl-0,serial=deadbeef \
		-drive if=none,file={}.qcow2,format=qcow2,id=nvme0 \
		-device nvme-ns,drive=nvme0 \
	-device i6300esb -watchdog-action reset \
	{} \
	-device edu,dma_mask=0xffffffff
"""
def cmd():
	return cmd_base.format(smmu_cfg, q_gic, q_dumpdtb, q_el, q_smp, q_mem, q_bootargs, q_dtb, q_initrd,
		q_graphic, q_debug, q_block, q_net, q_scsi, virtfs_cfg, plugged_mem_cfg,
		iommu_cfg, q_sound, q_flash, q_emmc, q_nvme, ufs_cfg)

def dumpdtb():
	dtb = q_dumpdtb.split('=')[-1]
	os.system("dtc -I dtb -O dts -@ -A {} -o {}".format(dtb, dtb.replace(".dtb", ".dts")))

if len(q_dtbo) > 0:
	# Dump raw QEMU dts
	q_dumpdtb = ",dumpdtb=qemu.tmp.dtb"
	os.system(cmd())
	dumpdtb();
	os.remove("qemu.tmp.dtb")
	q_dumpdtb = ""
	# Merge all dts
	q_dtbo = ["qemu.tmp.dts"] + q_dtbo
	with open("qemu.run.dts", 'w') as run_dts:
		for dts in q_dtbo:
			with open(dts) as dts:
				run_dts.write(dts.read())
	os.remove("qemu.tmp.dts")
	# Build runtime dtb
	dtc.dts_to_dtb(RTT_ROOT, ["qemu.run.dts"])
	os.remove("qemu.run.dts")
	# Run QEMU
	q_dtb = "-dtb qemu.run.dtb"
	os.system(cmd())
	os.remove("qemu.run.dtb")
else:
	os.system(cmd())

	if len(q_dumpdtb) != 0:
		dumpdtb()
