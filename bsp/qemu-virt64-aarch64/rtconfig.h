#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Project Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 24
#define RT_USING_SMART
#define RT_USING_SMP
#define RT_CPUS_NR 4
#define RT_ALIGN_SIZE 8
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 100
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_HOOK_USING_FUNC_PTR
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 8192
#define SYSTEM_THREAD_STACK_SIZE 8192
#define RT_USING_TIMER_SOFT
#define RT_TIMER_THREAD_PRIO 4
#define RT_TIMER_THREAD_STACK_SIZE 8192

/* kservice optimization */

#define RT_KSERVICE_USING_STDLIB
#define RT_KPRINTF_USING_LONGLONG
#define RT_DEBUG
#define RT_DEBUG_COLOR

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

/* Memory Management */

#define RT_PAGE_MAX_ORDER 11
#define RT_USING_MEMPOOL
#define RT_USING_SMALL_MEM
#define RT_USING_MEMHEAP
#define RT_MEMHEAP_FAST_MODE
#define RT_USING_MEMHEAP_AS_HEAP
#define RT_USING_MEMHEAP_AUTO_BINDING
#define RT_USING_MEMTRACE
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_DEVICE_OPS
#define RT_USING_DM
#define RT_USING_DM_FDT
#define RT_USING_INTERRUPT_INFO
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 256
#define RT_CONSOLE_DEVICE_NAME "uart0"
#define RT_VER_NUM 0x50001

/* RT-Thread Architecture */

#define ARCH_CPU_64BIT
#define RT_USING_CACHE
#define RT_USING_HW_ATOMIC
#define ARCH_MM_MMU
#define ARCH_ARM
#define ARCH_ARM_MMU
#define KERNEL_VADDR_START 0xffff000000000000
#define ARCH_ARMV8
#define ARCH_ARMV8_EXTENSIONS 0
#define ARCH_TEXT_OFFSET 0x200000
#define ARCH_RAM_OFFSET 0x40000000
#define ARCH_ASPACE_SIZE 0xffffffff
#define ARCH_SECONDARY_CPU_STACK_SIZE 4096
#define ARCH_HAVE_EFFICIENT_UNALIGNED_ACCESS

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 8192
#define RT_MAIN_THREAD_PRIORITY 10
#define RT_USING_MSH
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 8192
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 10
#define FINSH_USING_SYMTAB
#define FINSH_CMD_SIZE 256
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_USING_DESCRIPTION
#define FINSH_ARG_MAX 10

/* DFS: device virtual file system */

#define RT_USING_DFS
#define DFS_USING_POSIX
#define DFS_USING_WORKDIR
#define DFS_FD_MAX 32
#define RT_USING_DFS_V1
#define DFS_FILESYSTEMS_MAX 8
#define DFS_FILESYSTEM_TYPES_MAX 8
#define RT_USING_DFS_ELMFAT

/* elm-chan's FatFs, Generic FAT Filesystem Module */

#define RT_DFS_ELM_CODE_PAGE 437
#define RT_DFS_ELM_WORD_ACCESS
#define RT_DFS_ELM_USE_LFN_3
#define RT_DFS_ELM_USE_LFN 3
#define RT_DFS_ELM_LFN_UNICODE_0
#define RT_DFS_ELM_LFN_UNICODE 0
#define RT_DFS_ELM_MAX_LFN 255
#define RT_DFS_ELM_DRIVES 8
#define RT_DFS_ELM_MAX_SECTOR_SIZE 512
#define RT_DFS_ELM_REENTRANT
#define RT_DFS_ELM_MUTEX_TIMEOUT 3000
#define RT_USING_DFS_DEVFS
#define RT_USING_DFS_DIRECTFS
#define RT_USING_DFS_ROMFS
#define RT_USING_LWP
#define RT_LWP_MAX_NR 30
#define LWP_TASK_STACK_SIZE 16384
#define RT_CH_MSG_MAX_NR 1024
#define LWP_CONSOLE_INPUT_BUFFER_SIZE 1024
#define LWP_TID_MAX_NR 64
#define RT_LWP_SHM_MAX_NR 64

/* Device Drivers */

#define RT_USING_TTY
#define RT_USING_NULL
#define RT_USING_ZERO
#define RT_USING_RANDOM
#define RT_USING_DEV_BUS
#define RT_USING_DEVICE_IPC
#define RT_UNAMED_PIPE_NUMBER 64
#define RT_USING_SYSTEM_WORKQUEUE
#define RT_SYSTEM_WORKQUEUE_STACKSIZE 8192
#define RT_SYSTEM_WORKQUEUE_PRIORITY 23
#define RT_USING_CLK
#define RT_USING_FIRMWARE
#define RT_FIRMWARE_PSCI
#define RT_FIRMWARE_QEMU_FW_CFG
#define RT_USING_HWTIMER
#define RT_HWTIMER_ARM_ARCH
#define RT_USING_OFW
#define RT_FDT_EARLYCON_MSG_SIZE 128
#define RT_USING_OFW_DIRECTFS
#define RT_USING_OFW_BUS_RANGES_NUMBER 8
#define RT_USING_PCI
#define RT_PCI_MSI
#define RT_PCI_ENDPOINT
#define RT_PCI_SYS_64BIT
#define RT_PCI_CACHE_LINE_SIZE 8

/* PCI Device Drivers */

#define RT_PCI_ECAM
#define RT_PCI_HOST_COMMON
#define RT_PCI_HOST_GENERIC
#define RT_USING_PIC
#define MAX_HANDLERS 512
#define RT_PIC_ARM_GIC
#define RT_PIC_ARM_GIC_V2M
#define RT_PIC_ARM_GIC_V3
#define RT_PIC_ARM_GIC_V3_ITS
#define RT_PIC_ARM_GIC_V3_ITS_IRQ_MAX 127
#define RT_PIC_ARM_GIC_MAX_NR 1
#define RT_USING_PIN
#define RT_PIN_PL061

/* Power Management (PM) Domains device drivers */

#define RT_USING_RTC
#define RT_USING_ALARM
#define RT_RTC_PL031
#define RT_USING_SERIAL
#define RT_USING_SERIAL_V1
#define RT_SERIAL_USING_DMA
#define RT_SERIAL_RB_BUFSZ 256

/* Serial Device Drivers */

#define RT_SERIAL_PL011
#define RT_USING_NULL
#define RT_USING_ZERO
#define RT_USING_RANDOM
#define RT_USING_FDT
#define RT_USING_FDTLIB
#define RT_USING_RTC
#define RT_USING_DEV_BUS
#define RT_USING_LCD
#define RT_USING_VIDEO
#define RT_VIDEO_FB
#define RT_VIDEO_LOGO
#define RT_VIDEO_LOGO_FBDEV "fb0"
#define RT_VIDEO_LOGO_RENDERING_STAGE_DRIVER_EARLY
#define RT_VIDEO_LOGO_RT_THREAD_CLUT224

#define RT_USING_VIRTIO10
#define RT_USING_VIRTIO_MMIO_ALIGN
#define RT_USING_VIRTIO_BLK
#define RT_USING_VIRTIO_CONSOLE
#define RT_USING_VIRTIO_CONSOLE_PORT_MAX_NR 4
#define RT_USING_VIRTIO_GPU
#define RT_USING_VIRTIO_INPUT

/* Using USB */


/* C/C++ and POSIX layer */

#define RT_LIBC_DEFAULT_TIMEZONE 8

/* POSIX (Portable Operating System Interface) layer */

#define RT_USING_POSIX_FS
#define RT_USING_POSIX_DEVIO
#define RT_USING_POSIX_STDIO
#define RT_USING_POSIX_POLL
#define RT_USING_POSIX_SELECT
#define RT_USING_POSIX_TERMIOS
#define RT_USING_POSIX_DELAY
#define RT_USING_POSIX_CLOCK
#define RT_USING_POSIX_TIMER

/* Interprocess Communication (IPC) */

#define RT_USING_POSIX_PIPE
#define RT_USING_POSIX_PIPE_SIZE 512

/* Socket is in the 'Network' category */


/* Network */


/* Utilities */

#define RT_USING_CRASH_CORE
#define RT_USING_ADT
#define RT_USING_ADT_AVL
#define RT_USING_ADT_BITMAP
#define RT_USING_ADT_HASHMAP
#define RT_USING_ADT_REF

/* RT-Thread Utestcases */


/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */


/* IoT Cloud */


/* security packages */


/* language packages */

/* JSON: JavaScript Object Notation, a lightweight data-interchange format */


/* XML: Extensible Markup Language */


/* multimedia packages */

/* LVGL: powerful and easy-to-use embedded GUI library */


/* u8g2: a monochrome graphic library */


/* tools packages */


/* system packages */

/* enhanced kernel services */


/* acceleration: Assembly language or algorithmic acceleration packages */


/* CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */


/* Micrium: Micrium software products porting for RT-Thread */


/* peripheral libraries and drivers */

/* sensors drivers */


/* touch drivers */


/* Kendryte SDK */


/* AI packages */


/* Signal Processing and Control Algorithm Packages */


/* miscellaneous packages */

/* project laboratory */

/* samples: kernel and components samples */


/* entertainment: terminal games and other interesting software packages */


/* Arduino libraries */


/* Projects */


/* Sensors */


/* Display */


/* Timing */


/* Data Processing */


/* Data Storage */

/* Communication */


/* Device Control */


/* Other */


/* Signal IO */


/* Uncategorized */

#define SOC_VIRT64_AARCH64

#endif
