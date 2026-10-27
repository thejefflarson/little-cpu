// Where a boot image sits in the configuration flash and in instruction memory. This is the one place
// these numbers are stated: the loader, the wrapper that writes the flash, the image tool and the
// Makefile all read this file.
#define BOOT_FLASH_BYTES 0x400000
#define BOOT_FLASH_ALIGN 0x10000
#define BOOT_FLASH_OFFSET 0x20000
#define BOOT_ROM_BYTES 0x2000
#define BOOT_IMAGE_BASE 0x400
#define BOOT_MAGIC 0x4d49434c
