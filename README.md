# U-boot for NanoPi M3

Based on [u-boot-artik](https://github.com/SamsungARTIK/u-boot-artik)
to run Linux 64-bit kernel on Nanopi M3.


## Installation

If the Debian was installed using
[debian installer](https://github.com/rafaello7/debian-installer-nanopi-m3),
the following command (invoked as root) should install the new version:

	dd if=u-boot.bin of=/dev/mmcblk2 seek=65

### Compilation

        make ARCH=arm nanopim3_defconfig
        make ARCH=arm CROSS_COMPILE=aarch64-linux-gnu-

