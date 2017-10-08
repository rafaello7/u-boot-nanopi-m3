## U-boot for NanoPi M3

Based on [u-boot-artik](https://github.com/SamsungARTIK/u-boot-artik)
to run Linux 64-bit kernel on Nanopi M3.


### Installation

If the Debian was installed using
[debian installer](https://github.com/rafaello7/debian-installer-nanopi-m3),
it is recommended to use _nano-blembed_ utility from
[nanopi-boot-tools](https://github.com/rafaello7/nanopi-boot-tools)
to install the new version.


### Compilation

        make ARCH=arm nanopim3_defconfig
        make ARCH=arm CROSS_COMPILE=aarch64-linux-gnu-

