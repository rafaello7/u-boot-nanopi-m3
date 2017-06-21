# U-boot for NanoPi M3

Based on [u-boot-artik](https://github.com/SamsungARTIK/u-boot-artik)
to run Linux 64-bit kernel on Nanopi M3.

In development.

### compilation

        make ARCH=arm nanopim3_defconfig
        make ARCH=arm CROSS_COMPILE=aarch64-linux-gnu-

