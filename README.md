# U-boot for ARTIK5 and ARTIK10
## Contents
1. [Introduction](#1-introduction)
2. [Build guide](#2-build-guide)
3. [Update guide](#3-update-guide)

## 1. Introduction
This 'u-boot-artik' repository is u-boot source for artik710 and artik530.
The base version of the u-boot of artik710/artik530 is based on 2016-01 version.

---
## 2. Build guide
### 2.1 Install cross compiler
+ For artik710> You'll need an arm64 cross compiler
```
sudo apt-get install gcc-aarch64-linux-gnu device-tree-compiler
```
If you can't install the above toolchain, you can use linaro toolchain.
```
wget https://releases.linaro.org/components/toolchain/binaries/5.3-2016.05/aarch64-linux-gnu/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu.tar.xz
tar xf gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu.tar.xz
export PATH=~/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu:$PATH
```

+ ARTIK530
```
sudo apt-get install gcc-arm-linux-gnueabihf
```
If you can't install the above toolchain, you can use linaro toolchain.
```
wget http://releases.linaro.org/components/toolchain/binaries/latest-5/arm-linux-gnueabihf/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf.tar.xz
tar xf gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf.tar.xz
export PATH=~/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/bin:$PATH
```
You can the path permernently through adding it into ~/.bashrc

### 2.2 Build the u-boot
+ For artik710>
```
make ARCH=arm artik710_raptor_defconfig
make ARCH=arm CROSS_COMPILE=aarch64-linux-gnu-
```

+ For artik530>
```
make ARCH=arm artik530_raptor_defconfig
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
```

### 2.3 Generate params.bin
The params.bin contains u-boot environment variables and you can generate the file using below commands:

+ For artik710>
```
cp `find . -name "env_common.o"` copy_env_common.o
aarch64-linux-gnu-objcopy -O binary --only-section=.rodata.default_environment `find . -name "copy_env_common.o"`
tr '\0' '\n' < copy_env_common.o | grep '=' > default_envs.txt
cp default_envs.txt default_envs.txt.orig
tools/mkenvimage -s 16384 -o params.bin default_envs.txt
```

+ For artik530>
```
cp `find . -name "env_common.o"` copy_env_common.o
arm-linux-gnueabihf-objcopy -O binary --only-section=.rodata.default_environment `find . -name "copy_env_common.o"`
tr '\0' '\n' < copy_env_common.o | grep '=' > default_envs.txt
cp default_envs.txt default_envs.txt.orig
tools/mkenvimage -s 16384 -o params.bin default_envs.txt
```

### 2.4 Generate fip/nexell compatable image
The u-boot.bin cannot be loaded by bl1/bl2 bootloaders. You have to generate
a fip image and boot image for nexell bl1.

+ For artik710>
a. generate a fip-nonsecure.bin image using fip_create tool(input: u-boot.bin, output:fip-nonsecure.bin)
```
tools/fip_create/fip_create \
	--dump --bl33 u-boot.bin \
	fip-nonsecure.bin
```
b. generate a fip-nonsecure.img using SECURE_BINGEN tool(input: fip-nonsecure.bin, output: fip-nonsecure.img)
```
tools/nexell/SECURE_BINGEN \
	-c S5P6818 -t 3rdboot \
	-n tools/nexell/nsih/raptor-64.txt \
	-i fip-nonsecure.bin \
	-o fip-nonsecure.img \
	-l 0x7df00000 -e 0x00000000
```

+ For artik530>
a. generate a bootloader.img using BOOT_BINGEN tool
```
tools/nexell/BOOT_BINGEN \
	-c S5P4418 -t 3rdboot \
	-n tools/nexell/nsih/raptor-emmc.txt \
	-i u-boot.bin \
	-o bootloader.img \
	-l 0x43c00000 -e 0x43c00000

---
## 3. Update Guide
You can update the u-boot through fastboot or micro sd card(ext4 partition)

### 3.1 Fastboot
For artik710, you should prepare micro USB cable.

Install android-tools-fastboot
```
sudo apt-get install android-tools-fastboot
wget -S -O - http://source.android.com/source/51-android.rules | sed "s/<username>/$USER/" | sudo tee >/dev/null /etc/udev/rules.d/51-android.rules; sudo udevadm control --reload-rules
```

Insert the USB cable(not MicroUSB Cable) into your board.

Enter u-boot shell mode during boot countdown:
```
Net:   No ethernet found.
Hit any key to stop autoboot:  0
ARTIK710 #
ARTIK710 #
ARTIK710 # fastboot 0
```

+ For artik710>
You'll need to upload the partmap_emmc.txt prior than uploading the binaries.
It can be downloaded from boot-firmwares-artik710.
On your Host PC(Linux), flash the u-boot using below command
```
sudo fastboot flash partmap partmap_emmc.txt
sudo fastboot flash fip-nonsecure fip-nonsecure.img
sudo fastboot flash env params.bin
sudo fastboot reboot
```

+ For artik530>
You'll need to upload the partmap_emmc.txt prior than uploading the binaries.
It can be downloaded from boot-firmwares-artik530.
On your Host PC(Linux), flash the u-boot using below command
```
sudo fastboot flash partmap partmap_emmc.txt
sudo fastboot flash bootloader bootloader.img
sudo fastboot flash env params.bin
sudo fastboot reboot
```

### 3.2 microSD Card
Prepare a micro SD card and format to ext4 file system.
```
sudo mkfs.ext4 /dev/sd[X]1
sudo mount /dev/sd[X]1 /mnt
```

+ For artik710>
Copy compiled binaries(fip-nonsecure.img, params.bin) into a micro sd card.
```
sudo cp fip-nonsecure.img params.bin partmap_emmc.txt /mnt
sudo umount /mnt
```

+ For artik530>
Copy compiled binaries(bootloader.img, params.bin) into a micro sd card.
```
sudo cp bootloader.img params.bin partmap_emmc.txt /mnt
sudo umount /mnt
```

Insert the microSD card into your board and enter u-boot shell during boot countdown
```
sd_recovery mmc 1:1 48000000 partmap_emmc.txt
```
