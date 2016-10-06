#!/bin/sh

# Copyright (C) 2011 Twisted Playground

# This script is designed by Twisted Playground for use on MacOSX 10.7 but can be modified for other distributions of Mac and Linux

PROPER=`echo $1 | sed 's/\([a-z]\)\([a-zA-Z0-9]*\)/\u\1\2/g'`

HANDLE=TwistedZero
KERNELSPEC=/Volumes/android/trltetmo-kernel
KERNELREPO=$DROPBOX_SERVER/TwistedServer/Playground/kernels
TOOLCHAIN_PREFIX=/Volumes/android/android-toolchain-eabi-4.7/bin/arm-eabi-
MODULEOUT=$KERNELSPEC/buildimg/boot.img-ramdisk
GOOSERVER=loungekatt@upload.goo.im:public_html
PUNCHCARD=`date "+%m-%d-%Y_%H.%M"`

CPU_JOB_NUM=8

if [ -e $KERNELSPEC/out ]; then
rm -R $KERNELSPEC/out
fi
if [ -e $KERNELSPEC/buildimg/boot.img ]; then
rm -R $KERNELSPEC/buildimg/boot.img
fi
if [ -e $KERNELSPEC/buildimg/newramdisk.cpio.gz ]; then
rm -R $KERNELSPEC/buildimg/newramdisk.cpio.gz
fi
if [ -e $KERNELSPEC/buildimg/zImage ]; then
rm -R $KERNELSPEC/buildimg/zImage
fi

cp -R config/apq8084_sec_trlte_tmo_defconfig  arch/arm/configs/apq8084_sec_trlte_tmo_defconfig

mkdir $(pwd)/out
make -j$CPU_JOB_NUM -C $(pwd) O=$(pwd)/out VARIANT_DEFCONFIG=apq8084_sec_trlte_tmo_defconfig apq8084_sec_defconfig SELINUX_DEFCONFIG=selinux_defconfig CROSS_COMPILE=$TOOLCHAIN_PREFIX
make -j$CPU_JOB_NUM -C $(pwd) O=$(pwd)/out CROSS_COMPILE=$TOOLCHAIN_PREFIX
cp $(pwd)/out/arch/arm/boot/zImage $(pwd)/arch/arm/boot/zImage

if [ -e arch/arm/boot/zImage ]; then

    cp -R arch/arm/boot/zImage buildimg

    cd buildimg
    ./img.sh

    echo "building boot package"
    cp -R boot.img ../output
    cd ../

    if [ -e output/boot.tar ]; then
        rm -R output/boot.tar
    fi
    if [ -e output/boot.tar ]; then
        rm -R output/boot.tar.md5
    fi
    if [ -e output/boot.tar ]; then
        rm -R output/boot.tar.md5.gz
    fi

    IMAGEFILE=boot.$PUNCHCARD.img
    KERNELFILE=boot.$PUNCHCARD.tar

    cp -r  output/boot.img $KERNELREPO/trltetmo/boot.img
    cp -r  $KERNELREPO/trltetmo/boot.img $KERNELREPO/gooserver/$IMAGEFILE
    scp $KERNELREPO/gooserver/$IMAGEFILE $GOOSERVER/trltetmo/kernel

#    if cat /etc/issue | grep Ubuntu; then
#        tar -H ustar -c output/boot.img > output/boot.tar
#    else
#        tar --format ustar -c output/boot.img > output/boot.tar
#    fi
    tar cvf output/boot.tar output/boot.img
    rm -R $KERNELREPO/gooserver/$IMAGEFILE
    cp -r output/boot.tar $KERNELREPO/trltetmo/boot.tar
    cp -r $KERNELREPO/trltetmo/boot.tar $KERNELREPO/gooserver/$KERNELFILE
    scp $KERNELREPO/gooserver/$KERNELFILE $GOOSERVER/trltetmo/kernel
    rm -R $KERNELREPO/gooserver/$KERNELFILE
    cp -r output/boot.tar output/boot.tar.md5
    if cat /etc/issue | grep Ubuntu; then
        md5sum -t output/boot.tar.md5 >> output/boot.tar.md5
    else
        md5 -r output/boot.tar.md5 >> output/boot.tar.md5
    fi
# gzip output/boot.tar.md5 -c -v > output/boot.tar.md5.gz
    cp -r output/boot.tar.md5 $KERNELREPO/trltetmo/boot.tar.md5
    cp -r $KERNELREPO/trltetmo/boot.tar.md5 $KERNELREPO/gooserver/$KERNELFILE.md5
    scp $KERNELREPO/gooserver/$KERNELFILE.md5 $GOOSERVER/trltetmo/kernel
    rm -R $KERNELREPO/gooserver/$KERNELFILE.md5
#fi

fi

cd $KERNELSPEC
