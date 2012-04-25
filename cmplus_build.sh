#!/bin/bash
#

START=$(date)

# Initialize all arguments here ...
#
echo "*** Customizing settings ..."
echo ""
CONFIG="CMPlus-Tuna"
MY_BUILD_DIR=$(pwd)
MY_DEST_DIR=$HOME/Dropbox/GNex/kernel
MY_SIGNAPK=$HOME/Dropbox/GNex/signapk
MY_KFLAGS="-march=armv7-a -mtune=cortex-a9 -mfpu=neon"
export ARCH=arm
export KCFLAGS=$MY_KFLAGS
export KAFLAGS=$MY_KFLAGS
#echo "*** Running make clean ..."
#make clean
#echo
echo "Please select toolchain ..."
echo "   [1] Google GCC-4.4.3"
echo "   [2] Custom GCC-4.6.3"
echo "   [3] Custom GCC-4.7.0"
echo "   [4] Custom GCC-4.8.0"
echo "   [5] Linaro GCC-4.5.4"
echo "   [*] Linaro GCC-4.6.3"
read choice;
case $choice in
	"1") echo "*** Using Google GCC-4.4.3 toolchain ...";
	     MY_CC=$HOME/toolchain/arm-eabi-4.4.3/bin/arm-eabi-;
	     CONFIG="$CONFIG-gcc443";;

	"2") echo "*** Using Custom GCC-4.6.3 toolchain ...";
	     MY_CC=$HOME/toolchain/x-tools/gcc-4.6.3/bin/arm-tuna-eabi-;
	     CONFIG="$CONFIG-gcc463";;

	"3") echo "*** Using Custom GCC-4.7.0 toolchain ...";
	     MY_CC=$HOME/toolchain/x-tools/gcc-4.7.0/bin/arm-tuna-eabi-;
	     CONFIG="$CONFIG-gcc470";;

	"4") echo "*** Using Custom GCC-4.8.0 toolchain ...";
	     MY_CC=$HOME/toolchain/x-tools/gcc-4.8.0/bin/arm-tuna-eabi-;
	     CONFIG="$CONFIG-gcc480";;

	"5")   echo "*** Using Custom Linaro-4.5.4 toolchain ...";
	     MY_CC=$HOME/toolchain/x-tools/linaro-4.5.4/bin/arm-tuna-eabi-;
	     CONFIG="$CONFIG-lin454";;

	*)   echo "*** Using Custom Linaro-4.6.3 toolchain ...";
	     MY_CC=$HOME/toolchain/x-tools/linaro-4.6.3/bin/arm-tuna-eabi-;
	     CC="lin463";;
esac
export CROSS_COMPILE=$MY_CC
VERSION=`cat .config | grep Linux | awk '{print $(3)}'`
DTSTAMP=$(date +%y%m%d%H%M);
if [ $1 ]
then
	FILENAME="$VERSION-$CONFIG-$DTSTAMP-$1";
else
	FILENAME="$VERSION-$CONFIG-$DTSTAMP";
fi;
echo `expr $DTSTAMP "-" 1` > .version
sed -i "s/^CONFIG_LOCALVERSION.*/CONFIG_LOCALVERSION=\"-$CONFIG\"/g" .config
echo ""
#make clean
echo "*** Running make ..."
echo ""
#make -j8 2> error.log
make -j8
if [ -e arch/arm/boot/zImage ]
then
	echo ""
	echo "*** Making external modules ..."
	echo ""
	cp -av $MY_DEST_DIR/template-cmplus $MY_BUILD_DIR/$DTSTAMP >> build.log
	cp -av arch/arm/boot/zImage $MY_BUILD_DIR/$DTSTAMP/kernel/ >> build.log
	make modules_install INSTALL_MOD_PATH=./temp >> build.log
	cp -av temp/lib/modules/$VERSION-$CONFIG/kernel/* $MY_BUILD_DIR/$DTSTAMP/system/modules >> build.log
	rm -r temp
	echo ""
	echo "*** Creating signed zip file for recovery ..."
	echo ""
	cd $MY_BUILD_DIR/$DTSTAMP
	#echo "ui_print(\"Successfully installed ...\");" >> META-INF/com/google/android/updater-script
	#echo "ui_print(\"$FILENAME.zip\");" >> META-INF/com/google/android/updater-script
	zip -rTy $FILENAME.zip * >> build.log
	java -Xmx512m -jar $MY_SIGNAPK/signapk.jar -w $MY_SIGNAPK/testkey.x509.pem $MY_SIGNAPK/testkey.pk8 $FILENAME.zip ../$FILENAME.zip
	cd $MY_BUILD_DIR
	mv $FILENAME.zip $MY_DEST_DIR/. >> build.log
	rm -r $DTSTAMP
	echo "";
	echo "[ Build STARTED : $START ]"
	echo "[ Build SUCCESS : $(date) ]"
	echo ""
	echo "[ Recovery zip  : $MY_DEST_DIR/$FILENAME.zip ]"
	echo ""
else
	echo ""
	echo "[ Build STARTED : $START ]"
	echo "[ Build FAILED! : $(date) ]"
	echo ""
fi;

