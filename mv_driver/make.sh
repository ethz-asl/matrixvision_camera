#!/bin/bash

set -e

PACKAGE_DIR=$(rospack find mv_driver)
echo "### downloading and unpacking drivers to" $PACKAGE_DIR "###"


GENTL_URL="http://n.ethz.ch/~acmarkus/download/matrixvision_driver"
BLUEFOX_URL="http://n.ethz.ch/~acmarkus/download/matrixvision_driver"

#BLUEFOX_FILE=""
#GENTL_FILE=""

API=mvIMPACT_acquire

VERSION=2.5.2
BLUEFOX_VERSION=2.5.4
ABI=ABI2

LINKER_PATHS=$PACKAGE_DIR/linker_paths
COMPILER_FLAGS=$PACKAGE_DIR/compile_flags
ENV_VARS=$PACKAGE_DIR/envvars

TARGET=$(uname -m)
if [ "$TARGET" == "i686" ]; then
    TARGET=x86
else
    TARGET=x86_64
fi

GENTL_NAME=mvGenTL_Acquire
GENTL_TARNAME=$GENTL_NAME-$TARGET"_"$ABI-$VERSION

BLUEFOX_NAME=mvBlueFOX
BLUEFOX_TARNAME=$BLUEFOX_NAME-$TARGET"_"$ABI-$BLUEFOX_VERSION


# cleanup first
rm -rf $GENTL_NAME* $BLUEFOX_NAME* $API* tmp $LINKER_PATHS $COMPILER_FLAGS $ENV_VARS
rm -rf download

#### download driver archives ####
mkdir -p download
cd download
wget -O $GENTL_TARNAME.tar -nc $GENTL_URL/$GENTL_TARNAME.tar 
wget -O $BLUEFOX_TARNAME.tar -nc $BLUEFOX_URL/$BLUEFOX_TARNAME.tar


#### unpack blueCougar ####

mkdir -p $PACKAGE_DIR/tmp/$GENTL_NAME
cd $PACKAGE_DIR/tmp/$GENTL_NAME
tar -xf $PACKAGE_DIR/download/$GENTL_TARNAME.tar --overwrite

#### BlueCougar runtime stuff ####

cd $PACKAGE_DIR
mkdir -p $GENTL_NAME"_runtime"
cd $GENTL_NAME"_runtime"
tar xf $PACKAGE_DIR/tmp/$GENTL_NAME/$GENTL_NAME"_"runtime-$VERSION.tar --overwrite

cd lib
# kick out devicemanager and prophandling, since this is not device specific
rm -f libmvDeviceManager* 
rm -f libmvPropHandling*

# those should not be necessary anymore
#ln -fs libmvBlueCOUGAR.so.$VERSION libmvBlueCOUGAR.so
#ln -fs libmvTLIClientGigE.so mvTLIClientGigE.cti
#ln -fs libmvTLIClientGigE.so.$VERSION libmvTLIClientGigE.so
rm -f libmvBlueCOUGAR*
rm -f libmvTLIClientGigE*

# create missing symlinks
ln -fs libmvGenTLConsumer.so.$VERSION libmvGenTLConsumer.so
ln -fs libmvGenTLProducer.so.$VERSION libmvGenTLProducer.so
ln -fs libmvGenTLProducer.so libmvGenTLProducer.cti
#ln -fs libmvPropHandling.so.$VERSION libmvPropHandling.so
echo -L$PACKAGE_DIR/$GENTL_NAME"_runtime/lib" >> $LINKER_PATHS

#### mvImpact (device independent stuff) ####
cd $PACKAGE_DIR
tar xf $PACKAGE_DIR/tmp/$GENTL_NAME/$API-$VERSION.tar --overwrite
mv $API-$VERSION $API
echo -L$PACKAGE_DIR/$API/lib/$TARGET >> $LINKER_PATHS
echo -I$PACKAGE_DIR/$API >> $COMPILER_FLAGS

#### GenICam runtime/compile time stuff ####

cd $PACKAGE_DIR/$GENTL_NAME"_runtime"
mkdir -p GenICam
cd GenICam
# the runtime tar contains either the i86 or the x64 tar
if [ -r ../GenICam_Runtime_gcc40_Linux32_i86_v*.tgz ]; then
       
   if [ x$TARGET != xx86 ]; then
      echo 'Platform conflict : GenICam runtime is 32bit, but target is 64bit'
      exit 1
   fi
   tar xfz ../GenICam_Runtime_gcc40_Linux32_i86_v*.tgz;
   echo -L$PACKAGE_DIR/$GENTL_NAME"_runtime"/GenICam/bin/Linux32_i86 >> $LINKER_PATHS
   echo -L$PACKAGE_DIR/$GENTL_NAME"_runtime"/GenICam/bin/Linux32_i86/GenApi/Generic >> $LINKER_PATHS
fi
if [ -r ../GenICam_Runtime_gcc40_Linux64_x64_v*.tgz ]; then
   if [ x$TARGET = xx86 ]; then
      echo 'Platform conflict : GenICam runtime is 64bit, but target is 32bit'
      exit 1
   fi
   tar xfz ../GenICam_Runtime_gcc40_Linux64_x64_v*.tgz;
   echo -L$PACKAGE_DIR/$GENTL_NAME"_runtime"/GenICam/bin/Linux64_x64 >> $LINKER_PATHS
   echo -L$PACKAGE_DIR/$GENTL_NAME"_runtime"/GenICam/bin/Linux64_x64/GenApi/Generic >> $LINKER_PATHS
fi

# pass the genicam paths to the compiler, such that the app can setup the environment itself
echo -DGENICAM_ROOT=$PACKAGE_DIR/$GENTL_NAME"_runtime"/GenICam >> $COMPILER_FLAGS
echo -DGENICAM_ROOT_V2_3=$PACKAGE_DIR/$GENTL_NAME"_runtime"/GenICam >> $COMPILER_FLAGS
echo -DGENICAM_GENTL64_PATH=$PACKAGE_DIR/$GENTL_NAME"_runtime"/lib >> $COMPILER_FLAGS
echo -DGENICAM_LOG_CONFIG_V2_3=$PACKAGE_DIR/$GENTL_NAME"_runtime"/GenICam/log/config-unix/DefaultLogging.properties >> $COMPILER_FLAGS

echo export GENICAM_ROOT=$PACKAGE_DIR/$GENTL_NAME"_runtime"/GenICam >> $ENV_VARS
echo export GENICAM_ROOT_V2_3=$PACKAGE_DIR/$GENTL_NAME"_runtime"/GenICam >> $ENV_VARS
echo export GENICAM_GENTL64_PATH=$PACKAGE_DIR/$GENTL_NAME"_runtime"/lib >> $ENV_VARS
echo export GENICAM_LOG_CONFIG_V2_3=$PACKAGE_DIR/$GENTL_NAME"_runtime"/GenICam/log/config-unix/DefaultLogging.properties >> $ENV_VARS


#### bluefox runtime ####
# unpack
mkdir -p $PACKAGE_DIR/tmp/$BLUEFOX_NAME
cd $PACKAGE_DIR/tmp/$BLUEFOX_NAME
tar -xf $PACKAGE_DIR/download/$BLUEFOX_TARNAME.tar --overwrite

# copy blueFOX runtime libs
mkdir -p $PACKAGE_DIR/$BLUEFOX_NAME"_runtime/lib/"$TARGET
cp $PACKAGE_DIR/tmp/$BLUEFOX_NAME/$API-$TARGET-$BLUEFOX_VERSION/lib/$TARGET/libmvBlueFOX.* $PACKAGE_DIR/$BLUEFOX_NAME"_runtime/lib/"$TARGET

# copy udev rule
mkdir -p $PACKAGE_DIR/$BLUEFOX_NAME"_scripts"
cp $PACKAGE_DIR/tmp/$BLUEFOX_NAME/$API-$TARGET-$BLUEFOX_VERSION/Scripts/51-mvbf.rules $PACKAGE_DIR/$BLUEFOX_NAME"_scripts"

echo -L$PACKAGE_DIR/$BLUEFOX_NAME"_runtime/lib/"$TARGET >> $LINKER_PATHS

#### clean up ####
rm -rf $PACKAGE_DIR/tmp

#### note down that this is done ####
touch $PACKAGE_DIR/downloadandinstall

