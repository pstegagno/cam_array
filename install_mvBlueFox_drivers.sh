#!/bin/sh


ARM_TAR="mvBlueFOX-ARMhf_gnueabi-2.13.5.tgz"
ARM_FOLDER="mvIMPACT_Acquire-ARMhf-2.13.5"
X86_64_TAR="mvBlueFOX-x86_64_ABI2-2.13.5.tgz"
X86_64_FOLDER="mvIMPACT_acquire-x86_64-2.13.5"
COMMON_FOLDER="mvIMPACT_Acquire"


echo "Select action:"
echo "  1 - ARM installation"
echo "  2 - x86 64bit installation"
echo "  3 - clean previous installations"

read PTF

if [ $PTF  -eq 1 ]
then
  echo "Install drivers for ARM (1 - yes / 2 - no)?"
    read ANSWER
    if [ $ANSWER  -eq 1 ]
    then
      echo "Incoming installation."
      tar -zxvf $ARM_TAR 
      mv $ARM_FOLDER $COMMON_FOLDER 
      echo "" >> $COMMON_FOLDER/PLATFORM_ARM
    else
      echo "Ok, bye."
    fi
elif [ $PTF -eq 2 ]
then
  echo "Install drivers for x86 64bit (1 - yes / 2 - no)?"
    read ANSWER
    if [ $ANSWER  -eq 1 ]
    then
      echo "Incoming installation."
      tar -zxvf $X86_64_TAR
      mv $X86_64_FOLDER $COMMON_FOLDER 
      echo "" >> $COMMON_FOLDER/PLATFORM_x86_64
    else
      echo "Ok, bye."
    fi
elif [ $PTF -eq 3 ]
then
  echo "Clean previous installations (1 - yes / 2 - no)?"
    read ANSWER
    if [ $ANSWER  -eq 1 ]
    then
#      rm PLATFORM_x86_64
#      rm PLATFORM_ARM
      rm -rf $COMMON_FOLDER
    else
      echo "Ok, bye."
    fi
else
  echo "Unrecognized action.
Please be serious while doing this step or YOUR PC MIGHT EXPLODE!!!."
fi


#tar -zxvf mvBlueFOX-x86_64_ABI2-2.11.3.tgz
#cd mvIMPACT_acquire-x86_64-2.11.3/lib/x86_64/
#sudo cp * /usr/lib/
