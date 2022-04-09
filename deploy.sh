#!/bin/bash

export PATH=~/gcc-arm-none-eabi-6-2017-q2-update/bin/:$PATH

python2 -m pip uninstall -y pandas uavcan 
python2 -m pip install setuptools wheel virtualenv
python2 -m pip install pandas==0.21.0 tabulate natsort uavcan==1.0.0.dev32 crcmod empy
python2 -m pip install tokenize

rm -rf deploy
mkdir deploy

BOARDS="com.hex.here+_2.1 com.hex.here_2.1 com.hex.here_3.0"

make clean

for board in $BOARDS
do
    make clean
    make -j12 BOARD_DIR=boards/$board
    cp build/$board/Here2_$board-crc.bin build/$board/Here2_$board-combined.bin build/$board/Here2_$board.elf build/$board/Here2_$board-bootloader.bin build/$board/Here2_$board-bootloader.elf deploy
done
