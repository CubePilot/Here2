#!/bin/bash

export PATH=~/gcc-arm-none-eabi-6-2017-q2-update/bin/:$PATH

rm -rf deploy
mkdir deploy

BOARDS="com.hex.here_3.0 com.hex.here+_2.1 com.hex.here_2.1"

for board in $BOARDS
do
    make clean && make -j12 BOARD_DIR=boards/$board
    cp build/$board/Here2_$board-crc.bin build/$board/Here2_$board-combined.bin build/$board/Here2_$board.elf build/$board/Here2_$board-bootloader.bin build/$board/Here2_$board-bootloader.elf deploy
done
