#!/bin/bash
set -e

kinematic_type=$1 # 'diff' or 'mecanum'
sys_arch=$(uname -m)

echo $sys_arch
case $sys_arch in
x86_64)
    sudo stm32loader -c upboard -u -W
    sleep 1
    sudo stm32loader -c upboard -e -w -v /root/firmware_${kinematic_type}.bin
    ;;
armv7l)
    sudo stm32loader -c tinker -u -W
    sleep 1
    sudo stm32loader -c tinker -e -w -v /root/firmware_${kinematic_type}.bin
    ;;
esac