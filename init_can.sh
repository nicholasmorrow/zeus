ip link set can0 down
modprobe can_raw
modprobe can_dev
insmod usb_8dev.ko
ip link set can0 up type can bitrate 500000 sample-point 0.875
