#!/bin/bash
# troubleshoot_serial.sh
# Check and fix common serial port issues for FormicaBot

echo "--- FormicaBot Serial Health Check ---"

PORTS=("/dev/ttyUSB0" "/dev/ttyUSB1" "/dev/ttyCH341USB0")

for PORT in "${PORTS[@]}"; do
    if [ -e "$PORT" ]; then
        echo "Found $PORT"
        echo "  Permissions: $(ls -l $PORT)"
        echo "  Owner: $(stat -c '%U:%G' $PORT)"
    else
        echo "Missing $PORT - Is it plugged in?"
    fi
done

echo "--- Checking User Groups ---"
groups $USER | grep -q "dialout"
if [ $? -eq 0 ]; then
    echo "SUCCESS: User is in dialout group."
else
    echo "WARNING: User is NOT in dialout group. Run: sudo usermod -a -G dialout $USER"
fi

echo "--- Suggested Permanent Fix (udev rules) ---"
echo 'To fix permissions permanently, create /etc/udev/rules.d/99-formica.rules with:'
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar"'
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", SYMLINK+="formica"'

echo "--- Recent Serial Errors (dmesg) ---"
dmesg | grep -iE "usb|tty" | tail -n 5

echo "--- Applying Temporary Permission Fix ---"
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyCH341USB0 2>/dev/null
echo "Done."
