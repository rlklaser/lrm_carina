echo reseting firewire
sudo rmmod firewire_ohci
sleep 1s
sudo rmmod firewire_core
sleep 1s
sudo modprobe firewire_core
sleep 1s
sudo modprobe firewire_ohci
echo wait...
sleep 3s
dmesg
