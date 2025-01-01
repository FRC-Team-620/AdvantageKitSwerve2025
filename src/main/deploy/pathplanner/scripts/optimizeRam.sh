#!/bin/bash

# Disable System Webserver
/etc/init.d/systemWebServer stop;
update-rc.d -f systemWebServer remove;
sync;

chmod a-x /usr/local/natinst/etc/init.d/systemWebServer;
sync;

# Enable System Webserver

# update-rc.d -f systemWebServer defaults; /etc/init.d/systemWebServer start; sync
# chmod a+x /usr/local/natinst/etc/init.d/systemWebServer; sync


# Set Linux Kernel options (sysctls)
echo "vm.overcommit_memory=1" >> /etc/sysctl.conf
echo "vm.vfs_cache_pressure=1000" >> /etc/sysctl.conf
echo "vm.swappiness=100" >> /etc/sysctl.conf
sync


# Setup Swap File on USB stick TODO:

# fallocate -l 100M /u/swapfile
# mkswap /u/swapfile
# swapon /u/swapfile
# vi /etc/init.d/addswap.sh
# chmod a+x /etc/init.d/addswap.sh
# update-rc.d -v addswap.sh defaults
# sync

# File Contents

# #!/bin/sh
# [ -x /sbin/swapon ] && swapon -e /u/swapfile
# : exit 0

# Revert Swap
# update-rc.d -f addswap.sh remove; rm /etc/init.d/addswap.sh; sync; reboot