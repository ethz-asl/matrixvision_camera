#!/bin/bash

echo "###### copying udev rule to /etc/udev/rules.d ######"

set -e

PACKAGE_DIR=$(rospack find mv_driver)

sudo cp $PACKAGE_DIR/mvBlueFOX_scripts/51-mvbf.rules /etc/udev/rules.d

sudo service udev reload

echo "###### now unplug the camera and plug it back in in case it was connected while executing this script ######"