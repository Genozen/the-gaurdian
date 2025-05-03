#!/bin/bash

# Optional: bring WiFi interface up first
nmcli radio wifi on
sleep 2

# Start the hotspot
nmcli device wifi hotspot ifname wlP1p1s0 ssid JetsonNet password guardian123
