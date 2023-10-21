#!/bin/bash

# Set net.core.rmem_max and net.core.rmem_default
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400

# Set net.ipv4.udp_mem
sudo sysctl -w net.ipv4.udp_mem=26214400
