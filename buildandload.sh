#!/bin/bash

rm -rf build && mkdir build && cd build && cmake .. && make -j 4 && cd ..
sudo picotool load build/agent-reconnect.uf2 -f
