#!/bin/bash

tmux new-session -d -s my_session 'ssh root@192.168.7.1' \; \
  sleep 1 \; \
  send-keys 'firmware print_logs' C-m \; \
  split-window -h 'ssh root@192.168.7.1' \; \
  sleep 1 \; \
  send-keys 'firmware_bridge' C-m \; \
  split-window -h 'ssh root@192.168.7.1' \; \
  sleep 1 \; \
  send-keys 'legacy_compatibility_interface' C-m \; \ 
  split-window -h 'ssh root@192.168.7.1' \; \
  select-layout even-vertical \; \
  attach-session -d