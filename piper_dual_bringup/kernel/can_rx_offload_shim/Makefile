# Build the upstream rx-offload.c as a standalone prerequisite for gs_usb.
obj-m += can_rx_offload_shim.o
can_rx_offload_shim-y := module.o rx-offload.o
ccflags-y += -include $(src)/shim.h
