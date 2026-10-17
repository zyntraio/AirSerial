idf_component_register(SRCS "bridge_usb_host.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bridge_core usb usb_host_cdc_acm usb_host_ch34x_vcp)
