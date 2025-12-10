nrfutil device erase
nrfutil device program --firmware bicr.hex --options  chip_erase_mode=ERASE_NONE,verify=VERIFY_READ --core Application --x-family nrf54h
nrfutil device program --firmware uicr.hex --options  chip_erase_mode=ERASE_NONE,verify=VERIFY_READ --core Application --x-family nrf54h
nrfutil device program --firmware periphconf.hex --options  chip_erase_mode=ERASE_NONE,verify=VERIFY_READ --core Application --x-family nrf54h
nrfutil device program --firmware bap_unicast_client.hex --options  chip_erase_mode=ERASE_NONE,verify=VERIFY_READ --core Application --x-family nrf54h
nrfutil device program --firmware ipc_radio.hex --options  chip_erase_mode=ERASE_NONE,verify=VERIFY_READ --core Application --x-family nrf54h
nrfutil device reset
PAUSE