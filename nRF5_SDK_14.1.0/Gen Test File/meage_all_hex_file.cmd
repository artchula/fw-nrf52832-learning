mergehex -m "..\src_aider\ble_peripheral\ble_app_aider_a2_dfu_tester\pca10040\s132\arm5_no_packs\_build\nrf52832_xxaa.hex" "..\components\softdevice\s132\hex\s132_nrf52_5.0.0_softdevice.hex" ..\src_aider\dfu\bootloader_secure_ble\pca10040\arm5_no_packs\_build\nrf52832_xxaa_s132.hex -o sf_app_test_bl.hex
mergehex -m sf_app_test_bl.hex settings.hex -o sf_app_test_bl_setting.hex