# Bee@C3LAB firmware baseline

Source: https://github.com/espressif/esp-drone

Baseline commit: `db0f6562e4f67cccee3acac4dff39c9aaea4e5fa`

Toolchain: ESP-IDF `v5.0.9` through the official Docker image.

Managed dependencies: ESP-NOW `2.4.0`, cmake_utilities `0.5.3`.
Upstream's unbounded ESP-NOW requirement currently selects a release that
does not compile against IDF 5.0; the compatible versions are pinned here.

Bee changes add station-mode Wi-Fi with DHCP and automatic reconnection.
The UDP CRTP endpoint remains port 2390. Station mode is the default;
the upstream AP mode remains selectable in menuconfig.
