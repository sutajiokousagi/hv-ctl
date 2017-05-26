# hv-ctl

This build:

*Upgrade to stretch
*apt-get install cargo apache2
*enable SPI in raspi-config
*had to pull down cupi, modify board code to recognize updated CPU string, map BCM2835 to BCM2709 (due to difference in /proc/cpuinfo reporting on stretch vs jessie)
*had to pull down cupi_shift and have its Cargo.toml map to the locally changed version of cupi
*of course also had to set hv-ctl's Cargo.toml to map to locally changed version of cupi
*install libssl-dev for the web-ui version

Other things included having to install emacs and the rust-mode
