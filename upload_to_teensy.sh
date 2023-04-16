mkdir out
cargo objcopy --release -- -O ihex out/light.hex
teensy_loader_cli -v --mcu=TEENSY40 -w out/light.hex