export PICO_SDK_PATH=$HOME/.pico-sdk/sdk/2.3.1 PATH=$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$HOME/.pico-sdk/ninja/v1.13.2:$PATH
$HOME/.pico-sdk/cmake/v4.3.4/bin/cmake -S . -B build -G Ninja && ninja -C build
$HOME/.pico-sdk/picotool/2.3.1/picotool/picotool load -x build/rabbit-firmware.uf2