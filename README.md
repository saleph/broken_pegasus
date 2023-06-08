# broken_pegasus
Emulator of Pegasus console (knock-off of Nintendo Famicon) with options to model various HW issues (increased delays, random bit flipping). 

```
sudo apt install g++-13 gcc-13 pip cmake build-essential mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev libao-dev libmpg123-dev cmake pkg-config libxcomposite-dev libxcursor-dev libxdamage-dev libxfixes-dev libxi-dev libxinerama-dev libxkbfile-dev libxmuu-dev libxres-dev libxss-dev libxtst-dev libxv-dev libxvmc-dev libxxf86vm-dev libxcb-render0-dev libxcb-render-util0-dev libxcb-xkb-dev libxcb-icccm4-dev libxcb-image0-dev libxcb-keysyms1-dev libxcb-randr0-dev libxcb-shape0-dev libxcb-sync-dev libxcb-xfixes0-dev libxcb-xinerama0-dev libxcb-dri3-dev uuid-dev libxcb-cursor-dev libxcb-util-dev libxcb-util0-dev
export CONAN_VENV='~/.python/venv/conan'
python -m venv ${CONAN_VENV}
export PATH=${CONAN_VENV}/bin:$PATH
sudo pip install conan

mkdir build
conan install . --output-folder=build --build=missing
cmake -S . -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release -B build
cmake --build build
./build/tests
```