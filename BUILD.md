# Windows

First, install the rust build environment. Follow the instructions at
http://rustup.rs . You will also need CMake.

Create a build directory

    mkdir build

Configure the project

    cd build
    cmake .. -G "Visual Studio 15 2017" -DCMAKE_BUILD_TYPE=Release

Build the project

    cmake --build . --config release
