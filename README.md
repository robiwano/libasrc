Asynchronous Sample Rate Converter (libasrc)
=====================================

IMPORTANT: This project is work in progress! Do NOT expect it to work at all!

Introduction
------------

libasrc is a Asynchronous Sample Rate Converter based on Erik de Castro Lopos libsamplerate. It wraps libsamplerate to handle varying input and output samplerates. It should be perfect to use for piping audio data from USB to a soundcard where sample rates do not match exactly (clock drift).


Dependencies
------------

* A modern C/C++ compiler, tested for MSVC 14 (VS2015) and XCode 9.
* CMake version 3.4 or later.

Building
--------

### Building on Windows

Provided that all system dependencies are install correctly, the building can
be done by following the steps below.

First clone the repository and update the submodules:
```bash
git clone https://github.com/robiwano/libasrc.git
cd libasrc
git submodule update --init
cd ..
```

Then configure cmake and build:
```bash
mkdir build
cd build
call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" amd64
cmake -GNinja ../libasrc
ninja
```
