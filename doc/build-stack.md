Building openPOWERLINK Stack Libraries {#page_build_stack}
======================================

[TOC]

# Building Stack Libraries{#sect_build_stack_build}

The openPOWERLINK stack itself is implemented via stack libraries. A stack
library can either contain the whole stack (_complete library_), the user part
(_application library_) or the kernel part (_driver library_).

__NOTE:__ In order to be able to build an application, both, the Debug and the
Release library versions must be available. If one of the libraries is missing
you get an OPLKLIB-NOTFOUND or OPLKLIB_DEBUG-NOTFOUND error in the demos
CMake configuration.

## Linux {#sect_build_stack_build_linux}

Follow the steps below to build the stack on a Linux system. On Linux, CMake
generates Makefiles by default.

* Creating debug libraries

      > cd openPOWERLINK/stack/build/linux
      > cmake -DCMAKE_BUILD_TYPE=Debug ../..
      > make
      > make install

* Creating release libraries

      > cd openPOWERLINK/stack/build/linux
      > cmake -DCMAKE_BUILD_TYPE=Release ../..
      > make
      > make install

The default library installation path is: `<openPOWERLINK_DIR>/lib/linux/<ARCH>`


## Windows {#sect_build_stack_build_windows}

Follow the steps below to build the stack on a Windows system using NMake.
Open a Visual Studio command line and enter the following commands:

* Creating debug libraries

      > cd <openPOWERLINK_directory>\stack\build\windows
      > cmake -G"NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug ..\..
      > nmake
      > nmake install

* Creating release libraries

      > cd <openPOWERLINK_directory>\stack\build\windows
      > cmake -G"NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ..\..
      > nmake
      > nmake install

The default library installation path is: `<openPOWERLINK_DIR>\lib\windows\<ARCH>`

__NOTE:__ You can also generate a Visual Studio Solution and compile the
libraries in Visual Studio. Please refer to the CMAKE documentation for
generating Visual Studio solution files.

## Embedded Systems (No-OS) {#sect_build_stack_build_noos}

### Microblaze {#sect_build_stack_build_microblaze}

Follow the steps below to build the stack library on your host platform:
* Open a shell where the Xilinx ISE 14.7 Toolchain is configured.
  - On a Windows host platform open the `ISE Design Suite [64,32] Bit Command
    Prompt`.
  - On a Linux host platform execute the script `<ISE_ROOT_DIR>/settings[32,64].sh>`
    to configure your current shell.

* Creating debug libraries

      > cd <openPOWERLINK_directory>\stack\build\xilinx-microblaze
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Debug
      > make all
      > make install

* Creating release libraries

      > cd <openPOWERLINK_directory>\stack\build\xilinx-microblaze
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Release
      > make all
      > make install

The default library installation path is:
`<openPOWERLINK_DIR>/stack/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`

### Xilinx Zynq ARM {#sect_build_stack_build_zynqarm-xilinx}

Follow the steps below to build the stack library on your host platform:
* Open a shell where the Xilinx ISE 14.7 Toolchain is configured.
  - On a Windows host platform open the `ISE Design Suite [64,32] Bit Command
    Prompt`.
  - On a Linux host platform execute the script `<ISE_ROOT_DIR>/settings[32,64].sh>`
    to configure your current shell.

* Creating debug libraries

      > cd <openPOWERLINK_directory>\stack\build\xilinx-zynqarm
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-zynqarm-eabi-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Debug
      > make all
      > make install

* Creating release libraries

      > cd <openPOWERLINK_directory>\stack\build\xilinx-zynqarm
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain-xilinx-zynqarm-eabi-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Release
      > make all
      > make install

The default library installation path is:
`<openPOWERLINK_DIR>/stack/lib/generic/xilinx-zynqarm/<BOARD_NAME>/<DEMO_NAME>`

# Configuration Options {#sect_build_stack_options}

If you would like to change the configuration options you have to provide
the configuration options on the command line (-DCFG_XXX=XXX) or
[call CMake in an interactive mode](\ref sect_build_cmake_interactive).

## Generic Configuration Options

- **CFG_DEBUG_LVL**

  Debug level to be used for openPOWERLINK debugging functions.

- **CMAKE_INSTALL_PREFIX**

  Specifies the installation directory where your files will be installed.

  Default directory is: `<openPOWERLINK_DIR>/lib/${SYSTEM_DIR_NAME}/${CMAKE_SYSTEM_PROCESSOR}`

- **CMAKE_BUILD_TYPE**

  Specifies your build type.
  Valid build types are: _Debug_, _Release_

  If the build type _Debug_ is specified, the code is compiled with debugging
  options.

- **CMAKE_TOOLCHAIN_FILE**

  Specifies a cross toolchain file to be used. It is only needed if you
  cross-compile for another target platform. (\ref sect_build_cmake_crosscompile)

## Linux Configuration Options

- **CFG_COMPILE_LIB_MN**

  Compile complete openPOWERLINK MN library. The library contains an Ethernet
  driver which is using the PCAP library for accessing the network.

- **CFG_COMPILE_LIB_MNAPP_USERINTF**

  Compile openPOWERLINK MN application library which contains the interface
  to a Linux user space driver. It is used for implementing a multi-process
  solution where the openPOWERLINK kernel layer is running as a separate
  Linux user space daemon (e.g. a PCAP based user space daemon)

- **CFG_COMPILE_LIB_MNAPP_KERNELINTF**

  Compile openPOWERLINK MN application library which contains the interface to
  a Linux kernel space driver. It is used together with a Linux kernel module
  openPOWERLINK driver.

- **CFG_COMPILE_LIB_MNDRV_PCAP**

  Compile openPOWERLINK MN driver library for Linux user space. This library
  contains the openPOWERLINK kernel layer and uses the PCAP library for accessing
  the network. It is used by the Linux user space daemon driver.

- **CFG_COMPILE_LIB_CN**

  Compile a complete openPOWERLINK CN library. The library contains an Ethernet
  driver which is using the PCAP library for accessing the network. It is
  configured to contain only CN functionality.

- **CFG_COMPILE_LIB_CNAPP_USERINTF**

  Compile openPOWERLINK CN application library which contains the interface
  to a Linux user space driver. It is used for implementing a multi-process
  solution where the openPOWERLINK kernel layer is running as a separate
  Linux user space daemon (e.g. a PCAP based user space daemon). It is
  configured to contain only CN functionality.

- **CFG_COMPILE_LIB_CNAPP_KERNELINTF**

  Compile openPOWERLINK MN application library which contains the interface to
  a Linux kernel space driver. It is used together with a Linux kernel module
  openPOWERLINK driver. It is configured to contain only CN functionality.

- **CFG_COMPILE_LIB_CNDRV_PCAP**

  Compile openPOWERLINK CN driver library for Linux user space. This library
  contains the openPOWERLINK kernel layer and uses the PCAP library for accessing
  the network. It is used by the Linux user space daemon driver. It is configured
  to contain only CN functionality.


## Windows Configuration Options

- **CFG_WINDOWS_DLL**

  If this option is set to ON, the libraries will be compiled as dynamic link
  libraries (DLL) instead of static libraries.

- **CFG_COMPILE_LIB_MN**

  Compile complete openPOWERLINK MN library. The library contains an Ethernet
  driver which is using the WinPCAP library for accessing the network.

- **CFG_COMPILE_LIB_CN**

  Compile complete openPOWERLINK CN library. The library contains an Ethernet
  driver which is using the WinPCAP library for accessing the network. It is
  configured to contain only CN functionality.

## Options for embedded systems (No-OS)

### Microblaze Configuration Options

- **CFG_COMPILE_LIB_CN**

  Compile complete openPOWERLINK CN library. The library consists of the user and
  kernel parts of the stack. It is configured to contain only CN functionality.
  - __CFG_COMPILE_LIB_CN_HW_LIB_DIR__

    Specify the path to the hardware platform the CN library should refer to.
    The path to the hardware platform should point to the export folder of the hardware
    project. (e.g: `<openPOWERLINK_DIR>/hardware/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`)

- **CFG_COMPILE_LIB_MNDRV_DUALPROCSHM**

  Compile openPOWERLINK MN driver library for Xilinx Microblaze. This library
  contains the openPOWERLINK kernel layer and uses the openMAC driver for
  accessing the network. It communicates with the openPOWERLINK user part by
  using the dual processor shared memory library. It can be used to implement
  an openPOWERLINK driver (PCP) on dual processor designs where the openPOWERLINK
  user layer is running on a separate processor connected via a shared memory.

  - __CFG_COMPILE_LIB_MN_HW_LIB_DIR__

  Specify the path to the hardware platform the driver library should refer to.
  The path to the hardware platform should point to the export folder of the hardware
  project. (e.g: `<openPOWERLINK_DIR>/hardware/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`)

### Xilinx Zynq ARM Configuration Options

- **CFG_COMPILE_LIB_MNAPP_DUALPROCSHM**

  Compile openPOWERLINK MN application library for Xilinx Zynq ARM. The library
  contains the openPOWERLINK user layer with CAL modules for accessing dual
  processor shared memory library. The dual processor shared memory library is
  used to communication with the user part of openPOWERLINK. The library can be
  used to implement an application designed to run on non-OS Zynq ARM processor
  communicating with the openPOWERLINK kernel layer running on a separate
  processor connected via a shared memory.

    - __CFG_COMPILE_LIB_MN_HW_LIB_DIR__

  Specify the path to the hardware platform the application library should refer to.
  The path to the hardware platform should point to the export folder of the hardware
  project. (e.g: `<openPOWERLINK_DIR>/hardware/lib/generic/zynqarm/<BOARD_NAME>/<DEMO_NAME>`)
