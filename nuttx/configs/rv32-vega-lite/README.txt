README
======

rv32-vega-lite is an evaluation board for RV32M1.

RV32M1
72 MHz ARM® Cortex®-M0+/M4F RISC-V RI5CY/ZERO RISCY
Quad Core Microcontroller with up to 1280 KB Flash and 384
KB SRAM
The RV32M1 is an ultra-low-power, quad core solution ideal for
applications that require a high performance Cortex-M4F/RI5CY
processor to run the application and an efficient Cortex-M0+/
ZERO-RISCY to run radio and connectivity stack operations.
Applications include portable health care devices, wearable
sports and fitness devices, appliances, access control, climate
control, energy management, lighting, safety and security
systems .


See also:
rv32-vega-lite board:

RV32M1 datasheet:

Contents
========

  - Environment Setup
  - Configurations
  - Execute

Environment Setup
=================

Configurations
==============
  Each rv32-vega-lite configuration is maintained in a sub-directory and can
  be selected as follow:

    tools/configure.sh rv32-vega-lite/<subdir>

  Where <subdir> is one of the following:

    nsh
    ---
    This is an NSH example that uses the UART connected to /dev/ttyACM0 as
    the console. Default UART settings are 115200, 8N1.

Execute
=======

