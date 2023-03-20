# STM32
This repo is about STM32 microcontroller. 
I'm uploading some STM32 examples using a "blue pill" development board and STM32CubeIDE.

NOTE: I've got some "Blue pill" boards from China. They came with a chinese
microcontroller clone (it wasn't the STM32F103C8) so I had problems debugging it
on STM32CubeIDE.

The solution is modifiying one file (stm32f1x.cfg file) on the installation 
folder:

/c/ST/STM32CubeIDE_1.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.debug_1.2.0.201912201802/resources/openocd/st_scripts/target/stm32f1x.cfg

There we will find line:

set _CPUTAPID 0x1ba01477:

We have to change 0x1ba01477 to 0x2ba01477

Also, we have to add the next line next to it:

reset_config trst_only

Also, when using Cube IDE for debugging, is going to "Debug Configurations >>
Debugger >> Debug probe" then selecting "ST-LINK (OpenOCD)". It is going to need
a Cube IDE restart.