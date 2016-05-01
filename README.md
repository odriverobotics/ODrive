
### Compiling the firmware
#### Installing prerequisites 
To compile the program, you first need to install the prerequisite tools:
* `gcc-arm-none-eabi`: GCC compilation toolchain for ARM microcontrollers.
  * Installing on Ubuntu: `sudo apt-get install gcc-arm-none-eabi`
* `stm32cubeMX`: Tool from STM to automatically generate setup routines and configure libraries, etc.
  * Available [here](http://www2.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-configurators-and-code-generators/stm32cubemx.html?icmp=stm32cubemx_pron_pr-stm32cubef2_apr2014&sc=stm32cube-pr2)

#### Generate code
* Run stm32cubeMX and load the `Odrive.ioc` project file.
* Press `Project -> Generate code`
* You may need to let it download some drivers and such.

#### Generate makefile
There is an excellent project called CubeMX2Makefile, originally from baoshi. This project is included as a submodule.
* Initalise and clone the submodules: `git submodule init; git submodule update`
* 
