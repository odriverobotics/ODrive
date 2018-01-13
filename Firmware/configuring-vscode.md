# Configuring VSCode

VSCode is the recommended IDE for working with the ODrive codebase.  It is a light-weight text editor with Git integration and GDB debugging functionality.

Before doing the VSCode setup, make sure you've installed all of your [prerequisites](README.md#installing-prerequisites)

## Setup Procedure
1. Clone the ODrive repository
1. [Download VSCode](https://code.visualstudio.com/download)
1. Open VSCode
1. Install extensions.  This can be done directly from VSCode (Ctrl+Shift+X)
    * Required extensions:
        * C/C++
    * Recommended Extensions:
        * Cortex-Debug
        * vscode-icons
        * Code Outline
        * Include Autocomplete
        * Path Autocomplete
        * Auto Comment Blocks
1. Restart VSCode 
1. Open the VSCode Workspace file, which is located in the root of the ODrive repository.  It is called `VSCodeWorkspace.code-workspace`.  The first time you open it, VSCode will install some dependencies.  If it fails, you may need to [change your proxy settings](https://code.visualstudio.com/docs/getstarted/settings).

You should now be ready to compile and test the ODrive project.

## Building the Firmware
* Tasks -> Run Build Task

A terminal window will open with your native shell.  VSCode is configured to run the command `make -j4` in this terminal.

## Flashing the Firmware
* Tasks -> Run Task -> flash

A terminal window will open with your native shell.  VSCode is configured to run the command `make flash` in this terminal.

If the flashing worked, you can start sending commands. If you want to do that now, you can go to [Communicating over USB or UART](README.md#communicating-over-usb-or-uart).

## Debugging
An extension called Cortex-Debug has recently been released which is designed specifically for debugging ARM Cortex projects.  You can read more on Cortex-Debug here: https://github.com/Marus/cortex-debug

Note: If developing on Windows, you should have `arm-none-eabi-gdb` and `openOCD` on your PATH.

  * Make sure you have the Firmware folder as your active folder
  * Flash the board with the newest code (starting debug session doesn't do this)
  * Debug -> Start Debugging (or press F5)
  * The processor will reset and halt.
  * Set your breakpoints. Note: you can only set breakpoints when the processor is halted, if you set them during run mode, they won't get applied.
  * Run (F5)
  * Stepping over/in/out, restarting, and changing breakpoints can be done by first pressing the "pause" (F6) button at the top the screen.
  * When done debugging, simply stop (Shift+F5) the debugger.  It will kill your openOCD process too.

## Cleaning the Build
This sometimes needs to be done if you change branches.
* Open a terminal (View -> Integrated Terminal) and enter `make clean`