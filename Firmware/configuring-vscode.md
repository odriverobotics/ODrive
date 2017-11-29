# Configuring VSCode

VSCode is the recommended IDE for working with the ODrive codebase.  It is a light-weight text editor with Git integration and GDB debugging functionality.

Before doing the VSCode setup, make sure you've installed all of your [prerequisites](README.md#installing-prerequisites)

---
## Setup Procedure
1. Clone the ODrive repository
1. [Download VSCode](https://code.visualstudio.com/download)
1. Install extensions.  This can be done directly from VSCode (Ctrl+Shift+X)
    * Required extensions:
        * C/C++
        * Native Debug
    * Recommended Extensions:
        * vscode-icons
        * Code Outline
        * Include Autocomplete
        * Path Autocomplete
        * Auto Comment Blocks
1. Restart VSCode 
1. Open the VSCode Workspace file, which is located in the root of the ODrive repository.  It is called `VSCodeWorkspace.code-workspace`.  The first time you open it, VSCode will install some dependencies.  If it fails, you may need to [change your proxy settings](https://code.visualstudio.com/docs/getstarted/settings).

You should now be ready to compile and test the ODrive project.  See [Building the Firmware](README.md#building-the-firmware)