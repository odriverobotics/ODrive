## Overview ##

The goal of Fibre is to provide a framework to make suckless distributed applications easier to program.

In particular:

 - Nobody likes boiler plate code. Using a remote object should feel almost
   exactly as if it was local. No matter if it's in a different process, on
   a USB device, connected via Bluetooth, over the Internet or all at the
   same time.
   All complexity arising from the system being distributed should be taken
   care of by Fibre while still allowing the application developer to easily
   fine-tune things.

 - Fibre has the ambition to run on most major platforms and provide bindings
   for the most popular languages. Even bare metal embedded systems with very
   limited resources. See the Compatibility section for the current status.

 - Once you deployed your application and want to change the interface, don't
   worry about breaking other applications. With Fibre's object model, the
   most common updates like adding methods, properties or arguments won't
   break anything. Sometimes you can get away with removing methods if they
   weren't used by other programs.


## Architecture ##

Note that this is not yet representative for the existing code.

```

 /--------------\  /--------------\   /---------------------------\
 | Light Toggle |  | Driver for   |   | LED strip -> simple light |
 | Applet       |  | Built-in LED |   | adaptor                   |
 \--------------/  \--------------/   \---------------------------/
       |                  |                     |
 /-------------------------------------------------------\
 |                   Local Fibre Hub                     |
 \-------------------------------------------------------/
       |                |
 /-----------\    /-----------\
 | USB relay |    | UDP relay |
 \-----------/    \-----------/
       |                |
 /---------------\      |
 | fibre-enabled |   /-----------\
 | desk lamp     |   | UDP relay |
 \---------------/   \-----------/
                        |
                 /------------------\
                 | remote fibre hub |
                 \------------------/
                        |
                 /------------------\
                 | LED strip driver |
                 \------------------/

```


## Compatibility ##

|                  | Linux |
|------------------|:-----:|
| **UDP**          |   yes |
| **raw TCP**      |   yes |
| **WebSocks**     |       |
| **HTTP**         |       |
| **Bluetooth LE** |       |
| **USB**          |       |
| **Serial**       |       |
| **CAN**          |       |
| **SPI**          |       |
| **I2C**          |       |


## Projects based on Fibre ##

[lightd](https://github.com/samuelsadok/lightd) Service that can be run on a Raspberry Pi (or similar) to control RGB LED strips

## Roadmap ##

- Add compatibility with Windows, macOS, iOS, Android
- Add bindings for JavaScript, C#
- CI to test all items on the feature matrix

## Contribute ##

This project losely adheres to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
