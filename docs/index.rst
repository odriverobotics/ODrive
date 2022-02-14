.. ODrive Documentation documentation master file, created by
   sphinx-quickstart on Wed Nov  3 20:01:31 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

================================================================================
ODrive Documentation
================================================================================

Welcome to the ODrive V3.6 documentation homepage!

This project strives to bring high performance motor control to makers, all of the source code is 
open source and available on github `here. <https://github.com/odriverobotics/ODrive>`__

.. note::

   This documentation is specifically for V3.6, for ODrive Pro Beta click `here. <https://betadocs.odriverobotics.com/>`__

`ODrive Robotics homepage. <https://odriverobotics.com/>`__

Table of Contents
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 1
   :caption: General

   getting-started
   odrivetool
   control-modes
   commands
   encoders
   control
   troubleshooting
   specifications
   ground-loops

.. toctree::
   :maxdepth: 1
   :caption: Tutorials

   Hoverboard Guide <hoverboard>
   migration
   CAN Guide <can-guide>
   Motor Guide <https://docs.google.com/spreadsheets/d/12vzz7XVEK6YNIOqH0jAz51F5VUpc-lJEs3mmkWP1H4Y/edit#gid=0>
   Encoder Guide <https://docs.google.com/spreadsheets/d/1OBDwYrBb5zUPZLrhL98ezZbg94tUsZcdTuwiVNgVqpU/edit#gid=0>


.. toctree::
   :maxdepth: 1
   :caption: Interfaces and Protocols
   
   protocol
   Pinout <pinout>
   usb
   uart
   native-protocol
   ascii-protocol
   can-protocol
   Step & Direction <step-direction>
   rc-pwm
   analog-input
   endstops
   thermistors


.. fibreautosummary:: com.odriverobotics.ODrive
   :caption: ODrive Device API
   

.. toctree::
   :maxdepth: 1
   :caption: For ODrive Developers

   developer-guide
   configuring-vscode
   configuring-eclipse

