.. ODrive Documentation documentation master file, created by
   sphinx-quickstart on Wed Nov  3 20:01:31 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. include:: getting-started.rst


.. toctree::
   :hidden:
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
   :hidden:
   :maxdepth: 1
   :caption: Tutorials

   Hoverboard Guide <hoverboard>
   migration
   CAN Guide <can-guide>


.. toctree::
   :hidden:
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
   :hidden:
   :maxdepth: 1
   :caption: For ODrive Developers

   developer-guide
   configuring-vscode
   configuring-eclipse

