.. _native-protocol:

================================================================================
Native Protocol
================================================================================

This protocol is what the :code:`odrivetool` uses to talk to the ODrive. 
If you have a choice, this is the recommended protocol for all applications. 
The native protocol runs on :ref:`USB <usb-doc>` and on :ref:`UART <uart-doc>`.

Python
--------------------------------------------------------------------------------

The :code:`odrivetool` you installed as part of the :ref:`Getting Started guide <install-odrivetool>` comes with a library that you can use to easily control the ODrive from Python.

Assuming you already installed the odrive library (:code:`pip install odrive`), the simplest program to control the ODrive is this:

.. code:: Python

    import odrive
    odrv0 = odrive.find_any()
    print(str(odrv0.vbus_voltage))


For a more comprehensive example, see `tools/odrive_demo.py <https://github.com/odriverobotics/ODrive/blob/master/tools/odrive_demo.py>`_.

Other Languages
--------------------------------------------------------------------------------

We don't have an official library for you just yet. Check the community, there might be someone working on it. 
If you want to write a library yourself, refer to the :ref:`native protocol specification <protocol-doc>`. 
You are of course welcome to contribute it back.