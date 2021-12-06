================================================================================
Specifications
================================================================================

.. contents::
   :depth: 1
   :local:
   
Electrical Specifications
--------------------------------------------------------------------------------

Besides the input voltage range, (12V to 24V for ODrive v3.6 24V, 12V to 56V for ODrive v3.6 56V), the electrical specifications of both version of ODrive v3.6 are the same.

.. note:: ODrive versions after v3.5 are closed-source with respect to board files and schematics.

ODrive v3.6 24V and 56V
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Peak current per motor: 120 Amps
* Max continuous current depends on cooling. See `this <https://discourse.odriverobotics.com/t/odrive-mosfet-temperature-rise-measurements-using-the-onboard-thermistor/972>`__ for more details.
    * Heatsink in still air: 40A per channel
    * Heatsink with basic fan cooling: 75A per channel
    * Heatsink with overkill fan cooling: 90A per channel
* Max motor RPM: This depends on your power supply voltage, motor, and encoder. It is the lesser of: 
    * motor RPM limit
    * encoder RPM limit
    * motor KV * 0.7 * Supply voltage
    * 35000 eRPM / # of motor pole pairs
    * (840M counts/minute) / encoder counts per revolution (for incremental encoders - 4 x pulses per revolution).

Schematic
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The electrical schematic for ODrive v3.5 is available `here <https://github.com/madcowswe/ODriveHardware/blob/master/v3/v3.5docs/schematic_v3.5.pdf>`__ in PDF format.

Mechanical Specifications
--------------------------------------------------------------------------------

STEP File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A step file for ODrive v3.5 is available `here <https://github.com/madcowswe/ODriveHardware/blob/master/v3/v3.5docs/PCB_v3.5.step>`__.

Board Outline and Mounting Hole Dimensions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. image:: figures/mech_dimensions.png
    :scale: 30 %
    :align: center
    :alt: board dimensions
