# Native Protocol

This protocol is what the ODrive Tool uses to talk to the ODrive. If you have a choice, this is the recommended protocol for all applications. The native protocol runs on [USB](usb) and on [UART](uart).

## Python

The ODrive Tool you installed as part of the [Getting Started guide](getting-started.md#downloading-and-installing-tools) comes with a library that you can use to easily control the ODrive from Python.

Assuming you already installed the odrive library (`pip install odrive`), the simplest program to control the ODrive is this:

```python
import odrive
odrv0 = odrive.find_any()
print(str(odrv0.vbus_voltage))
```

For a more comprehensive example, see [tools/odrive_demo.py](../tools/odrive_demo.py).

## Other languages

We don't have an official library for you just yet. Check the community, there might be someone working on it. If you want to write a library yourself, refer to the [native protocol specification](protocol). You are of course welcome to contribute it back.