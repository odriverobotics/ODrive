# PyFibre

This directory provides Python bindings for [Fibre](https://github.com/samuelsadok/fibre). Its home is located [here](https://github.com/samuelsadok/fibre/tree/master/python). There's also a standalone repository for this directory [here](https://github.com/samuelsadok/pyfibre).

## Current Status

Currently only client-side features are implemented, that means you can discover objects but you cannot publish objects.

## How to use

```python
import fibre

with fibre.Domain("tcp-client:address=localhost,port=14220") as domain:
    obj = domain.discover_one()
    obj.test_function()
```
