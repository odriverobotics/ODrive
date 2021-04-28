# Interface Definition File

This document describes the rules on which the ODrive Interface Definition file is built. It is intended for ODrive contributors who wish to modify it or ODrive users who want to autogenerate their own code from this file to interface with the ODrive.

## Terms and Concepts

*Value types* are a way of saying how values of this type are serialized/deserialized to/from raw bytes.
Value types can be:
  - one of the well-known types `bool`, `int8`, `uint8`, `int16`, `uint16`, `int32`, `uint32`, `int32`, `uint32`, `int64`, `uint64`, `float32`,  `float64`, `fibre.Ref<Interface>`
  - An enumeration (that is, a mapping between serialized numbers and well known value names)
  - A set of flags (in many programming languages this is the same as normal enums)

An *interface* is a collection of features (attributes and functions) that can be implemented by an object or used by a client as a filter for object discovery.

A *function* is something that takes zero or more inputs from the client, does something, and then returns zero or more outputs to the client. Since these input and output arguments are transmitted as raw bytes, they each have a value type.

An *attribute* is a reference to a subobject which again implements some interface.

Many languages don't make clear distinctions between interfaces and value types so let's be clear on this: attributes _always_ have an interface type and function input/output arguments _always_ have a value type. If you see something that looks like an attribute with a value type (let's say `uint32`), it's actually an attribute with the interface type `fibre.Property<uint32>`. If you see a function argument that looks like an interface type (let's say `MyIntf`) it's actually of the value type `fibre.Ref<MyIntf>`.


## File Structure

The top level contains a dictionary of interfaces and a dictionary of value types.
Interfaces as well as value types can be subordinate to other interfaces. Nested names are specified using dots in between the subnames.

Example:

```yaml
interfaces:
  MyFirstInterface: ...
  MyFirstInterface.SubInterface: ...

valuetypes:
  MyFirstEnum: ...
  MyFirstInterface.SubEnum: ...
```

## Interfaces

Interfaces consist of an `attributes` dictionary and a `functions` dictionary.

**Attributes** have a type which is either given by name as a string or directly in place.
Even though attributes conceptually and internally are always resolved to an interface type, for your convenience you can also give a value type which is then implicitly resolved to `fibre.Property<value type>`.

If the type is given as a string, it is resolved based on the scope in which it occurs. The search precedence is as follows: The innermost scope is searched first for an interface with that name and then for a value type with that name. If both names don't exist, the next outer scope is checked. Note that the order in which types are defined does not matter. The whole file is read before any type resolution occurs.

**Functions** have an `in` and `out` dictionary specifying one or more argument names with their corresponding value types. Like with attributes, the types can be specified in place or as a name. Type resolution also works the same except that only value types are checked for.

Example:
```yaml
interfaces:
  Car:
    attributes:
      velocity: float
      door_front_left: Door
      door_front_right: Door
      steering_wheel:
        attributes:
          angle: float
        functions:
          turn: {in: {delta_angle: float32}, out: {final_angle: float32}}
  Car.Door:
    attributes:
      is_open: bool
      part_of: Car
    functions:
      open:
      close:
```

Let's see how the type resolution of the attibute `Car.Door.part_of: Car` would work here:

 1. Interface `Car.Door.Car` => not found, proceed
 2. Value type `Car.Door.Car` => not found, proceed
 3. Interface `Car.Car` => not found, proceed
 4. Value type `Car.Car` => not found, proceed
 5. Interface `Car` => found. Link to this interface type.


## Enums

Enums are values which are associated with a name. They are serialized as 32-bit numbers.

Enumerators without an explicitly stated numerical value are guaranteed to have an underlying value one larger than that of the preceding enumerator.

Each enumerator must have a unique value.

Example:

```yaml
valuetypes:
  ModeOfTransport:
    values:
      Walking:
      Bicycle:
      Car: {value: 5}
      Train:
```

This would be serialized as:
 - Walking <=> `0x00000000` <=> `0x00 0x00 0x00 0x00`
 - Bicycle <=> `0x00000001` <=> `0x01 0x00 0x00 0x00`
 - Car <=> `0x00000005` <=> `0x05 0x00 0x00 0x00`
 - Train <=> `0x00000006` <=> `0x06 0x00 0x00 0x00`

## Flagfields

Flagfields are serialized as 32-bit low endian values where each bit has a named meaning.

A flag without an explicit bit number is guaranteed to have the bit number of the preceding flag plus one or bit 0 it it's the first in the list.

Each flag must have a unique bit number.

Example:

```yaml
valuetypes:
  Anchor:
    nullflag: Nowhere
    flags:
      Top:
      Left:
      Bottom: {bit: 8}
      Right:
```

This would be serialized as:
 - Nowhere <=> `0x00000000` <=> `0x00 0x00 0x00 0x00`
 - Top <=> `0x00000001` <=> `0x01 0x00 0x00 0x00`
 - Top and Left <=> `0x00000003` <=> `0x03 0x00 0x00 0x00`
 - Bottom <=> `0x00000100` <=> `0x00 0x01 0x00 0x00`
 - Top and Bottom and Right <=> `0x00000301` <=> `0x01 0x03 0x00 0x00`
