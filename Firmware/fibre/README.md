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
   weren't used by other programs. **This is not implemented yet.**

## Current Status ##

The project is in an early stage and the focus so far was to get a minimum working implementation.

* **C++**: Currently only supports the server side (i.e. publishing local objects). The C++ library comes with builtin support for TCP and UDP transport layers on Posix platforms. The library can easily be used with user provided transport layers.

* **Python**: Currently only supports the client side (i.e. using remote objects). The Python library comes with builtin support for TCP, UDP, USB and UART transport layers.

Support for more languages (most importantly JavaScript) will be added once the protocol matures. Feel free to add your contribution.

## Show me some code

Consider this program:

```
class TestClass {
public:
    float property1;
    float property2;

    float set_both(float arg1, float arg2) {
        property1 = arg1;
        property2 = arg2;
        return property1 + property2;
    }
};


int main() {
    TestClass test_object = TestClass();

    while (1) {
        printf("test_object.property1: %f\n", test_object.property1);
        usleep(1000000 / 5); // 5 Hz
    }
}
```

Say you want to publish `test_object` so that a remote Fibre node can use it.

1. Add includes
      ```C++
      #include <fibre/protocol.hpp>
      #include <fibre/posix_tcp.hpp>
      ```
1. Add Fibre export definitions to the exported class
      ```C++
      class TestClass {
            [...]
            FIBRE_EXPORTS(TestClass,
                  make_protocol_property("property1", &property1),
                  make_protocol_property("property2", &property2),
                  make_protocol_function("set_both", *obj, &TestClass::set_both, "arg1", "arg2")
            );
      };
      ```
   Note: in the future this will be generated from a YAML file using automatic code generation.

1. Publish the object on Fibre
      ```C++
      auto definitions = test_object.fibre_definitions;
      fibre_publish(definitions);
      ```
   Note: currently you must publish all objects at once. This will be fixed in the future.

1. Start the TCP server
      ```C++
      std::thread server_thread_tcp(serve_on_tcp, 9910);
      ```
      Note: this step will be replaced by a simple `fibre_start()` call in the future. All builtin transport layers then will be started automatically.

## Adding Fibre to your project ##

We recommend Git subtrees if you want to include the Fibre source code in another project.
Other contributors don't need to know anything about subtrees, to them the Fibre repo will be like any other normal directory.

#### Adding the repo
```
git remote add fibre-origin git@github.com:samuelsadok/fibre.git
git fetch fibre-origin
git subtree add --prefix=fibre --squash fibre-origin master
```
Instead of using the upstream remote, you might want to use your own fork for greater flexibility.

#### Pulling updates from upstream
```
git subtree pull --prefix=fibre --squash fibre-origin master
```

#### Contributing changes back to upstream
This requires push access to `fibre-origin`.
```
git subtree push --prefix=fibre fibre-origin master
```

## Projects using Fibre ##

 - [ODrive](https://github.com/madcowswe/ODrive): High performance motor control
 - [lightd](https://github.com/samuelsadok/lightd): Service that can be run on a Raspberry Pi (or similar) to control RGB LED strips

## Contribute ##

This project losely adheres to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
