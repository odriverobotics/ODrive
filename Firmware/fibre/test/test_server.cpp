
#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <signal.h>

#include <fibre/protocol.hpp>
#include <fibre/posix_tcp.hpp>
#include <fibre/posix_udp.hpp>


class TestClass {
public:
    float property1;
    float property2;

    float set_both(float arg1, float arg2) {
        property1 = arg1;
        property2 = arg2;
        return property1 + property2;
    }

    FIBRE_EXPORTS(TestClass,
        make_protocol_property("property1", &property1),
        make_protocol_property("property2", &property2),
        make_protocol_function("set_both", *obj, &TestClass::set_both, "arg1", "arg2")
    );
};


int main() {
    printf("Starting Fibre server...\n");

    TestClass test_object = TestClass();

    // publish the object on Fibre
    auto definitions = test_object.fibre_definitions;
    fibre_publish(definitions);

    // Expose Fibre objects on TCP and UDP
    std::thread server_thread_tcp(serve_on_tcp, 9910);
    std::thread server_thread_udp(serve_on_udp, 9910);
    printf("Fibre server started.\n");

    // Dump property1 value
    while (1) {
        printf("test_object.property1: %f\n", test_object.property1);
        usleep(1000000 / 5); // 5 Hz
    }

    return 0;
}
