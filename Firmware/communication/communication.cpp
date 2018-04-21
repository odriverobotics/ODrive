
/* Includes ------------------------------------------------------------------*/

#include "communication.h"

#include "interface_usb.h"
#include "interface_uart.h"

#include "odrive_main.h"
#include "protocol.hpp"
#include "freertos_vars.h"
#include "utils.h"


#include <cmsis_os.h>
#include <memory>
//#include <usbd_cdc_if.h>
//#include <usb_device.h>
//#include <usart.h>
//#include <gpio.h>

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

uint64_t serial_number;
char serial_number_str[13]; // 12 digits + null termination

/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/

void enter_dfu_mode() {
    __asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
    _reboot_cookie = 0xDEADBEEF;
    NVIC_SystemReset();
}

void init_communication(void) {
    printf("hi!\r\n");

    // Start command handling thread
    osThreadDef(task_cmd_parse, communication_task, osPriorityNormal, 0, 5000 /* in 32-bit words */); // TODO: fix stack issues
    osThreadCreate(osThread(task_cmd_parse), NULL);
}


uint32_t comm_stack_info = 0; // for debugging only

// Helper class because the protocol library doesn't yet
// support non-member functions
// TODO: make this go away
class StaticFunctions {
public:
    void save_configuration_helper() { save_configuration(); }
    void erase_configuration_helper() { erase_configuration(); }
    void NVIC_SystemReset_helper() { NVIC_SystemReset(); }
    void enter_dfu_mode_helper() { enter_dfu_mode(); }
} static_functions;

// When adding new functions/variables to the protocol, be careful not to
// blow the communication stack. You can check comm_stack_info to see
// how much headroom you have.
static inline auto make_obj_tree() {
    return make_protocol_member_list(
        make_protocol_ro_property("vbus_voltage", &vbus_voltage),
        make_protocol_ro_property("comm_stack_info", &comm_stack_info),
        make_protocol_ro_property("serial_number", &serial_number),
        make_protocol_ro_property("brake_resistor_armed", &brake_resistor_armed_),
        make_protocol_object("config",
            make_protocol_property("brake_resistance", &board_config.brake_resistance),
            // TODO: changing this currently requires a reboot - fix this
            make_protocol_property("enable_uart", &board_config.enable_uart),
            make_protocol_property("dc_bus_undervoltage_trip_level", &board_config.dc_bus_undervoltage_trip_level),
            make_protocol_property("dc_bus_overvoltage_trip_level", &board_config.dc_bus_overvoltage_trip_level)
        ),
        make_protocol_object("axis0", axes[0]->make_protocol_definitions()),
        make_protocol_object("axis1", axes[1]->make_protocol_definitions()),
        make_protocol_function("save_configuration", static_functions, &StaticFunctions::save_configuration_helper),
        make_protocol_function("erase_configuration", static_functions, &StaticFunctions::erase_configuration_helper),
        make_protocol_function("reboot", static_functions, &StaticFunctions::NVIC_SystemReset_helper),
        make_protocol_function("enter_dfu_mode", static_functions, &StaticFunctions::enter_dfu_mode_helper)
    );
}

using tree_type = decltype(make_obj_tree());
uint8_t tree_buffer[sizeof(tree_type)];

// the protocol has one additional built-in endpoint
constexpr size_t MAX_ENDPOINTS = decltype(make_obj_tree())::endpoint_count + 1;
Endpoint* endpoints_[MAX_ENDPOINTS] = { 0 };
const size_t max_endpoints_ = MAX_ENDPOINTS;
size_t n_endpoints_ = 0;

// Thread to handle deffered processing of USB interrupt, and
// read commands out of the UART DMA circular buffer
void communication_task(void * ctx) {
    (void) ctx; // unused parameter

    // TODO: this is supposed to use the move constructor, but currently
    // the compiler uses the copy-constructor instead. Thus the make_obj_tree
    // ends up with a stupid stack size of around 8000 bytes. Fix this.
    auto tree_ptr = new (tree_buffer) tree_type(make_obj_tree());
    auto endpoint_provider = EndpointProvider_from_MemberList<tree_type>(*tree_ptr);
    set_application_endpoints(&endpoint_provider);
    comm_stack_info = uxTaskGetStackHighWaterMark(nullptr);
    
    serve_on_uart();
    serve_on_usb();

    for (;;) {
        osDelay(1000); // nothing to do
    }
}

extern "C" {
int _write(int file, const char* data, int len);
}

// @brief This is what printf calls internally
int _write(int file, const char* data, int len) {
#ifdef USB_PROTOCOL_STDOUT
    usb_stream_output.process_bytes((const uint8_t *)data, len);
#endif
#ifdef UART_PROTOCOL_STDOUT
    uart4_stream_output.process_bytes((const uint8_t *)data, len);
#endif
    return len;
}
