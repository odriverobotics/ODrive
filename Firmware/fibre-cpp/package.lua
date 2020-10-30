
fibre_package = {
    core_files = {
        'libfibre.cpp',
        'legacy_protocol.cpp',
        'legacy_object_client.cpp',
    },
    features = {
        LIBUSB={'platform_support/libusb_transport.cpp'},
        LOGGING={'logging.cpp'},
    }
}
