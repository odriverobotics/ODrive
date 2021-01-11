
-- Prevent tup from running `Firmware/fibre-cpp/Tupfile.lua` when we run it in
-- the `Firmware` folder.
no_libfibre = tup.getconfig("BUILD_LIBFIBRE") != "true"
