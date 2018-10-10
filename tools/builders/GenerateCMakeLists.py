import os,sys

# with open("../CMakeLists.txt","w") as F:
#     F.write("""
#     cmake_minimum_required(VERSION 3.11)
#     include(Firmware/CMakeListsPrivate.txt)
#     """)



with open("CMakeListsPrivate.txt","w") as F:
    F.write("cmake_minimum_required(VERSION 3.11)\n")
    Mode = None
    for i in sys.argv:


        if "--INC" in i:
            Mode = "Inc"
            continue
        if "--SRC" in i:
            Mode = "Src"
            continue

        if "--FLAG" in i:
            Mode = "Flag"
            continue

        if Mode == "Src":
            F.write('list(APPEND SOURCES "${CMAKE_SOURCE_DIR}/Firmware/'+i+'")\n')

        if Mode == "Inc":
            F.write('include_directories("${CMAKE_SOURCE_DIR}/Firmware/'+i+'")\n')

        if Mode == "Flag":
            while(i[0] == " "):
                i = i[1:]
            if "-D" in i:
                if "weak" in i: ## Special Case
                    F.write("add_definitions(-D__weak=__attribute__\(\(weak\)\))\n")
                elif "packed" in i: ## Special Case
                    F.write("add_definitions(-D__packed=__attribute__\(\(__packed__\)\))\n")
                else:
                    F.write("add_definitions("+i+")\n")

    F.write("add_executable(CodeCompletionHelper ${SOURCES})")




