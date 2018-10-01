import sys

Debug = False

if Debug:
    TupConfigPath = "D:/LaptopSync/CLionWorkspace/ODrive/Firmware/tup.config"
    HALConfigLuaPath = "D:/LaptopSync/CLionWorkspace/ODrive/Firmware/HAL_Config.lua"
else:
    TupConfigPath = sys.argv[1]
    HALConfigLuaPath = sys.argv[2]
    HelperPath = sys.argv[3]


def ParseConfig(Path):
    with open(Path,"r") as File:
        FileLines = File.readlines()

    for C,Line in enumerate(FileLines): ## Remove comments
        if "#" in Line:
            FileLines[C] = Line[:Line.index("#")]+"\n"

    Settings = {}
    for C,Line in enumerate(FileLines):
        if "=" in Line:
            ThisSetting = Line.rstrip().split("=")
            if ThisSetting[0] in Settings:
                print("Warning:",ThisSetting[0], "Redefined. Line:",C+1)

            Settings[ThisSetting[0].replace("CONFIG_","")] = ThisSetting[1]

    return Settings




def ParseLua(Path):
    with open(Path,"r") as File:
        FileLines = File.readlines()

    for C,OrigLine in enumerate(FileLines):
        if "--" in FileLines[C]:
            FileLines[C] = FileLines[C][:FileLines[C].index("--")]+"\n" ## Remove comments


        FileLines[C] = FileLines[C].replace(" then",":")
        FileLines[C] = FileLines[C].replace("elseif ","elif ")
        FileLines[C] = FileLines[C].replace("..","+")
        FileLines[C] = FileLines[C].replace("tup.","lua_interpreter.")
        FileLines[C] = FileLines[C].replace("'",'"')
        FileLines[C] = FileLines[C].replace('LDFLAGS',"lua_interpreter.ldf")
        FileLines[C] = FileLines[C].replace('FLAGS',"lua_interpreter.f")
        FileLines[C] = FileLines[C].replace('else',"else:")
        FileLines[C] = FileLines[C].replace('end',"")
    CodeStr = ""
    for i in FileLines:
        CodeStr += i
    return CodeStr



class LuaReturn:
    def __init__(self):
        self.f = []
        self.ldf = []

    def getconfig(self,In):
        if In in ConfigSettings:
            return ConfigSettings[In]
        else:
            return False


def Agglomerate(In,SplitChar):
    L = []
    S = ""
    for Char in In:
        if Char == SplitChar:
            if S != "":
                L.append(S)
                S = ""
        S += Char
    if S != "":
        L.append(S)
    return L

def error(In):
    print("ERROR IN CONFIG/HAL",In)
    exit(1)

# Parse Config for vars
# Parse luascript and convert to python
# Run pyluascript with config

ConfigSettings = ParseConfig(TupConfigPath)
PyLuaScript = ParseLua(HALConfigLuaPath)


lua_interpreter = LuaReturn();
exec(PyLuaScript)

Flags = Agglomerate(lua_interpreter.f,"-")
LDFlags = Agglomerate(lua_interpreter.ldf,"-")
import os
if os.path.exists(HelperPath+"/HAL_Generated.cmake"):
    os.remove(HelperPath+"/HAL_Generated.cmake")

with open(HelperPath+"/HAL_Generated.cmake","w") as F:
    F.write("set(BOARD_DIR Firmware/" +boarddir+")\n")

    F.write('set(COMMON_FLAGS "${COMMON_FLAGS} ')
    for i in Flags:
        F.write(i+" ")
    F.write('")')
    F.write("\n")

    F.write('set(LINKER_FLAGS "')
    for i in LDFlags:
        if "-T" in i:
            i = "-T${CMAKE_SOURCE_DIR}/Firmware/"+i[2:]
        F.write(i+" ")
    F.write('")')
    F.write("\n")
