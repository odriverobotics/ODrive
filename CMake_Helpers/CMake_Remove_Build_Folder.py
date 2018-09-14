import sys,os,time,shutil
Path = sys.argv[2]
if os.path.isdir(Path):
    shutil.rmtree(Path)
    time.sleep(0.1)
    print("Removed build/version.h")
