import os,sys,time
Path = sys.argv[2]
if not os.path.isdir(Path):
    print("Created build/version.h")
    os.mkdir(Path)
    time.sleep(0.1)