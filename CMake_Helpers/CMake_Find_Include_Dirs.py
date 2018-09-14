import sys
SourcesHeaders = sys.argv[2].split(";")

def ShouldBeIncluded(Path):
    Path = Path.lower()
    return ".h" in Path or "include" in Path

def RemoveToLast(In,Char):
    for i in range(len(In)-1,-1,-1):
        if In[i] == Char:
            return In[:i]

Headers = []
for FilePath in SourcesHeaders:
    if ShouldBeIncluded(FilePath):
        Headers.append(FilePath)



Folders = []
for HeaderFilePath in Headers:
    while ShouldBeIncluded(HeaderFilePath):
        HeaderFilePath = RemoveToLast(HeaderFilePath,'/')
        Folders.append(HeaderFilePath)


Folders = sorted(list(set(Folders)))
for i in Folders:
    print(i,end=";")
