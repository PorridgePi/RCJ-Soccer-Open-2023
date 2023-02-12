Import("env")

import os, socket, platform

def preBuild(source, target, env):
    print("----- EXECUTING PRE-BUILD SCRIPT -----")

    hostname = socket.gethostname()
    system = platform.system()

    if system == 'Darwin' and hostname == 'MBP':
        if os.path.exists('/Applications/Xcode.app/Contents/Developer'):
            os.rename('/Applications/Xcode.app/Contents/Developer', '/Applications/Xcode.app/Contents/Developer.bak')
            print("INFO: Xcode renamed")
        else:
            print("WARN: Xcode already renamed")

    print("----- PRE-BUILD SCRIPT DONE -----")

def postBuild(source, target, env):
    print("----- EXECUTING POST-BUILD SCRIPT -----")

    hostname = socket.gethostname()
    system = platform.system()

    if system == 'Darwin' and hostname == 'MBP':
        if os.path.exists('/Applications/Xcode.app/Contents/Developer.bak'):
            os.rename('/Applications/Xcode.app/Contents/Developer.bak', '/Applications/Xcode.app/Contents/Developer')
            print("INFO: Xcode renamed back")
        else:
            print("WARN: Xcode (probably) renamed back")
    
    print("----- POST-BUILD SCRIPT DONE -----")

# print("PRE-BUILD SCRIPT LOADED")
# pwd=os.getcwd()
# for filename in os.listdir(pwd+'/src'):
#     if filename.endswith('.cpp'):
#         env.AddPreAction(f"$BUILD_DIR/src/{filename}.o", preBuild)
#         print(f"PRE-BUILD SCRIPT ADDED TO {filename}")

preBuild(None, None, None)
env.AddPostAction("checkprogsize", postBuild)
