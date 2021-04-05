# xpbd
An implementation of "XPBD: position-based simulation of compliant constrained dynamics", MIG'16

## How to build on mac

```bash
mkdir build
cd build
cmake ../src
cmake ../src/ && (cmake --build .; /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  -isysroot /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX11.1.sdk -Wl,-search_paths_first -Wl,-headerpad_max_install_names CMakeFiles/xpbd.dir/xpbd.cpp.o -o xpbd -framework OpenGL -framework GLUT )
```

and run 
```bash
./xpbd
```