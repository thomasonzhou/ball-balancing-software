# How to use this repo
Make sure you have the devcontainer extension installed in VSCode, then click on the bottom left and reopen the IDE in a container

Add pip packages in .devcontainer/requirements.txt

Also, to open a devcontainer directly from the command line, [make sure you have the devcontainer-cli installed](https://code.visualstudio.com/docs/devcontainers/devcontainer-cli), then type
```bash
devcontainer open .
```

## How to build the C++ binary

Make sure you have CMake installed. cd to the build directory and enter
```bash
cmake ../src
cmake --build .
```

The binary/binaries will be outputted as a file. To run a binary, type 
```bash
./name_of_binary
```
