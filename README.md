# How to use this repo
Make sure you have the devcontainer extension installed in VSCode, then click on the bottom left and reopen the IDE in a container

Add pip packages in .devcontainer/requirements.txt

Also, to open a devcontainer directly from the command line, [make sure you have the devcontainer-cli installed](https://code.visualstudio.com/docs/devcontainers/devcontainer-cli), then type
```bash
devcontainer open .
```

## Installing OpenCV

Clone the github repos for:
- [opencv core](https://github.com/opencv/opencv)
- [opencv_contrib](https://github.com/opencv/opencv_contrib)

Build using CMake. Optionally, you can add `-G Ninja` to build more quickly (if you have it installed).

```bash
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ../opencv
cmake --build .
```

## How to build the C++ binary

Make sure you have CMake installed. cd to the build directory and enter
```bash
cmake ..
cmake --build .
```

The binary/binaries will be outputted as a file. To run a binary, type 
```bash
./name_of_binary
```


## Misc
To find an appropriate FOV: [FOV Calculator](https://www.arducam.com/fov-calculator/)
