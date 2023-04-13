# RCJ Soccer Open 2023

## Usage

For Visual Studio Code:

- Clone this repository: `git clone https://github.com/PorridgePi/RCJ-Soccer-Open-2023`
- Change directory: `cd RCJ-Soccer-Open-2023/`
- Initialise Git submodule(s): `git submodule update --init --recursive`
- Open the repository in VSCode
- Install the PlatformIO plugin
- Use the PlatformIO plugin to compile and upload code onto the microcontrollers

## Code Structure
- `.vscode/`: Folder containing VSCode workspace settings
    - `extensions.json`: List of recommended and unwanted extensions ([Documentation](https://code.visualstudio.com/docs/editor/extension-gallery#_workspace-recommended-extensions))
- `lib/`: Libraries for components
    - `private/`: Our own libraries for components
    - `public/`: Libraries from other sources
        - `fastled/`: Library for SK6812 LED - [FastLED/FastLED](https://github.com/fastled/fastled/)
        - `Led.h`: Library for SK6812 LED - [gitpeut/SK6812-RGBW-ESP32
](https://github.com/gitpeut/SK6812-RGBW-ESP32)
- `scripts/`: Scripts for PlatformIO
    - `sim.py`: Simulation of ball tracking with Python Turtle graphics
    - `extra_scripts.py`: Script to run before and after build
- `src/`: Main code
    - `test/`: Test code
    - `main.cpp`: Main code for RPi Pico
    - `esp32.cpp`: Code for ESP32 (on bottom plate)
- `.clang-format`: ClangFormat code style file - to format code in VSCode
- `.gitignore`: gitignore file ([Documentation](https://git-scm.com/docs/gitignore))
- `platformio.ini`: PlatformIO configuration file ([Documentation](https://docs.platformio.org/en/stable/projectconf/index.html))
- `extra_scripts.py`: Extra scripts for PlatformIO (run before and after build - [Documentation](https://docs.platformio.org/en/latest/scripting/actions.html))

## Calibration
1. Camera centre x and y coordinates
2. Individual LiDAR readings with 30cm ruler
3. Overall LiDAR sum in each axis such that confidence = 1.00

# Team Members
- [Aaron](https://github.com/Aaron-Ong)
- [Estelle](https://github.com/sniparret)
- [Yikun](https://github.com/PorridgePi)
- [Zachary](https://github.com/Zachareeeeee) ([former account](https://github.com/C0RDITE))
