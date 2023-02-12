# RCJ Soccer Open 2023

## Usage

For Visual Studio Code:

- Clone and open this repository in VSCode
- Install the PlatformIO plugin
- Use the PlatformIO plugin to compile and upload code onto the microcontrollers

## Code Structure
- `.vscode/`: Folder containing VSCode workspace settings
    - `extensions.json`: List of recommended and unwanted extensions ([Documentation](https://code.visualstudio.com/docs/editor/extension-gallery#_workspace-recommended-extensions))
- `lib/`: Libraries for components
    - `private/`: Our own libraries for components
- `src/`: Main code
- `.clang-format`: ClangFormat code style file - to format code in VSCode
- `.gitignore`: gitignore file ([Documentation](https://git-scm.com/docs/gitignore))
- `platformio.ini`: PlatformIO configuration file ([Documentation](https://docs.platformio.org/en/stable/projectconf/index.html))
- `extra_scripts.py`: Extra scripts for PlatformIO (run before and after build - [Documentation](https://docs.platformio.org/en/latest/scripting/actions.html))
