# AFC Drone Workspace Setup
This is a guide into how to setup your workspace for contributing to the Automated-Flight-Control Drone. 

## Required Software
The project was originally created with [VSCode](https://code.visualstudio.com/) and the [PlatformIO](https://platformio.org/) extension. Due to use of the Teensy 4.1, you will also need the [Teensy Loader](https://www.pjrc.com/teensy/loader.html) It should be noted that neither of these softwares are explicitly needed but this guide will not cover any methods outside of this.

For pulling our latest code and contributing any new code that you write you will need Git or GitHub Desktop.

### Installation

1. Visual Studio Code

Start by downloading Visual Studio Code. The download can be found [Here](https://code.visualstudio.com/download). Make sure to pick the appropriate download for your computer.

2. PlatformIO

Once VSCode has completed downloading open the extension tab on the right side of the screen or by pressing `CTRL + SHIFT + X`. There should then be a search bar at the top of the screen, enter in `platformio.platformio-ide`. Click on the extension shown and click the blue `Install` button. 

This should install PlatformIO into your VSCode profile. If you have used VSCode in the past it is recomended that you create a new profile by following [this guide](https://code.visualstudio.com/docs/configure/profiles).

The instalation should take a couple minutes before opening up to the PlatformIO homescreen. While this installs move on to step 3

3. GitHub Desktop

All of the code for this project is stored in GitHub. Git is a neat tool that allows us to keep track of changes through commits and branches. For simplicity this guide will use GitHub Desktop due to the simple GUI. 

[Download GitHub Desktop here](https://desktop.github.com/download/). 

Once installed sign in with your GitHub account. This [guide](https://docs.github.com/en/desktop/installing-and-authenticating-to-github-desktop/authenticating-to-github-in-github-desktop) can be used if you run into any issues. 

Finally you will need to clone the repository to your computer. Click `File` in the top right then `Clone Repository`. Search for `AFC-Drone` and then choose the location you would like to save the files to. Please make sure that this is in a remeberable spot and contained within its own folder. 

4. Import the project into ProjectIO

Open the PlatformIO home page in VSCode. This can be found in the search bar at the top of the VSCode window by typing, `>PlatformIO: PlatformIO Home`. Then click `Open Project`, navigate to the project folder, `AFC-Drone`, that you cloned in Step 3.

*Note* PlatformIO may take a few moments to load. 

5. Teensy Loader

Download [Teensy Loader](https://www.pjrc.com/teensy/loader.html). Make sure you pick the download that is appropriate for your opperating system. Open the download and install the program. 

This program is only used to help PlatformIO upload files to the Teensy. 