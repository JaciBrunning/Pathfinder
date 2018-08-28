# Pathfinder LabVIEW Library
This is a LabVIEW wrapper of the 'Pathfinder' motion profiling library. A few small changes were added to the Pathfinder-Core code to allow LabVIEW to wrap the key function calls properly.

## Installing
1. Follow the instructions to build the Pathfinder-Core, the output will be a .dll if running on Windows, or a .so for Linux (or for the roboRIO) **LabVIEW requires 32-bit**
2. Run the gradle install, or copy the output .dll to the LabVIEW resource folder for the appropriate year
    - Windows example: C:\Program Files (x86)\National Instruments\LabVIEW 2016\resource\pathfinder.dll
    - Linux example: \usr\local\bin\pathfinder.so

## Using the Library
Add the Pathfinder-Lib to your code, the Pathfinder.lvproj and Tank.vi give an example using the FRC 2017 game.
