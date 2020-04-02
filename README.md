# Pathfinder
## BUILDS FAILING, CAN'T FIND MAVEN? See here: https://www.chiefdelphi.com/t/important-changes-to-pathfinder-hosting/383357
Cross-Platform, Multi-Use Motion Profiling and Trajectory Generation.

Pathfinder is a library for generating Motion Profiles, a way to smoothly fit and follow a trajectory based upon 
given waypoints. Currently, both a C and Java API are available, but can be applied to almost any application.
There also exists a C++ port which can be used by copying the files into your C++ project.

An example profile is given below, with the waypoints:  
1) X = -4, Y = -1, Angle = -45 degrees  
2) X = -2, Y = -2, Angle = 0  
3) X = 0,  Y = 0,  Angle = 0

The Graph on top is the X/Y position, and the Graph on the bottom is the Velocity.  
![](img/trajectory.png)

## Modifiers
Pathfinder supports Modifiers. Modifiers are a way to manipulate a trajectory with a given rule.  
Pathfinder supports Tank and Swerve Drive modifiers.  

Tank Drive:  
![](img/tank.png)

Swerve Drive:  
![](img/swerve.png)

## A note on calculation
Pathfinder requires some knowledge about your robot. The most important measurement is the maximum velocity, which can be either calculated or measured
emperically. 

Measuring the velocity is the best option, and can be done in multiple ways:
- Drive at full speed in a straight line. After allowing the robot to reach max speed, measure the distance it covers within a certain amount of time.
- Drive at full speed in a circle. Time how long it takes to do 10 rotations, and use the formula for circumference `C = 2*pi*r` with `r = track radius` to calculate the distance travelled in total, dividing by `10 * time` to get max velocity.

## Part of the FIRST Robotics Competition?

### For 2019:
This (old) version of Pathfinder has been published for the 2019 FRC Season, if you would like to use the legacy version
over the new and shiny [Pathfinder v2](https://github.com/GrappleRobotics/Pathfinder).

The Vendor JSON File is published here: [https://dev.imjac.in/maven/jaci/pathfinder/PathfinderOLD-latest.json](https://dev.imjac.in/maven/jaci/pathfinder/PathfinderOLD-latest.json)

Refer to the 2019 Vendor Library instructions for how to use this file, but the short and sweet of it is:
- VSCode: Command Palette (CTRL + SHIFT + P) -> WPILib: Manage Vendor Libraries -> Install new library (online) -> Paste the above URL.
- GradleRIO Standalone: Download the file and place it in `vendordeps/PathfinderOLD.json` relative to your project's root directory.

### For 2018:
Add the following lines to your build.gradle if you're using GradleRIO (2018.01.11 or higher):

### Java:
```gradle
dependencies {
    compile pathfinder()
}
```

### C++:
```gradle
model {
    frcUserProgram(NativeExecutableSpec) {
        lib library: "pathfinder"
    }
}
```

**If you're not using GradleRIO, do the following:**  
### Java
If you're not using GradleRIO, you must download this manually and copy into `~/wpilib/user/java/lib`: http://dev.imjac.in/maven/jaci/pathfinder/Pathfinder-Java/1.8/Pathfinder-Java-1.8.jar  
You also have to download this, extract it, and place `libpathfinder.so` into `~/wpilib/user/java/lib`: http://dev.imjac.in/maven/jaci/pathfinder/Pathfinder-JNI/1.8/Pathfinder-JNI-1.8-athena.zip

### C++
Download this and put `libpathfinder.a` in `~/wpilib/user/cpp/lib`: http://dev.imjac.in/maven/jaci/pathfinder/Pathfinder-Core/1.8/Pathfinder-Core-1.8-athena.zip   
Download this and extract all of its contents to `~/wpilib/user/cpp/include`: http://dev.imjac.in/maven/jaci/pathfinder/Pathfinder-Core/1.8/Pathfinder-Core-1.8-headers.zip   

## Using Pathfinder on your Architecture.
1. Download the prebuilt Pathfinder files: [https://dev.imjac.in/maven/jaci/pathfinder/](https://dev.imjac.in/maven/jaci/pathfinder/)
    - C/C++: Use Pathfinder-Core. You will need both the `headers` file and the file for your platform (e.g. 64-bit linux is `linuxx86-64`)
    - Java: Use Pathfinder-Java and Pathfinder-JNI. For Pathfinder-JNI, you will need the jar for your platform (e.g. 64-bit linux is `linuxx86-64`)

2. Add Pathfinder to your project.
    - For C/C++: Unzip the headers zip and add it to your include path. Also unzip your platform zip and link with `-lpathfinder`
    - For Java: Add both the Pathfinder-Java and Pathfinder-JNI jars to your classpath.

## Usage
To see the usage for each language variation of the API, see the README in their folder.

| Language | Folder |
| -------- | ------ |
| C        | [Pathfinder-Core](Pathfinder-Core/) |
| Java     | [Pathfinder-Java](Pathfinder-Java/) |
| LabVIEW  | [Pathfinder-LabVIEW](Pathfinder-LabVIEW/) |

### Other languages

The RobotPy project has created python bindings around the Pathfinder libraries, and can be found at https://github.com/robotpy/robotpy-pathfinder

## A word on releases
The releases on Maven (mentioned above) are built for the NI RoboRIO (v16/7 2018 image). If you go to the github releases, you can download the 1.5 version for Windows/Linux/Mac releases.

## Further reading
If you want to know more about how Pathfinder works, I highly suggest watching the seminar on [Motion Profiling by FRC Team 254](https://www.youtube.com/watch?v=8319J1BEHwM), which inspired and provided a lot of guidance for this project. Both use the same generation procedure, with some logistical differences.
