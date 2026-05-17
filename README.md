![alt text](https://github.com/deep-sea-tactics/Dyver/blob/main/dyver_light.png "Dyver Logo")

> [!NOTE]
> Dyver has not reached its MVP state. As such, some features listed here may be completely missing. Under construction!

# Dyver
Dyver is a set of applications specifically tailored to the operation of an ROV. It contains both a topside user interface as well as robot client software.

### Topside
Dyver Topside (target `DyverTopside`) is your go-to command center for all of your ROVs. It can recieve telemetry, send instructions, and stream camera data from any robot running the Dyver Robot daemon.

### Robot
Dyver Robot (target `DyverRobot`) will communicate with topside, manage sensors, and perform general flight controller tasks. It is important to note that the Dyver Robot application **is not a driver for a flight controller.** It is designed to run on what would traditionally be a companion computer, and uses an external IMU.

## Contributing
### Getting Started
Ready to jump on-deck and help us develop Dyver? Wonderful!

The best first place to look will be the `issues` tab on GitHub. There, you will find an up-to-date list of all of our tasks and their statuses. Grab a `good first issue` and get to work! We recommend you read/write documentation if you're still stuck on where to begin.

### Conventions
Keep the code clean!

* Units of Measurement: 
If the unit of measurement is unknown, assume SI standard. Place unit annotations after any variables that have units. Base units are preferrable, but some SI-prefixed units are acceptable (e.g. kg).

* Formatting: 
Formatting rules are defined in the `.clang-format` file and should be used. Save our time, format your code. Snake case is heavily encouraged. Use `.h` for header files and `.cpp` for source files.

* Documentation: 
Write documentation with Doxygen. Use `javadoc` style documentation comments when documenting internal code. Prefer a documentation on the function of the code rather than the implementation of the code, but always prefer documenting the implementation of the code if it has been declared elsewhere (`.h` vs `.cpp`).

* Dependencies: 
Create an issue requesting a dependency before any pull requests or changes. After a project manager (most like Estelle Coonan) reviews the issue, it will then be decided upon. Please keep things organized.

* Compilation: 
Prefer `clang` and its set of tools. Release code will be compiled with `clang`, so ensure that your code works on `clang` before pushing it.

* Testing: 
`src/test/test.cpp`. Write testing code here. Tests are highly encouraged for code whose functionality can be easily tested.

## Installation (Users)
Run the installer program **with an internet connection**, and it'll do the rest!

## Usage
Install a binary of the topside application onto the topside computer, and install a binary of the robot application onto the robot.
> [!NOTE]
> Advanced usage such as modifying cache entries or configuring the network does not have a publicly available source of documentation. This is being actively fixed.

## Installation (Developers)

1. Build on Linux. If you haven't already, install `cmake` and `clang` with its associated tools (such as `clang-format`)
2. Clone the repository.
3. With the repository's master directory as your current working directory, execute `source deps.sh` to install dependencies.
5. If you've done everything correctly, any Dyver target you choose should build successfully. Nice!
