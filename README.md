# FTCMythos2026
This project contains the code for 1369's 2026 Robot "Frank".

![Robot Image](/images/comp.png)

See also:
- Code from [2024](https://github.com/MiddletonRobotics/FTCMythos2024/) and [2025](https://github.com/MiddletonRobotics/FTCMythos2025/).

## Highlights
- Selectable Autonomous

    Depending on the autonomous routine that as an alliance we have agreed to run, the autonomous is built in such a way that if the actions and commands are already built in the AutoFactory, we can select any autonomous we could need with only one OpMode. This allows us to not only ensures that we select the right autonomous (with the correct feedback from the LED), but forces us to check if our controllers are connected.

- Automcatic Ball Control

    Frank has three sensors which allows it to see how many balls it is holding at a time. The topmost and middle compartments have a beam-break, and the bottom has a REV Color Sensor V3 (using distance measurement to see if there's a ball present). These sensors enable us to run our automatic shooting command, which shoots the balls no matter how many are present in the intake or where they currently are.

- Control Bindings

    Frank only requires a single controller to be able to operate all major functionaility (except for endgame mechanisms which we never used). However, we do have a second controller that serves more as a debugging controller, allowing us to rezero incase something goes wrong, and toggle buttons for the driver to switch to manual mode.

- AprilTag Detection and Odometry Fusion

    Frank as one Limelight 3A mounted directly above the third intake roller, which is always reading for AprilTags on the field. This helps us relocalize over the timespan of teleop, where are odometry tends to drift, and keeps our turret aimed directly towards the goal.

## Notable Package Functions
- [`org.firstinspires.ftc.library`](/TeamCode/src/main/java/org/firstinspires/ftc/library/)

  Contains all the base files that our library, MiddletonLib runs on. This contains a lot of WPILib utility files such as Pose2d, Rotation2d and Pair<T>. This also contains a port of FTCLib, fixing noteable errors and custom added features.

- [`org.firstinspires.ftc.teamcode.autonomous`](/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/)

  Contains all the files needed to maintain run our autonomous routines. This contains two enumerators to store the setup position and auto type planned to run, an AutoRoutine class which stores a Location, Auto and a functional embedded pair to get the path starting location, ending location and commandFactory.

  All of the AutoRoutines are stored in the AutoFactory, which for each type of auto in that specific location creates the desired path and the commandFactory.

  Furthermore, a custom AutoChooser takes all specified paths and adds it to a List, which can then be retrieved with the `getDesiredProgram()` function.

- [`org.firstinspires.ftc.teamcode.command_factory`](/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/command_factories/)

  Contains all the commands that require only that specific subsystem (IntakeFactory, ShooterFactory, etc.).

- [`org.firstinspires.ftc.teamcode.subsystems`](/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/)

  Contains all subsystems, including (position control subsystems, which Climber and Turret), motor subsystems (Drivetrain, Intake, Shooter), among others.
