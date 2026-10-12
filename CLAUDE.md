# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

The FIRST Tech Challenge (FTC) Robot Controller SDK, v12.0, for the BIOBUZZ (2026–2027) season. It builds an Android app that runs on the robot's Control Hub or Robot Controller phone. Team robot code goes in the `TeamCode` module. That module is currently empty apart from its readme.

## Build commands

On Windows use `gradlew.bat`, or `./gradlew` from Git Bash. Android Studio is the usual IDE (Narwhal 3 Feature Drop or later).

- Build the app: `./gradlew :TeamCode:assembleDebug`
- Install to a connected Control Hub or phone over adb: `./gradlew :TeamCode:installDebug`
- Clean: `./gradlew clean`
- Compile check only (quickest): `./gradlew :TeamCode:compileDebugJavaWithJavac`

Gradle needs a JDK 17 or later. The default Java on this machine is too old (JDK 11 or Java 8), so from Git Bash set `export JAVA_HOME='C:\Program Files\Android\Android Studio1\jbr'` first. Android Studio uses its bundled JDK automatically.

The project has no unit tests and no lint configuration. To confirm code works, build it and run it on robot hardware, where the Driver Station app selects and starts OpModes.

## Architecture

- **`FtcRobotController/`**: the SDK's app shell (the activity, OpMode registration, manifest) plus about 66 sample OpModes in `src/main/java/org/firstinspires/ftc/robotcontroller/external/samples`. Treat this module as read-only. FIRST updates it in new SDK releases, and local edits get in the way of merging those updates.
- **`TeamCode/`**: the application module (`apply plugin: 'com.android.application'`) that depends on `:FtcRobotController`. All team code goes under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- **SDK libraries**: the core SDK comes as prebuilt Maven artifacts listed in `build.dependencies.gradle` (`org.firstinspires.ftc:RobotCore`, `Hardware`, `Vision`, `FtcCommon`, and others, all at 12.0.0), not as source code. Check the [FTC Javadoc](https://javadoc.io/doc/org.firstinspires.ftc) for these APIs.
- **Build configuration**: `build.common.gradle` holds the shared Android configuration: compileSdk 30, minSdk 24, targetSdk 28, **Java 8 source level** (so no `var`, records, or other newer language features), and debug signing with `libs/ftc.debug.keystore`. The app's versionCode and versionName come from `FtcRobotController/src/main/AndroidManifest.xml`. Put build customizations in `TeamCode/build.gradle`, not in the root or common Gradle files.

## Team code layout

Several robots share the same drivetrain, odometry and vision hardware, so that code lives in one shared library. Each robot gets its own package.

- **`teamcode/common/`**: the shared library. Put hardware support shared by every robot here, not in one robot's package.
  - `FourBarOdometry`: wraps the Pinpoint. The constructor takes the pod X/Y offsets in inches and the two encoder directions.
  - `MecanumDrive`: field-centric mecanum drive with two constructors that choose where heading comes from. The **Pinpoint constructor** (the default) takes the pod offsets, encoder directions and max speed; it also tracks X/Y position (`getOdometry()`). The **Control Hub IMU constructor** takes the hub's logo and USB directions, each motor's direction and max speed; it tracks heading only, and `getOdometry()` returns `null`. Call `update()` once per loop before `driveFieldCentric(forward, strafeRight, turnCW)`.
  - `LimelightVision`: selects a pipeline and reads results. `getFieldPose(heading, unit)` returns a MegaTag2 AprilTag pose.
- **`teamcode/<robot>/`**: one package each for `frankenstein`, `ham`, `egg` and `burrito`. `<Name>Robot` holds that robot's constants (pod offsets, encoder directions, max speed, pipeline) and builds its hardware. OpModes create it in `init()`. The pod offsets and encoder directions are still placeholders (`TODO`) until they're measured on each robot.
- **Conventions:** all distances are in inches. Coordinates follow the Pinpoint convention: +X forward, +Y left, heading counter-clockwise positive. Team OpModes **extend `OpMode` (iterative), not `LinearOpMode`**; use `init`/`start`/`loop`/`stop` and don't block inside `loop()`.

## Writing OpModes

- OpModes are found at runtime through annotations, so they need no manual registration. A class extending `OpMode` (this team's convention) is registered by `@TeleOp(name=..., group=...)` or `@Autonomous(...)`. `@Disabled` hides an OpMode from the Driver Station list.
- To start a new OpMode, copy the closest sample from `external/samples` into `TeamCode`, rename it, update the `name` in its annotation, and remove `@Disabled`.
- Sample name prefixes show what a sample covers: `Basic` (skeletons), `Sensor` (one sensor), `Robot` (2-motor drive base), `Concept` (a single technique). The full conventions are in `FtcRobotController/.../samples/sample_conventions.md`.
- Hardware objects are fetched by name with `hardwareMap.get(DcMotor.class, "left_drive")`. These names must match the robot's configuration on the Robot Controller exactly, so they cannot be checked from this repository.
- Webcam calibrations go in `TeamCode/src/main/res/xml/teamwebcamcalibrations.xml`.

## Robot hardware

**Mecanum drivetrain**
- Motors: 4× goBILDA Yellow Jacket, SKU 5203-2402-0019. That SKU is the 19.2:1 gearbox: about 312 RPM and 537.7 encoder ticks per output revolution.
- Wheels: 4× GripForce mecanum, 104 mm diameter.
- Motor configuration names: `"leftFront"`, `"leftRear"`, `"rightFront"`, `"rightRear"`. Get them with `hardwareMap.get(DcMotor.class, "leftFront")` or `DcMotorEx.class`. On a typical mecanum build the left-side motors need `setDirection(DcMotor.Direction.REVERSE)`; check this on the robot.
- Starting samples: `RobotTeleopMecanumFieldRelativeDrive.java` and `BasicOmniOpMode_Linear.java`.

**Driver controller**
- Logitech F310 gamepad controller.

**Odometry**
- 2× goBILDA 4-Bar odometry pods, one forward (X) and one strafe (Y), wired to a goBILDA Pinpoint Odometry Computer over I2C.
- The driver is `GoBildaPinpointDriver`, which ships with the SDK. Set it up with `setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)`, then `setOffsets(...)` measured on this robot. The offsets in the sample were tuned for a different robot. Reference sample: `SensorGoBildaPinpoint.java`.
- The Pinpoint's configuration name isn't recorded yet. The sample uses `"pinpoint"`.

**Vision**
- Limelight 3A, SDK class `Limelight3A`. Reference sample: `SensorLimelight3A.java`, which uses the configuration name `"limelight"` and `pipelineSwitch(0)`. Pipelines are set up on the Limelight itself through its web UI, not in this repository.
