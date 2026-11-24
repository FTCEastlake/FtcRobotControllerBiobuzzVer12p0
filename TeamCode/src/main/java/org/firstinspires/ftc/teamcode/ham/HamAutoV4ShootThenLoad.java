package org.firstinspires.ftc.teamcode.ham;

import org.firstinspires.ftc.teamcode.ham.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


/**
 * HAM AUTO V4 - SHOOT 4 POLLEN, THEN DRIVE TO RED LOADING ZONE
 * =============================================================
 *
 * Ham starting orientation:
 *
 *              NORTH
 *                ↑
 *          Launcher / Back
 *
 *               HAM
 *
 *            Intake / Front
 *                ↓
 *              SOUTH
 *
 * Pinpoint robot convention:
 *
 * +X = robot forward
 * +Y = robot left
 *
 * Because Ham starts facing south:
 *
 * +X = south
 * -X = north
 * +Y = east
 * -Y = west
 *
 * Intended route after shooting:
 *
 * 1. Move 24 inches west.
 * 2. Move 84 inches north.
 * 3. Move another 24 inches west into the Red Loading Zone.
 *
 * Absolute Pedro coordinates:
 *
 * Start:      (  0,   0)
 * Waypoint 1: (  0, -24)
 * Waypoint 2: (-84, -24)
 * Load Zone:  (-84, -48)
 *
 * IMPORTANT:
 * Verify the -X and -Y directions with short physical tests before
 * running the full autonomous route.
 */
@Autonomous(
        name = "Ham: Auto V4 - Shoot Then Load",
        group = "Ham"
)
@Disabled
public class HamAutoV4ShootThenLoad extends OpMode {

    //*************************************************************
    // Pedro Pathing
    //*************************************************************

    private Follower follower;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();


    //*************************************************************
    // Shooter and intake hardware
    //*************************************************************

    private DcMotorEx launcherMotor;

    private DcMotor intakeMotor;

    private CRServo windmillServo;

    private CRServo leftIntakeServo;

    private CRServo rightIntakeServo;


    //*************************************************************
    // Launcher settings
    //*************************************************************

    /*
     * These values come from HamRobot.java.
     *
     * Changing the launcher target in HamRobot.java automatically
     * changes it here.
     */
    private static final double LAUNCHER_TARGET_VELOCITY =
            HamRobot.LAUNCHER_TARGET_VELOCITY;

    private static final double LAUNCHER_MIN_VELOCITY =
            HamRobot.LAUNCHER_MIN_VELOCITY;


    /*
     * Intake/feed power while shooting.
     */
    private static final double FEED_POWER = 0.50;

    /*
     * Windmill power while feeding POLLEN.
     */
    private static final double WINDMILL_POWER = 1.00;


    /*
     * If the launcher does not reach minimum shooting speed within
     * 3 seconds, V4 aborts the shooting portion.
     *
     * It will NOT feed POLLEN at low launcher speed.
     */
    private static final double LAUNCHER_SPINUP_TIMEOUT_MS =
            3000.0;


    /*
     * This is a timed feed window for the four preloaded POLLEN.
     *
     * Ham currently has no sensor that counts individual POLLEN.
     * Tune this value using V5 Shoot 4 POLLEN first.
     *
     * Starting test value:
     *
     * 3000 ms = 3 seconds
     */
    private static final double FOUR_POLLEN_FEED_TIME_MS =
            3000.0;


    //*************************************************************
    // V4 route poses
    //*************************************************************

    /*
     * Ham starts at the local Pedro origin.
     *
     * Heading remains at 0 degrees for the entire route.
     */
    private final Pose startPose =
            poseFactory.of(0, 0, 0);


    /*
     * First move:
     *
     * 24 inches west.
     *
     * Since west is -Y from Ham's starting orientation:
     */
    private final Pose waypoint1 =
            poseFactory.of(0, -24, 0);


    /*
     * Second move:
     *
     * 84 inches north.
     *
     * Since north is -X from Ham's starting orientation:
     */
    private final Pose waypoint2 =
            poseFactory.of(-84, -24, 0);


    /*
     * Third move:
     *
     * Another 24 inches west into the Red Loading Zone.
     */
    private final Pose loadPose =
            poseFactory.of(-84, -48, 0);


    //*************************************************************
    // Autonomous state machine
    //*************************************************************

    private enum AutoState {

        /*
         * Waiting for START.
         */
        READY,

        /*
         * Launcher is accelerating.
         */
        SPINNING_UP,

        /*
         * Windmill and intake are feeding the four POLLEN.
         */
        FEEDING_4_POLLEN,

        /*
         * Shooting is complete and Pedro is driving.
         */
        DRIVING_TO_LOAD_ZONE,

        /*
         * Autonomous routine completed.
         */
        COMPLETE,

        /*
         * Launcher failed to reach minimum speed.
         *
         * Shooting was skipped, but the robot will still drive to
         * the Loading Zone.
         */
        SHOOTING_ABORTED
    }

    private AutoState state = AutoState.READY;

    private final ElapsedTime stateTimer =
            new ElapsedTime();

    private boolean driveRoutineScheduled = false;


    //*************************************************************
    // Pedro paths
    //*************************************************************

    /**
     * Drive 24 inches west from the starting position.
     */
    private Path startToWaypoint1() {

        return line(startPose, waypoint1)
                .linear(startPose, waypoint1);
    }


    /**
     * Drive 84 inches north.
     */
    private Path waypoint1ToWaypoint2() {

        return line(waypoint1, waypoint2)
                .linear(waypoint1, waypoint2);
    }


    /**
     * Drive another 24 inches west into the Red Loading Zone.
     */
    private Path waypoint2ToLoad() {

        return line(waypoint2, loadPose)
                .linear(waypoint2, loadPose);
    }


    /**
     * Complete drive routine.
     *
     * sequential() ensures each path finishes before the next path begins.
     */
    private Command driveRoutine() {

        return sequential(

                follow(
                        follower,
                        startToWaypoint1()
                ),

                follow(
                        follower,
                        waypoint1ToWaypoint2()
                ),

                follow(
                        follower,
                        waypoint2ToLoad()
                )
        );
    }


    //*************************************************************
    // Shooter and intake methods
    //*************************************************************

    /**
     * Sets the intake motor and two intake CR servos.
     *
     * Range.clip prevents commands below -1.0 or above +1.0.
     */
    private void setIntakePower(double requestedPower) {

        double safePower =
                Range.clip(
                        requestedPower,
                        -1.0,
                        1.0
                );

        intakeMotor.setPower(safePower);

        leftIntakeServo.setPower(safePower);

        rightIntakeServo.setPower(safePower);
    }


    /**
     * Starts the windmill and intake feed system.
     */
    private void startFeeding() {

        windmillServo.setPower(
                WINDMILL_POWER
        );

        setIntakePower(
                FEED_POWER
        );
    }


    /**
     * Stops all shooting and intake mechanisms.
     */
    private void stopShooterAndIntake() {

        launcherMotor.setVelocity(0.0);

        windmillServo.setPower(0.0);

        setIntakePower(0.0);
    }


    /**
     * Schedules the three-segment Pedro drive routine once.
     */
    private void startDriveRoutine() {

        if (!driveRoutineScheduled) {

            driveRoutineScheduled = true;

            schedule(
                    driveRoutine()
            );

            state =
                    AutoState.DRIVING_TO_LOAD_ZONE;
        }
    }


    //*************************************************************
    // FTC initialization
    //*************************************************************

    @Override
    public void init() {

        Scheduler.reset();


        // Intake and launcher hardware mappings.
        intakeMotor =
                hardwareMap.get(
                        DcMotor.class,
                        "intakeMotor"
                );

        launcherMotor =
                hardwareMap.get(
                        DcMotorEx.class,
                        "launcherMotor"
                );

        windmillServo =
                hardwareMap.get(
                        CRServo.class,
                        "windmillServo"
                );

        leftIntakeServo =
                hardwareMap.get(
                        CRServo.class,
                        "leftIntakeServo"
                );

        rightIntakeServo =
                hardwareMap.get(
                        CRServo.class,
                        "rightIntakeServo"
                );


        // Intake motor setup.
        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );


        /*
         * The right intake servo is physically mirrored.
         *
         * Reversing it lets both intake servos move POLLEN in the
         * same physical direction.
         */
        rightIntakeServo.setDirection(
                DcMotorSimple.Direction.REVERSE
        );


        // Windmill direction from the updated HamRobot.java.
        windmillServo.setDirection(
                DcMotorSimple.Direction.FORWARD
        );


        // Launcher encoder velocity control.
        launcherMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        launcherMotor.setPIDFCoefficients(

                DcMotor.RunMode.RUN_USING_ENCODER,

                new PIDFCoefficients(
                        40,
                        0,
                        0,
                        12.5
                )
        );


        // Start all mechanisms safely stopped.
        stopShooterAndIntake();


        // Create the Pedro follower.
        follower =
                Constants.create(hardwareMap);

        follower.setPose(
                startPose
        );

        follower.update();


        state =
                AutoState.READY;

        driveRoutineScheduled =
                false;


        telemetry.addLine(
                "Ham V4 ready"
        );

        telemetry.addLine(
                "Shoot 4 POLLEN, then drive to Red Loading Zone"
        );

        telemetry.addLine(
                "Route: West 24 -> North 84 -> West 24"
        );

        telemetry.addData(
                "Start",
                "X %.1f  Y %.1f",
                startPose.x(),
                startPose.y()
        );

        telemetry.addData(
                "Waypoint 1",
                "X %.1f  Y %.1f",
                waypoint1.x(),
                waypoint1.y()
        );

        telemetry.addData(
                "Waypoint 2",
                "X %.1f  Y %.1f",
                waypoint2.x(),
                waypoint2.y()
        );

        telemetry.addData(
                "Load Zone",
                "X %.1f  Y %.1f",
                loadPose.x(),
                loadPose.y()
        );

        telemetry.addData(
                "Launcher target",
                "%.0f ticks/sec",
                LAUNCHER_TARGET_VELOCITY
        );

        telemetry.addData(
                "Launcher minimum",
                "%.0f ticks/sec",
                LAUNCHER_MIN_VELOCITY
        );

        telemetry.addData(
                "4-POLLEN feed time",
                "%.0f ms",
                FOUR_POLLEN_FEED_TIME_MS
        );

        telemetry.update();
    }


    //*************************************************************
    // FTC START
    //*************************************************************

    @Override
    public void start() {

        state =
                AutoState.SPINNING_UP;

        stateTimer.reset();

        launcherMotor.setVelocity(
                LAUNCHER_TARGET_VELOCITY
        );
    }


    //*************************************************************
    // FTC autonomous loop
    //*************************************************************

    @Override
    public void loop() {

        /*
         * Pedro must be updated every loop, even while Ham is
         * stationary and shooting.
         */
        follower.update();

        /*
         * Ivy must also be updated every loop.
         */
        Scheduler.execute();


        double launcherVelocity =
                launcherMotor.getVelocity();


        switch (state) {

            case SPINNING_UP:

                /*
                 * Begin feeding only after the launcher reaches the
                 * configured minimum velocity.
                 */
                if (launcherVelocity
                        >= LAUNCHER_MIN_VELOCITY) {

                    startFeeding();

                    state =
                            AutoState.FEEDING_4_POLLEN;

                    stateTimer.reset();

                } else if (
                        stateTimer.milliseconds()
                                >= LAUNCHER_SPINUP_TIMEOUT_MS
                ) {

                    /*
                     * Launcher did not reach shooting speed.
                     *
                     * Do not feed POLLEN at low speed.
                     */
                    stopShooterAndIntake();

                    state =
                            AutoState.SHOOTING_ABORTED;

                    stateTimer.reset();
                }

                break;


            case FEEDING_4_POLLEN:

                /*
                 * Keep feeding for the calibrated four-POLLEN window.
                 */
                if (stateTimer.milliseconds()
                        >= FOUR_POLLEN_FEED_TIME_MS) {

                    stopShooterAndIntake();

                    startDriveRoutine();
                }

                break;


            case SHOOTING_ABORTED:

                /*
                 * Shooting was aborted, but Ham can still attempt the
                 * Loading Zone route.
                 */
                startDriveRoutine();

                break;


            case DRIVING_TO_LOAD_ZONE:

                /*
                 * When Pedro is no longer following a path, the last
                 * path has completed.
                 *
                 * Depending on the installed Pedro version, the exact
                 * completion behavior should be verified during testing.
                 */
                if (!follower.following()) {

                    state =
                            AutoState.COMPLETE;
                }

                break;


            case COMPLETE:

                /*
                 * Keep all mechanisms stopped.
                 */
                stopShooterAndIntake();

                break;


            case READY:
            default:

                /*
                 * Remain stopped until START.
                 */
                break;
        }


        //*********************************************************
        // Driver Station telemetry
        //*********************************************************

        telemetry.addData(
                "V4 State",
                state
        );

        telemetry.addData(
                "X",
                "%.2f in",
                follower.pose().x()
        );

        telemetry.addData(
                "Y",
                "%.2f in",
                follower.pose().y()
        );

        telemetry.addData(
                "Heading",
                "%.2f deg",
                Math.toDegrees(
                        follower.pose().heading()
                )
        );

        telemetry.addData(
                "Launcher target",
                "%.0f",
                LAUNCHER_TARGET_VELOCITY
        );

        telemetry.addData(
                "Launcher actual",
                "%.0f",
                launcherVelocity
        );

        telemetry.addData(
                "Launcher ready",
                launcherVelocity
                        >= LAUNCHER_MIN_VELOCITY
        );

        telemetry.addData(
                "State timer",
                "%.0f ms",
                stateTimer.milliseconds()
        );

        telemetry.addData(
                "Follower mode",
                follower.mode()
        );

        telemetry.update();
    }


    //*************************************************************
    // FTC STOP
    //*************************************************************

    @Override
    public void stop() {

        Scheduler.reset();

        stopShooterAndIntake();
    }
}