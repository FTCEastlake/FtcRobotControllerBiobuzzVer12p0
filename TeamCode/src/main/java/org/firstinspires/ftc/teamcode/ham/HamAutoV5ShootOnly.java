package org.firstinspires.ftc.teamcode.ham;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * HAM AUTO V5 - SHOOT ONLY
 * ========================
 *
 * This autonomous DOES NOT move the drivetrain.
 *
 * Sequence:
 * 1. Spin launcher to HamRobot.LAUNCHER_TARGET_VELOCITY.
 * 2. Wait until launcher reaches HamRobot.LAUNCHER_MIN_VELOCITY,
 *    or until the spin-up timeout expires.
 * 3. Run the windmill and intake for FEED_TIME_MS.
 * 4. Stop launcher, windmill, intake motor, and intake servos.
 * 5. Remain stationary for the rest of autonomous.
 *
 * This version is useful for testing/tuning the shooter without combining
 * shooting with Pedro Pathing or robot movement.
 */
@Autonomous(name = "Ham: Auto V5 - Shoot Only", group = "Ham")
@Disabled
public class HamAutoV5ShootOnly extends OpMode {

    private DcMotorEx launcherMotor;
    private DcMotor intakeMotor;
    private CRServo windmillServo;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    // These values come from HamRobot.java so the shooter settings stay centralized.
    private static final double LAUNCHER_TARGET_VELOCITY =
            HamRobot.LAUNCHER_TARGET_VELOCITY;

    private static final double LAUNCHER_MIN_VELOCITY =
            HamRobot.LAUNCHER_MIN_VELOCITY;

    // Starting values to tune on the physical robot.
    private static final double FEED_POWER = 0.50;
    private static final double WINDMILL_POWER = 1.0;
    private static final long LAUNCHER_SPINUP_TIMEOUT_MS = 3000;
    private static final long FEED_TIME_MS = 2000;

    /** Safely controls the intake motor and both intake CR servos together. */
    private void setIntakePower(double requestedPower) {
        double safePower = Range.clip(requestedPower, -1.0, 1.0);
        intakeMotor.setPower(safePower);
        leftIntakeServo.setPower(safePower);
        rightIntakeServo.setPower(safePower);
    }

    /** Starts feeding POLLEN into the launcher. */
    private void startFeeding() {
        windmillServo.setPower(WINDMILL_POWER);
        setIntakePower(FEED_POWER);
    }

    /** Stops every shooter/intake mechanism. */
    private void stopShooterAndIntake() {
        launcherMotor.setVelocity(0.0);
        windmillServo.setPower(0.0);
        setIntakePower(0.0);
    }

    /**
     * Complete V5 routine. No drive command is included.
     */
    private Command autoRoutine() {
        return sequential(
                // Spin up launcher.
                instant(() -> launcherMotor.setVelocity(LAUNCHER_TARGET_VELOCITY)),

                // Wait until shooting speed, but do not wait forever.
                race(
                        waitUntil(() ->
                                launcherMotor.getVelocity() >= LAUNCHER_MIN_VELOCITY),
                        waitMs(LAUNCHER_SPINUP_TIMEOUT_MS)
                ),

                // Feed POLLEN.
                instant(this::startFeeding),
                waitMs(FEED_TIME_MS),

                // Stop all shooter/intake hardware.
                instant(this::stopShooterAndIntake)
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);

        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5)
        );

        stopShooterAndIntake();

        telemetry.addLine("Ham V5 ready: Shoot Only - NO DRIVE MOVEMENT");
        telemetry.addData("Launcher target", "%.0f ticks/sec",
                LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Launcher minimum", "%.0f ticks/sec",
                LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Windmill power", "%.0f%%", WINDMILL_POWER * 100.0);
        telemetry.addData("Feed power", "%.0f%%", FEED_POWER * 100.0);
        telemetry.addData("Feed time", "%d ms", FEED_TIME_MS);
        telemetry.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        Scheduler.execute();

        double launcherVelocity = launcherMotor.getVelocity();

        telemetry.addLine("V5 SHOOT ONLY - drivetrain stationary");
        telemetry.addData("Launcher target", "%.0f", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Launcher actual", "%.0f", launcherVelocity);
        telemetry.addData("Launcher ready",
                launcherVelocity >= LAUNCHER_MIN_VELOCITY);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        stopShooterAndIntake();
    }
}
