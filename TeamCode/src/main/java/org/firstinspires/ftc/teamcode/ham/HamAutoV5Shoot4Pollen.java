package org.firstinspires.ftc.teamcode.ham;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// DELETE THIS if @Disabled is not being used
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * HAM AUTO V5 - SHOOT 4 POLLEN, THEN STOP
 * ========================================
 *
 * This autonomous DOES NOT move the drivetrain.
 *
 * IMPORTANT:
 * The current Ham hardware/code has no sensor that counts individual POLLEN.
 * Therefore, this version cannot truly detect "4 shots" electronically.
 * Instead, it uses a calibrated feed window that should be tuned so the
 * four preloaded POLLEN are launched, then all mechanisms stop.
 *
 * Sequence:
 * 1. Spin launcher to HamRobot.LAUNCHER_TARGET_VELOCITY.
 * 2. Wait until HamRobot.LAUNCHER_MIN_VELOCITY is reached.
 * 3. If ready, feed for FOUR_POLLEN_FEED_TIME_MS.
 * 4. Stop launcher, windmill, intake motor, and intake servos.
 * 5. Remain stationary.
 *
 * Safety behavior:
 * If launcher speed is not reached within LAUNCHER_SPINUP_TIMEOUT_MS,
 * V5 aborts the shot and stops rather than feeding POLLEN at low speed.
 */
@Autonomous(name = "Ham: Auto V5 - Shoot 4 POLLEN", group = "Ham")
@Disabled
public class HamAutoV5Shoot4Pollen extends OpMode {

    private DcMotorEx launcherMotor;
    private DcMotor intakeMotor;
    private CRServo windmillServo;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    private static final double LAUNCHER_TARGET_VELOCITY =
            HamRobot.LAUNCHER_TARGET_VELOCITY;
    private static final double LAUNCHER_MIN_VELOCITY =
            HamRobot.LAUNCHER_MIN_VELOCITY;

    private static final double FEED_POWER = 0.50;
    private static final double WINDMILL_POWER = 1.0;

    // Maximum time allowed for launcher spin-up before aborting the shot.
    private static final double LAUNCHER_SPINUP_TIMEOUT_MS = 3000.0;

    // CALIBRATE THIS ON THE ROBOT:
    // Start at 2000 ms and adjust until exactly the four PRELOADED POLLEN
    // consistently pass through the launcher, then stop feeding.
    private static final double FOUR_POLLEN_FEED_TIME_MS = 2000.0;

    private enum State {
        READY,
        SPINNING_UP,
        FEEDING_4_POLLEN,
        COMPLETE,
        ABORTED
    }

    private State state = State.READY;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private void setIntakePower(double requestedPower) {
        double safePower = Range.clip(requestedPower, -1.0, 1.0);
        intakeMotor.setPower(safePower);
        leftIntakeServo.setPower(safePower);
        rightIntakeServo.setPower(safePower);
    }

    private void startFeeding() {
        windmillServo.setPower(WINDMILL_POWER);
        setIntakePower(FEED_POWER);
    }

    private void stopShooterAndIntake() {
        launcherMotor.setVelocity(0.0);
        windmillServo.setPower(0.0);
        setIntakePower(0.0);
    }

    @Override
    public void init() {
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
        state = State.READY;

        telemetry.addLine("Ham V5 ready: Shoot 4 POLLEN - NO DRIVE");
        telemetry.addData("Launcher target", "%.0f ticks/sec", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Launcher minimum", "%.0f ticks/sec", LAUNCHER_MIN_VELOCITY);
        telemetry.addData("4-POLLEN feed window", "%.0f ms", FOUR_POLLEN_FEED_TIME_MS);
        telemetry.addLine("NOTE: feed time must be calibrated on Ham.");
        telemetry.update();
    }

    @Override
    public void start() {
        state = State.SPINNING_UP;
        stateTimer.reset();
        launcherMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
    }

    @Override
    public void loop() {
        double launcherVelocity = launcherMotor.getVelocity();

        switch (state) {
            case SPINNING_UP:
                if (launcherVelocity >= LAUNCHER_MIN_VELOCITY) {
                    startFeeding();
                    state = State.FEEDING_4_POLLEN;
                    stateTimer.reset();
                } else if (stateTimer.milliseconds() >= LAUNCHER_SPINUP_TIMEOUT_MS) {
                    stopShooterAndIntake();
                    state = State.ABORTED;
                }
                break;

            case FEEDING_4_POLLEN:
                if (stateTimer.milliseconds() >= FOUR_POLLEN_FEED_TIME_MS) {
                    stopShooterAndIntake();
                    state = State.COMPLETE;
                }
                break;

            case COMPLETE:
            case ABORTED:
            case READY:
            default:
                // Intentionally remain stopped and stationary.
                break;
        }

        telemetry.addData("V5 State", state);
        telemetry.addData("Launcher target", "%.0f", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Launcher actual", "%.0f", launcherVelocity);
        telemetry.addData("Launcher ready", launcherVelocity >= LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Feed timer", "%.0f / %.0f ms",
                state == State.FEEDING_4_POLLEN ? stateTimer.milliseconds() : 0.0,
                FOUR_POLLEN_FEED_TIME_MS);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopShooterAndIntake();
    }
}
