package org.firstinspires.ftc.teamcode.ham;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.common.LimelightVision;
import org.firstinspires.ftc.teamcode.common.MecanumDrive;

/**
 * Ham's hardware. Robot-specific constants live here; OpModes construct this in init().
 */
public class HamRobot {

    // TODO: measure on Ham. Inches from the tracking center (see FourBarOdometry).
    public static final double XPOD_OFFSET = 0.0;   // sideways inches offset of the X (forward) pod from the tracking center, left positive.
    public static final double YPOD_OFFSET = 0.0;   // forward inches offset of the Y (strafe) pod from the tracking center, forward positive.
    // TODO: verify X counts up driving forward and Y counts up strafing left.
    public static final GoBildaPinpointDriver.EncoderDirection X_ENCODER = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public static final double MAX_SPEED = 1.0;
    public static final int LIMELIGHT_PIPELINE = 0;
    public static final int LAUNCHER_TARGET_VELOCITY = 1250;
    public static final int LAUNCHER_MIN_VELOCITY = 1200;

    private static final boolean USE_PINPOINT_ODOMETRY = false;

    public final MecanumDrive drive;
    public final DcMotor intakeMotor;
    public final DcMotorEx launcherMotor;
    public final CRServo windmillServo;
    public final CRServo leftIntakeServo;
    public final CRServo rightIntakeServo;
//    public final LimelightVision vision;

    public HamRobot(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);
        stopShooter();

        if (USE_PINPOINT_ODOMETRY)
        {
            // Uses the Pinpoint odometry instead of Control Hub IMU.
            drive = new MecanumDrive(hardwareMap, XPOD_OFFSET, YPOD_OFFSET, X_ENCODER, Y_ENCODER,
                    DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE,
                    DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, MAX_SPEED);
        }
        else
        {
            // Uses the Control Hub IMU instead of Pinpoint odometry.
            drive = new MecanumDrive(hardwareMap,
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP,
                    DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE,
                    DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                    MAX_SPEED);
        }

//        vision = new LimelightVision(hardwareMap, LIMELIGHT_PIPELINE);
    }

    /** Runs the launcher and feeds only after it reaches the minimum shooting speed. */
    public void runShooter(boolean launchRequested, double intakePower) {
        launcherMotor.setVelocity(launchRequested ? LAUNCHER_TARGET_VELOCITY : 0);

        if (launchRequested && launcherMotor.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            windmillServo.setPower(0);
        }

        setIntakePower(intakePower);
    }

    /** Sets the intake motor and both feeder servos to the same power. */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    /** Stops all shooter and intake hardware. */
    public void stopShooter() {
        launcherMotor.setVelocity(0);
        windmillServo.setPower(0);
        setIntakePower(0);
    }
}
