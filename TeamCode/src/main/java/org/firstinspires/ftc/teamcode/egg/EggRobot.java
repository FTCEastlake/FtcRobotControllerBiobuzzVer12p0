package org.firstinspires.ftc.teamcode.egg;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.LimelightVision;
import org.firstinspires.ftc.teamcode.common.MecanumDrive;

/**
 * Egg's hardware. Robot-specific constants live here; OpModes construct this in init().
 */
public class EggRobot {

    // TODO: measure on Egg. Inches from the tracking center (see FourBarOdometry).
    public static final double POD_X_OFFSET = 0.0;
    public static final double POD_Y_OFFSET = 0.0;
    // TODO: verify X counts up driving forward and Y counts up strafing left.
    public static final GoBildaPinpointDriver.EncoderDirection X_ENCODER = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public static final double MAX_SPEED = 1.0;
    public static final int LIMELIGHT_PIPELINE = 0;

    public final MecanumDrive drive;
    public final LimelightVision vision;

    public EggRobot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, POD_X_OFFSET, POD_Y_OFFSET, X_ENCODER, Y_ENCODER, MAX_SPEED);
        // To use the Control Hub IMU instead of Pinpoint odometry:
        // drive = new MecanumDrive(hardwareMap,
        //         RevHubOrientationOnRobot.LogoFacingDirection.UP,
        //         RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
        //         DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE,
        //         DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
        //         MAX_SPEED);
        vision = new LimelightVision(hardwareMap, LIMELIGHT_PIPELINE);
    }
}
