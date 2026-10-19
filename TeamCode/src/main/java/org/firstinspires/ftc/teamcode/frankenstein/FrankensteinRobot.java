package org.firstinspires.ftc.teamcode.frankenstein;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.LimelightVision;
import org.firstinspires.ftc.teamcode.common.MecanumDrive;

/**
 * Frankenstein's hardware. Robot-specific constants live here; OpModes construct this in init().
 */
public class FrankensteinRobot {

    // TODO: measure on Frankenstein. Inches from the tracking center (see FourBarOdometry).
    public static final double XPOD_OFFSET = 3.6875;   // sideways inches offset of the X (forward) pod from the tracking center, left positive.
    public static final double YPOD_OFFSET = -6.625;   // forward inches offset of the Y (strafe) pod from the tracking center, forward positive.
    // TODO: verify X counts up driving forward and Y counts up strafing left.
    public static final GoBildaPinpointDriver.EncoderDirection X_ENCODER = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static final double MAX_SPEED = 1.0;
    public static final int LIMELIGHT_PIPELINE = 0;

    public final MecanumDrive drive;
    public final LimelightVision vision;

    public FrankensteinRobot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, XPOD_OFFSET, YPOD_OFFSET, X_ENCODER, Y_ENCODER,
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, MAX_SPEED);
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
