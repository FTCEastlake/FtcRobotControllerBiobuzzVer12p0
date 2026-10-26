package org.firstinspires.ftc.teamcode.egg;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.LimelightVision;
import org.firstinspires.ftc.teamcode.common.MecanumDrive;

/**
 * Egg's hardware. Robot-specific constants live here; OpModes construct this in init().
 */
public class EggRobot {

    // TODO: measure on Egg. Inches from the tracking center (see FourBarOdometry).
    public static final double XPOD_OFFSET = 0.0;   // sideways inches offset of the X (forward) pod from the tracking center, left positive.
    public static final double YPOD_OFFSET = 0.0;   // forward inches offset of the Y (strafe) pod from the tracking center, forward positive.
    // TODO: verify X counts up driving forward and Y counts up strafing left.
    public static final GoBildaPinpointDriver.EncoderDirection X_ENCODER = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public static final double MAX_SPEED = 1.0;
    public static final int LIMELIGHT_PIPELINE = 0;

    private static final boolean USE_PINPOINT_ODOMETRY = false;

    public final MecanumDrive drive;
    public final LimelightVision vision;

    public EggRobot(HardwareMap hardwareMap) {
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
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
                    DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE,
                    DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                    MAX_SPEED);
        }

        vision = new LimelightVision(hardwareMap, LIMELIGHT_PIPELINE);
    }
}
