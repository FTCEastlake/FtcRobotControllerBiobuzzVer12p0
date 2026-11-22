package org.firstinspires.ftc.teamcode.frankenstein;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.AprilTagAligner;
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

    public static final boolean USE_LIMELIGHT = true;
    public static final int LIMELIGHT_PIPELINE = 0;

    private static final boolean USE_PINPOINT_ODOMETRY = true;

    public final MecanumDrive drive;
    public final LimelightVision vision;
    public final AprilTagAligner tagAligner;

    public FrankensteinRobot(HardwareMap hardwareMap) {

        // Initialize the drive system.
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

        // Initialize the vision system.
        vision = USE_LIMELIGHT ? new LimelightVision(hardwareMap, LIMELIGHT_PIPELINE) : null;
        tagAligner = USE_LIMELIGHT ? new AprilTagAligner(drive, vision) : null;

        // Initialize the tag aligner.
        if (USE_LIMELIGHT) {
            // Note: adjust these parameters to tune the tag alignment behavior. The defaults are reasonable.
            double alignToleranceIn = 0.50;         // position tolerance (inches)
            double alignYawToleranceDeg = 0.5;      // yaw tolerance (degrees)    
            double alignPowerPerInch = 0.15;        // drive power per inch of error (default 0.15)
            double alignTurnPowerPerDegree = 0.015; // turn power per degree of yaw error (default 0.015)
            double alignMaxPower = 0.30;            // maximum forward/strafe power (default 0.30)
            double alignMaxTurnPower = 0.25;        // maximum turn power (default 0.25)
            tagAligner.Init(
                    alignToleranceIn, alignYawToleranceDeg, alignPowerPerInch, 
                    alignTurnPowerPerDegree, alignMaxPower, alignMaxTurnPower);
        }
    }
}
