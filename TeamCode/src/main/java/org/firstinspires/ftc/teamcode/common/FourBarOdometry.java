package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * goBILDA 4-Bar odometry: two pods (X = forward, Y = strafe) read by a Pinpoint Odometry Computer.
 *
 * All distances are in inches. Coordinate frame (Pinpoint convention): +X forward, +Y left,
 * heading counter-clockwise positive.
 *
 * {@link #update()} must be called once per loop before reading the pose.
 */
public class FourBarOdometry {

    /** Robot configuration name of the Pinpoint. */
    public static final String PINPOINT_NAME = "pinpoint";

    private final GoBildaPinpointDriver pinpoint;

    /**
     * @param podXOffset how far sideways from the tracking center the X (forward) pod is, in inches.
     *                   Left of center is positive, right is negative.
     * @param podYOffset how far forward from the tracking center the Y (strafe) pod is, in inches.
     *                   Forward of center is positive, behind is negative.
     * @param xDirection set so the X pod count increases when the robot moves forward.
     * @param yDirection set so the Y pod count increases when the robot moves left.
     */
    public FourBarOdometry(HardwareMap hardwareMap,
                           double podXOffset, double podYOffset,
                           GoBildaPinpointDriver.EncoderDirection xDirection,
                           GoBildaPinpointDriver.EncoderDirection yDirection) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        pinpoint.setOffsets(podXOffset, podYOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(xDirection, yDirection);
        // Robot must be stationary: zeroes the pose and recalibrates the Pinpoint's IMU.
        pinpoint.resetPosAndIMU();
    }

    /** Reads fresh data from the Pinpoint. Call once per loop. */
    public void update() {
        pinpoint.update();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public double getX() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getY() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getHeading(AngleUnit unit) {
        return pinpoint.getHeading(unit);
    }

    /** Field-relative velocity along X, in inches per second. */
    public double getVelX() {
        return pinpoint.getVelX(DistanceUnit.INCH);
    }

    /** Field-relative velocity along Y, in inches per second. */
    public double getVelY() {
        return pinpoint.getVelY(DistanceUnit.INCH);
    }

    public void setPose(double x, double y, double heading, AngleUnit unit) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, unit, heading));
    }

    public void setPose(Pose2D pose) {
        pinpoint.setPosition(pose);
    }

    /** Sets the current heading without changing X/Y. */
    public void setHeading(double heading, AngleUnit unit) {
        pinpoint.setHeading(heading, unit);
    }

    /** Zeroes the pose and recalibrates the IMU. The robot must be stationary. */
    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return pinpoint.getDeviceStatus();
    }

    /** The underlying driver, for anything this class doesn't wrap. */
    public GoBildaPinpointDriver getDriver() {
        return pinpoint;
    }
}
