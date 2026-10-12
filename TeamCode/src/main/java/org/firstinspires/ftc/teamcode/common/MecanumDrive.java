package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-centric 4-wheel mecanum drive (goBILDA Yellow Jacket motors, 104 mm GripForce wheels).
 *
 * Heading for field-centric control comes from one of two sources, chosen by constructor:
 * <ul>
 *   <li>Pinpoint 4-bar odometry (default): also tracks X/Y position, see {@link #getOdometry()}.</li>
 *   <li>REV Control Hub IMU (optional): heading only, no position tracking.</li>
 * </ul>
 *
 * Call {@link #update()} once per loop, then {@link #driveFieldCentric}.
 */
public class MecanumDrive {

    public static final String LEFT_FRONT = "leftFront";
    public static final String LEFT_REAR = "leftRear";
    public static final String RIGHT_FRONT = "rightFront";
    public static final String RIGHT_REAR = "rightRear";
    public static final String IMU_NAME = "imu";

    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;

    private final FourBarOdometry odometry; // null when using the Control Hub IMU
    private final IMU imu;                  // null when using Pinpoint odometry

    private double maxSpeed;

    /**
     * Default: heading and position from Pinpoint 4-bar odometry. Distances are in inches.
    * Motor directions are supplied explicitly so they can be configured per robot.
     *
     * @param xOffset    sideways offset of the X (forward) pod from the tracking center, left positive.
     * @param yOffset    forward offset of the Y (strafe) pod from the tracking center, forward positive.
     * @param xEncoder   X pod direction; should count up when the robot moves forward.
     * @param yEncoder   Y pod direction; should count up when the robot moves left.
    * @param leftFrontDirection direction of the left-front motor.
    * @param leftRearDirection  direction of the left-rear motor.
    * @param rightFrontDirection direction of the right-front motor.
    * @param rightRearDirection direction of the right-rear motor.
     * @param maxSpeed   motor power cap, 0 to 1.
     */
    public MecanumDrive(HardwareMap hardwareMap,
                        double xOffset, double yOffset,
                        GoBildaPinpointDriver.EncoderDirection xEncoder,
                        GoBildaPinpointDriver.EncoderDirection yEncoder,
                        DcMotorSimple.Direction leftFrontDirection,
                        DcMotorSimple.Direction leftRearDirection,
                        DcMotorSimple.Direction rightFrontDirection,
                        DcMotorSimple.Direction rightRearDirection,
                        double maxSpeed) {
        leftFront = initMotor(hardwareMap, LEFT_FRONT, leftFrontDirection);
        leftRear = initMotor(hardwareMap, LEFT_REAR, leftRearDirection);
        rightFront = initMotor(hardwareMap, RIGHT_FRONT, rightFrontDirection);
        rightRear = initMotor(hardwareMap, RIGHT_REAR, rightRearDirection);
        odometry = new FourBarOdometry(hardwareMap, xOffset, yOffset, xEncoder, yEncoder);
        imu = null;
        setMaxSpeed(maxSpeed);
    }

    /**
     * Optional: heading from the REV Control Hub's built-in IMU. No position tracking.
     *
     * @param logoDirection direction the REV logo on the hub faces.
     * @param usbDirection  direction the hub's USB ports face.
     * @param maxSpeed      motor power cap, 0 to 1.
     */
    public MecanumDrive(HardwareMap hardwareMap,
                        RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
                        RevHubOrientationOnRobot.UsbFacingDirection usbDirection,
                        DcMotorSimple.Direction leftFrontDirection,
                        DcMotorSimple.Direction leftRearDirection,
                        DcMotorSimple.Direction rightFrontDirection,
                        DcMotorSimple.Direction rightRearDirection,
                        double maxSpeed) {
        leftFront = initMotor(hardwareMap, LEFT_FRONT, leftFrontDirection);
        leftRear = initMotor(hardwareMap, LEFT_REAR, leftRearDirection);
        rightFront = initMotor(hardwareMap, RIGHT_FRONT, rightFrontDirection);
        rightRear = initMotor(hardwareMap, RIGHT_REAR, rightRearDirection);
        odometry = null;
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();
        setMaxSpeed(maxSpeed);
    }

    private static DcMotorEx initMotor(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void setMotorDirections(DcMotorSimple.Direction leftFrontDirection,
                                   DcMotorSimple.Direction leftRearDirection,
                                   DcMotorSimple.Direction rightFrontDirection,
                                   DcMotorSimple.Direction rightRearDirection) {
        leftFront.setDirection(leftFrontDirection);
        leftRear.setDirection(leftRearDirection);
        rightFront.setDirection(rightFrontDirection);
        rightRear.setDirection(rightRearDirection);
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = Range.clip(maxSpeed, 0, 1);
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    /** Refreshes the odometry reading. Call once per loop. No-op in IMU mode. */
    public void update() {
        if (odometry != null) {
            odometry.update();
        }
    }

    /** Robot heading, counter-clockwise positive. */
    public double getHeading(AngleUnit unit) {
        if (odometry != null) {
            return odometry.getHeading(unit);
        }
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }

    /** Makes the robot's current facing the field's "forward" for field-centric driving. */
    public void resetHeading() {
        if (odometry != null) {
            odometry.setHeading(0, AngleUnit.RADIANS);
        } else {
            imu.resetYaw();
        }
    }

    /** The Pinpoint odometry, or null when using the Control Hub IMU. */
    public FourBarOdometry getOdometry() {
        return odometry;
    }

    /**
     * Drives relative to the field. Typical gamepad mapping:
     * {@code driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)}.
     *
     * @param forward away from the driver, -1 to 1.
     * @param strafe  to the driver's right, -1 to 1.
     * @param turn    clockwise, -1 to 1.
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        // Rotate the field-frame command into the robot frame. Heading is CCW positive and
        // strafe is right positive, so rotating by +heading here is the inverse rotation.
        double heading = getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double robotForward = forward * cos - strafe * sin;
        double robotStrafe = forward * sin + strafe * cos;
        driveRobotCentric(robotForward, robotStrafe, turn);
    }

    /**
     * Drives relative to the robot.
     *
     * @param forward robot forward, -1 to 1.
     * @param strafe  robot right, -1 to 1.
     * @param turn    clockwise, -1 to 1.
     */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        double lf = forward + strafe + turn;
        double lr = forward - strafe + turn;
        double rf = forward - strafe - turn;
        double rr = forward + strafe - turn;

        // Keep the wheel power ratios when any value exceeds 1.
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lr)),
                                            Math.max(Math.abs(rf), Math.abs(rr))));
        double scale = maxSpeed / max;

        leftFront.setPower(lf * scale);
        leftRear.setPower(lr * scale);
        rightFront.setPower(rf * scale);
        rightRear.setPower(rr * scale);
    }

    /**
     * Runs individual wheels at low power for hardware checks. Call once per loop
     * while testing; a wheel stops as soon as its button is released.
     * X: left front, Y: right front, A: left rear, B: right rear.
     */
    public void testIndividualMotors(Gamepad gamepad) {
        leftFront.setPower(gamepad.x ? 0.15 : 0);
        rightFront.setPower(gamepad.y ? 0.15 : 0);
        leftRear.setPower(gamepad.a ? 0.15 : 0);
        rightRear.setPower(gamepad.b ? 0.15 : 0);
    }

    public void stop() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
