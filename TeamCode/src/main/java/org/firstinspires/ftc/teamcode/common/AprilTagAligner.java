package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Drives a mecanum robot to a desired camera-relative AprilTag position and orientation.
 * Call {@link #alignToTag(double, double, double, Telemetry)} repeatedly while alignment is held.
 */
public class AprilTagAligner {

    private double ALIGN_TOLERANCE_IN;
    private double ALIGN_YAW_TOLERANCE_DEG;
    private double ALIGN_POWER_PER_INCH;
    private double ALIGN_TURN_POWER_PER_DEGREE;
    private double ALIGN_MAX_POWER;
    private double ALIGN_MAX_TURN_POWER;

    private final MecanumDrive drive;
    private final LimelightVision vision;

    public AprilTagAligner(MecanumDrive drive, LimelightVision vision) {
        this.drive = drive;
        this.vision = vision;
    }

    /** Configures alignment tolerances, proportional gains, and maximum drive powers. */
    public void Init(double alignToleranceIn, double alignYawToleranceDeg,
                     double alignPowerPerInch, double alignTurnPowerPerDegree,
                     double alignMaxPower, double alignMaxTurnPower) {
        ALIGN_TOLERANCE_IN = alignToleranceIn;
        ALIGN_YAW_TOLERANCE_DEG = alignYawToleranceDeg;
        ALIGN_POWER_PER_INCH = alignPowerPerInch;
        ALIGN_TURN_POWER_PER_DEGREE = alignTurnPowerPerDegree;
        ALIGN_MAX_POWER = alignMaxPower;
        ALIGN_MAX_TURN_POWER = alignMaxTurnPower;
    }

    /**
     * Moves to the configured camera-relative tag pose and rotates to face it straight on.
     * Finds the first visible tag, aligns to it, and returns its pose for telemetry/use by callers.
     * Stops the robot when the pose is unavailable.
     */
    public Pose3D alignToTag(double targetXIn, double targetZIn,
                             double targetYawDeg, Telemetry telemetry) {
        Pose3D tagPose = getFirstTagPose();
        if (tagPose == null) {
            drive.stop();
            telemetry.addData("Tag alignment", "No tag pose - stopped");
            return null;
        }

        Position tagPosition = tagPose.getPosition().toUnit(DistanceUnit.INCH);
        double xError = tagPosition.x - targetXIn;
        double zError = tagPosition.z - targetZIn;
        double yawError = normalizeDegrees(
            tagPose.getOrientation().getYaw(AngleUnit.DEGREES) - targetYawDeg);

        double strafeRight = Math.abs(xError) <= ALIGN_TOLERANCE_IN
                ? 0.0 : clipAlignmentPower(xError);
        double forward = Math.abs(zError) <= ALIGN_TOLERANCE_IN
                ? 0.0 : clipAlignmentPower(zError);
        // MecanumDrive's turn input is clockwise-positive; counter-steer against observed tag yaw.
        double turnClockwise = Math.abs(yawError) <= ALIGN_YAW_TOLERANCE_DEG
                ? 0.0 : clipTurnPower(-yawError);

        // Commands are already limited below, so avoid scaling them a second time in the drive.
        drive.setMaxSpeed(1.0);
        drive.driveRobotCentric(forward, strafeRight, turnClockwise);

        boolean aligned = Math.abs(xError) <= ALIGN_TOLERANCE_IN
                && Math.abs(zError) <= ALIGN_TOLERANCE_IN
                && Math.abs(yawError) <= ALIGN_YAW_TOLERANCE_DEG;
        telemetry.addData("Tag alignment", aligned ? "Aligned" : "Aligning (hold A)");
        telemetry.addData("Alignment error", "X %.2f in  Z %.2f in  Yaw %.1f deg",
                xError, zError, yawError);
        return tagPose;
    }

    /** Returns the first visible tag's camera-space pose, or null when none is available. */
    public Pose3D getFirstTagPose() {
        if (vision == null) {
            return null;
        }
        List<LLResultTypes.FiducialResult> fiducials = vision.getFiducials();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            Pose3D pose = fiducial.getTargetPoseCameraSpace();
            if (pose != null) {
                return pose;
            }
        }
        return null;
    }

    private double clipAlignmentPower(double errorInches) {
        double power = errorInches * ALIGN_POWER_PER_INCH;
        return Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, power));
    }

    private double clipTurnPower(double yawErrorDegrees) {
        double power = yawErrorDegrees * ALIGN_TURN_POWER_PER_DEGREE;
        return Math.max(-ALIGN_MAX_TURN_POWER, Math.min(ALIGN_MAX_TURN_POWER, power));
    }

    private double normalizeDegrees(double angleDegrees) {
        return Math.atan2(Math.sin(Math.toRadians(angleDegrees)),
                Math.cos(Math.toRadians(angleDegrees))) * 180.0 / Math.PI;
    }
}