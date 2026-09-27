package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Collections;
import java.util.List;

/**
 * Limelight 3A vision. Pipelines are configured on the Limelight through its web UI;
 * this class only selects one by index and reads its results.
 *
 * Call {@link #start()} before reading results and {@link #stop()} when the OpMode ends.
 */
public class LimelightVision {

    /** Robot configuration name of the Limelight. */
    public static final String LIMELIGHT_NAME = "limelight";

    private final Limelight3A limelight;

    public LimelightVision(HardwareMap hardwareMap, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(pipeline);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public boolean switchPipeline(int pipeline) {
        return limelight.pipelineSwitch(pipeline);
    }

    /** The latest result, or null when there is none or it is invalid. */
    public LLResult getResult() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result : null;
    }

    public boolean hasTarget() {
        return getResult() != null;
    }

    /** Horizontal offset to the primary target, in degrees. NaN with no target. */
    public double getTx() {
        LLResult result = getResult();
        return result != null ? result.getTx() : Double.NaN;
    }

    /** Vertical offset to the primary target, in degrees. NaN with no target. */
    public double getTy() {
        LLResult result = getResult();
        return result != null ? result.getTy() : Double.NaN;
    }

    public List<LLResultTypes.FiducialResult> getFiducials() {
        LLResult result = getResult();
        return result != null ? result.getFiducialResults() : Collections.<LLResultTypes.FiducialResult>emptyList();
    }

    /**
     * Field pose from AprilTags using MegaTag2, which needs the robot's heading.
     * The heading comes from odometry/IMU and uses the Limelight field map's convention.
     *
     * @return the pose, or null when no tags are visible.
     */
    public Pose2D getFieldPose(double robotHeading, AngleUnit unit) {
        limelight.updateRobotOrientation(unit.toDegrees(robotHeading));
        LLResult result = getResult();
        if (result == null || result.getBotposeTagCount() == 0) {
            return null;
        }
        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) {
            return null;
        }
        Position position = botpose.getPosition();
        return new Pose2D(position.unit, position.x, position.y,
                AngleUnit.DEGREES, botpose.getOrientation().getYaw(AngleUnit.DEGREES));
    }

    /** The underlying driver, for anything this class doesn't wrap. */
    public Limelight3A getDevice() {
        return limelight;
    }
}
