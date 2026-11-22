package org.firstinspires.ftc.teamcode.frankenstein;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.FourBarOdometry;

@TeleOp(name = "Frankenstein: TeleOp", group = "Frankenstein")
//@Disabled
public class FrankensteinTeleOp extends OpMode {

    // Desired camera-relative tag pose: horizontal offset, forward range, and face-on yaw.
    private static final double TAG_TARGET_X_IN = 0.0;
    private static final double TAG_TARGET_Z_IN = 52.0;
    private static final double TAG_TARGET_YAW_DEG = 0.0;

    private FrankensteinRobot robot;
    private double maxSpeed = 1.0;
    private boolean leftBumperWasPressed;

    // Called one time as soon as you press the Init button on the Driver Station. 
    // It is used for basic setup, such as initializing hardware maps and setting default variable states.
    @Override
    public void init() {
        robot = new FrankensteinRobot(hardwareMap);
        telemetry.addLine("Frankenstein ready. Options resets field heading.");
    }

    // Called repeatedly after you press the Init button, but before you press the Play button on the Driver Station.
    @Override
    public void init_loop() {

        // We're using this loop for testing individual motors.  
        // The driver can press X/Y/A/B (LF/RF/LR/RR) to run each motor at 15% power.
        // All of the wheels should be spinning forward when the X/Y/A/B buttons are pressed.  
        // If any of the wheels are spinning backwards, then the motor direction for that 
        // wheel needs to be reversed in FrankensteinRobot.java.
        robot.drive.testIndividualMotors(gamepad1);

        // Add other motor/servo tests here if needed, e.g.:
        // robot.arm.testArmMotors(gamepad2);
    }

    // Called one time as soon as you press the Play button on the Driver Station.
    @Override
    public void start() {
        if (robot.vision != null) {
            robot.vision.start();
        }
    }

    // Called repeatedly after you press the Play button on the Driver Station.
    @Override
    public void loop() {
        robot.drive.update();

        // Toggle the drive speed once per left-bumper press.
        boolean leftBumperPressed = gamepad1.left_bumper;
        if (leftBumperPressed && !leftBumperWasPressed) {
            maxSpeed = maxSpeed == 1.0 ? 0.5 : maxSpeed == 0.5 ? 0.25 : 1.0;
        }
        leftBumperWasPressed = leftBumperPressed;

        if (gamepad1.start) {
            robot.drive.resetHeading();
        }

        //Pose3D tagPose = null;
        if (gamepad1.a && (robot.vision != null)) {
            robot.tagAligner.alignToTag(TAG_TARGET_X_IN, TAG_TARGET_Z_IN, TAG_TARGET_YAW_DEG, telemetry);
        } else {
            robot.drive.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, maxSpeed);
        }

        telemetry.addData("Max speed", "%.2f", maxSpeed);
        telemetry.addData("Heading (deg)", "%.2f", robot.drive.getHeading(AngleUnit.DEGREES));
        FourBarOdometry odometry = robot.drive.getOdometry();
        if (odometry != null) {
            telemetry.addData("Position (in)", "X %.2f  Y %.2f", odometry.getX(), odometry.getY());
        }

        if (robot.vision == null) {
            telemetry.addData("Tag alignment", "Limelight is disabled");
        }
        else {
            Pose3D tagPose = robot.tagAligner.getFirstTagPose();
            Position tagPosition = tagPose == null ? null : tagPose.getPosition().toUnit(DistanceUnit.INCH);
            if (tagPosition != null) {
                telemetry.addData("AprilTag position from camera (in)",
                        "X %.2f  Y %.2f  Z %.2f", tagPosition.x, tagPosition.y, tagPosition.z);
                telemetry.addData("AprilTag yaw from camera (deg)", "%.1f",
                        tagPose.getOrientation().getYaw(AngleUnit.DEGREES));
            } else {
                telemetry.addData("AprilTag position from camera", "no tag pose");
            }
        }

    }

    @Override
    public void stop() {
        robot.drive.stop();
        if (robot.vision != null) {
            robot.vision.stop();
        }
    }
}
