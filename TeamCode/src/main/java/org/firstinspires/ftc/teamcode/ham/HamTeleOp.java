package org.firstinspires.ftc.teamcode.ham;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.FourBarOdometry;

@TeleOp(name = "Ham: TeleOp", group = "Ham")
@Disabled
public class HamTeleOp extends OpMode {

    private HamRobot robot;
    private double maxSpeed = 1.0;
    private boolean leftBumperWasPressed;

    @Override
    public void init() {
        robot = new HamRobot(hardwareMap);
        telemetry.addLine("Ham ready. Options resets field heading.");
    }

    @Override
    public void start() {
        robot.vision.start();
    }

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

        robot.drive.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, maxSpeed);
        telemetry.addData("Max speed", "%.2f", maxSpeed);

        telemetry.addData("Heading (deg)", "%.2f", robot.drive.getHeading(AngleUnit.DEGREES));
        FourBarOdometry odometry = robot.drive.getOdometry();
        if (odometry != null) {
            telemetry.addData("Position (in)", "X %.2f  Y %.2f", odometry.getX(), odometry.getY());
        }
        Pose2D visionPose = robot.vision.getFieldPose(robot.drive.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES);
        if (visionPose != null) {
            telemetry.addData("Limelight (in)", "X %.2f  Y %.2f",
                    visionPose.getX(DistanceUnit.INCH), visionPose.getY(DistanceUnit.INCH));
        } else {
            telemetry.addData("Limelight", "no tags");
        }
    }

    @Override
    public void stop() {
        robot.drive.stop();
        robot.vision.stop();
    }
}
