package org.firstinspires.ftc.teamcode.frankenstein;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.FourBarOdometry;

@TeleOp(name = "Frankenstein: TeleOp", group = "Frankenstein")
//@Disable
public class FrankensteinTeleOp extends OpMode {

    private FrankensteinRobot robot;

    @Override
    public void init() {
        robot = new FrankensteinRobot(hardwareMap);
        telemetry.addLine("Frankenstein ready. Options resets field heading.");
    }

    @Override
    public void start() {
        robot.vision.start();
    }

    @Override
    public void loop() {
        robot.drive.update();

        if (gamepad1.options) {
            robot.drive.resetHeading();
        }

        robot.drive.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        telemetry.addData("Heading (deg)", "%.1f", robot.drive.getHeading(AngleUnit.DEGREES));
        FourBarOdometry odometry = robot.drive.getOdometry();
        if (odometry != null) {
            telemetry.addData("Position (in)", "X %.1f  Y %.1f", odometry.getX(), odometry.getY());
        }
        Pose2D visionPose = robot.vision.getFieldPose(robot.drive.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES);
        if (visionPose != null) {
            telemetry.addData("Limelight (in)", "X %.1f  Y %.1f",
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
