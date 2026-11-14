package org.firstinspires.ftc.teamcode.ham;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.FourBarOdometry;

@TeleOp(name = "Ham: TeleOp", group = "Ham")
//@Disabled
public class HamTeleOp extends OpMode {

    private HamRobot robot;
    private double maxSpeed = 1.0;
    private boolean leftBumperWasPressed;

    @Override
    public void init() {
        robot = new HamRobot(hardwareMap);

        telemetry.addLine("Ham ready. Options resets field heading.");
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

    @Override
    public void start() {

//        robot.vision.start();
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

        /*
         * Set the intake power variable to equal the right trigger, minus the left trigger.
         * Each trigger outputs a signal from 0-1, with 0 as fully released, and 1 fully depressed.
         * This gives us proportional control of the intake speed. The speed increases as we pull
         * the right trigger further. It's occasionally helpful to be able to reverse the intake,
         * so we also factor in the left trigger. If the left trigger is fully depressed,
         * the intakePower variable will be -1. If the right trigger is fully depressed, the variable
         * will be 1. If the driver pulls both triggers, the intake will remain off.
         * We use this technique (creating a variable, and setting it to our control inputs) to
         * allow us to avoid setting the same motors/servos power more than once per loop. That can
         * create erratic behavior.
         */
        double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        robot.runShooter(gamepad2.right_bumper, intakePower);


        // For this configuration, intake is considered the back
        //robot.drive.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, maxSpeed);

        // For this configuration, intake is considered the front
        robot.drive.driveFieldCentric(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, maxSpeed);

        telemetry.addData("Max speed", "%.2f", maxSpeed);

        telemetry.addData("Heading (deg)", "%.2f", robot.drive.getHeading(AngleUnit.DEGREES));
        FourBarOdometry odometry = robot.drive.getOdometry();
        if (odometry != null) {
            telemetry.addData("Position (in)", "X %.2f  Y %.2f", odometry.getX(), odometry.getY());
        }
//        Pose2D visionPose = robot.vision.getFieldPose(robot.drive.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES);
//        if (visionPose != null) {
//            telemetry.addData("Limelight (in)", "X %.2f  Y %.2f",
//                    visionPose.getX(DistanceUnit.INCH), visionPose.getY(DistanceUnit.INCH));
//        } else {
//            telemetry.addData("Limelight", "no tags");
//        }
    }

    @Override
    public void stop() {
        robot.drive.stop();
        robot.stopShooter();
//        robot.vision.stop();
    }
}
