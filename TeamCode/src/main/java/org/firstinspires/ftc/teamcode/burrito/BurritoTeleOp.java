package org.firstinspires.ftc.teamcode.burrito;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.FourBarOdometry;

@TeleOp(name = "Burrito: TeleOp", group = "Burrito")
@Disabled
public class BurritoTeleOp extends OpMode {

    private BurritoRobot robot;

    @Override
    public void init() {
        robot = new BurritoRobot(hardwareMap);
        telemetry.addLine("Burrito ready. Options resets field heading.");
    }

    // Called repeatedly after you press the Init button, but before you press the Play button on the Driver Station.
    @Override
    public void init_loop() {

        // We're using this loop for testing individual motors.  
        // The driver can press X/Y/A/B (LF/RF/LR/RR) to run each motor at 15% power.
        // All of the wheels should be spinning forward when the X/Y/A/B buttons are pressed.  
        // If any of the wheels are spinning backwards, then the motor direction for that 
        // wheel needs to be reversed in BurritoRobot.java.
        robot.drive.testIndividualMotors(gamepad1);

        // Add other motor/servo tests here if needed, e.g.:
        // robot.arm.testArmMotors(gamepad2);
    }

    // Called one time as soon as you press the Play button on the Driver Station.
    @Override
    public void start() {
        robot.vision.start();
    }

    // Called repeatedly after you press the Play button on the Driver Station.
    @Override
    public void loop() {
        robot.drive.update();

        if (gamepad1.start) {
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
