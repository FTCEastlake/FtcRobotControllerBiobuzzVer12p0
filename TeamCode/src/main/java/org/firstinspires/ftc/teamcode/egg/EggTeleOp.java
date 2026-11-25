package org.firstinspires.ftc.teamcode.egg;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.FourBarOdometry;

@TeleOp(name = "Egg: TeleOp", group = "Egg")
@Disabled
public class EggTeleOp extends OpMode {

    private EggRobot robot;
    private EggIntake intake;
    private EggFlyWheel flywheel;

    private double maxSpeed = 1.0;
    private boolean leftBumperWasPressed;


    @Override
    public void init() {
        robot = new EggRobot(hardwareMap);
        intake = new EggIntake(hardwareMap);
        flywheel = new EggFlyWheel(hardwareMap);

        telemetry.addLine("Egg ready. Options resets field heading.");
    }

    // Called repeatedly after you press the Init button, but before you press the Play button on the Driver Station.
    @Override
    public void init_loop() {
        robot.drive.testIndividualMotors(gamepad1);
    }

    // Called one time as soon as you press the Play button on the Driver Station.
    @Override
    public void start() {
        // robot.vision.start();
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

        robot.drive.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, maxSpeed);
        telemetry.addData("Max speed", "%.2f", maxSpeed);

        telemetry.addData("Heading (deg)", "%.2f", robot.drive.getHeading(AngleUnit.DEGREES));
        FourBarOdometry odometry = robot.drive.getOdometry();
        if (odometry != null) {
            telemetry.addData("Position (in)", "X %.2f  Y %.2f", odometry.getX(), odometry.getY());
        }

        if (gamepad2.right_bumper) {
            intake.setOn();
        } else if (gamepad2.x) {
            intake.reverse();
        } else {
            intake.setOff();
        }

        if (gamepad2.left_bumper) {
            flywheel.setOn();
        } else {
            flywheel.setOff();
        }

        telemetry.addData("flywheel speed RPM: ", flywheel.getSpeed());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.drive.stop();
        intake.setOff();
        flywheel.setOff();
    }
}
