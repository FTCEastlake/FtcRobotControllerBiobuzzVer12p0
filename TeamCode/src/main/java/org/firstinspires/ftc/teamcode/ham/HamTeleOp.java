package org.firstinspires.ftc.teamcode.ham;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

    private DcMotorEx launcherMotor = null;
    private DcMotor intakeMotor = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    // Create a variable to set to the intake.
    double intakePower;

    @Override
    public void init() {
        robot = new HamRobot(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        //windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        intakeMotor.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        //windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        //windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

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
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        /*
         * The launch() function handles setting motor velocity, and running the windmill servo
         * to feed the elements into the launcher wheel.
         */
        launch();

        /*
         * Here we set our intake motor and servos to their intake power. The order of operations
         * here is important though. The gamepad triggers define the starting point for the intake
         * power variable in each loop of our code, but inside our launch function we also sometimes
         * change the intake power. So we need to give our launch function a chance to modify the
         * variable before we write it to our motor and servos.
         */
        intakeMotor.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);


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

    void launch() {
        /*
         * Calling gamepad1.right_bumper returns a boolean which will be true if the bumper is
         * held down, and false if it is not. Notably, this will continue to be true for every
         * cycle of our code that the driver holds down that bumper.
         * The first step of our launch() function is checking to see if the user is currently
         * holding down the right gamepad. If they are, then we want to start spinning up the launcher.
         * Otherwise, we start spinning the launcher down.
         */
        if (gamepad1.right_bumper) {
            launcherMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcherMotor.setVelocity(0);
        }

        /*
         * Here we ask if the driver is currently pressing the right bumper, AND the launcher is
         * spinning fast enough to make a successful shot. If it is, then we will turn on the
         * windmill servo to start feeding the elements into the launcher motor. We also
         * add some power to the intake power. This can sometimes help dislodge stuck elements from
         * inside the hopper.
         */
        if (gamepad1.right_bumper && launcherMotor.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            //windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            //windmillServo.setPower(0);
        }
    }

    @Override
    public void stop() {
        robot.drive.stop();
//        robot.vision.stop();
    }
}
