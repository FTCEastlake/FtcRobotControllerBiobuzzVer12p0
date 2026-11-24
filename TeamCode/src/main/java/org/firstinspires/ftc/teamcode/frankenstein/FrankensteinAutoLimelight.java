package org.firstinspires.ftc.teamcode.frankenstein;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.frankenstein.FrankensteinRobot;
import org.firstinspires.ftc.teamcode.frankenstein.pedro.Constants;

@Autonomous
@Disabled
public class FrankensteinAutoLimelight extends OpMode {

    private FrankensteinRobot robot;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private boolean tagAligned;

    //*************************************************************
    // Creating the poses
    //*************************************************************
    // poses (x, y, heading/degrees)
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose shootPose3 = poseFactory.of(0, -15, -270);
    private final Pose parkPose = poseFactory.of(0, 0, 0);


    //*************************************************************
    // Creating the paths
    //*************************************************************

    // Drive directly from the start pose to the AprilTag alignment position.
    private Path startToShoot3() { return line(startPose, shootPose3).linear(startPose, shootPose3); }
    private Path shoot3ToPark()   { return line(shootPose3, parkPose).linear(shootPose3, parkPose); }


    //*************************************************************
    // Creating the routine
    //*************************************************************
    // Drive to shootPose3, wait, align to the AprilTag, then stop.
    private Command autoRoutine() {
        return sequential(

            // Drive to the shootPose3 position, then stop.
            follow(follower, startToShoot3()),
            instant(() -> follower.stop()),

            // Pause for 1 second. This can be commented out if you want to 
            // start aligning to the AprilTag immediately after driving to shootPose3.
            Command.build()
                .setStart(shooterTimer::reset)
                .setDone(() -> shooterTimer.seconds() >= 1.0),
            
            // Align to the AprilTag.
            Command.build()
                .setStart(() -> tagAligned = false)
                .setExecute(() -> {
                    Pose3D tagPose = robot.tagAligner.alignToTag(0.0, 65.0, 0.0, telemetry);

                    // Once the tag is coarsely aligned, reset the timer to allow for another
                    // 2 seconds of finely tuned alignment before the routine is considered done.
                    if (!tagAligned && isTagAligned(tagPose)) {
                        tagAligned = true;
                        shooterTimer.reset();
                    }
                })
                .setDone(() -> tagAligned && shooterTimer.seconds() >= 2.0),
            instant(() -> robot.drive.stop()),

            // Simulate shooting for 5 seconds.
            Command.build()
                .setStart(shooterTimer::reset)
                .setDone(() -> shooterTimer.seconds() >= 5.0),

            // Drive to the park position, then hold the pose.
            follow(follower, shoot3ToPark()),
            instant(() -> follower.hold(parkPose))
        );
    }

    private boolean isTagAligned(Pose3D tagPose) {
        if (tagPose == null) {
            return false;
        }

        Position position = tagPose.getPosition().toUnit(DistanceUnit.INCH);
        double yaw = normalizeDegrees(tagPose.getOrientation().getYaw(AngleUnit.DEGREES));
        return Math.abs(position.x) <= 0.50
                && Math.abs(position.z - 65.0) <= 0.50
                && Math.abs(yaw) <= 0.5;
    }

    private double normalizeDegrees(double angleDegrees) {
        return Math.atan2(Math.sin(Math.toRadians(angleDegrees)),
                Math.cos(Math.toRadians(angleDegrees))) * 180.0 / Math.PI;
    }

    @Override
    public void init()      // runs when the OpMode is initialized, but before the Start button is pressed.
    {
        // Reset Ivy and create the follower
        Scheduler.reset();
        robot = new FrankensteinRobot(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start()
    {
        // Schedule the routine
        if (robot.vision != null) {
            robot.vision.start();
        }
        schedule(autoRoutine());
    }

    @Override
    public void loop()
    {
        follower.update();
        Scheduler.execute();
        // add your other methods needed in the loop here

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.drive.stop();
        if (robot.vision != null) {
            robot.vision.stop();
        }
    }

}
