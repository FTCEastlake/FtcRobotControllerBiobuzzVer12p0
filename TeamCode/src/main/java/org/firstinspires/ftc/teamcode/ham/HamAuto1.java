package org.firstinspires.ftc.teamcode.ham;

import org.firstinspires.ftc.teamcode.ham.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;



@Autonomous
@Disabled
public class HamAuto1 extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    //*************************************************************
    // Creating the poses
    //*************************************************************
    // poses (x, y, heading/degrees)
//    private final Pose startPose = poseFactory.of(24, 24, 0);
//    private final Pose scorePose = poseFactory.of(48, 48, 90);
//    private final Pose parkPose = poseFactory.of(72, 48, 90);
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose scorePose = poseFactory.of(24, 4, 90);
    private final Pose parkPose = poseFactory.of(12, -4, -90);


    //*************************************************************
    // Creating the paths
    //*************************************************************

    // Creating a line path
    // line() creates a straight path from startPose to park.
    // The .linear() part is our Interpolation.
    // It changes the robot's heading from the heading of startPose to the heading of park while it moves along the path.
    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }
    private Path scoreToPark() {
        return line(scorePose, parkPose).linear(scorePose, parkPose);
    }


    // Creating a curved path
    // The first and last Poses are the start and end of the curve.
    // Any Poses between them are control points that change the shape of the curve.
    // You can add multiple control poses in between first and las pose.
//    private final Pose controlPose = p.of(36, 60, 45);
//    private Path curveStartToPark() {
//        return curve(startPose, controlPose, parkPose).linear(startPose, parkPose);
//    }


    //*************************************************************
    // Creating the routine
    //*************************************************************
    // The second path begins after the first follow() command finishes.
    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                // Add shooter mechanism commands here.
                follow(follower, scoreToPark())
        );
    }

    @Override
    public void init()      // runs when the OpMode is initialized, but before the Start button is pressed.
    {
        // Reset Ivy and create the follower
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start()
    {
        // Schedule the routine
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
}