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
import static com.pedropathing.ivy.pedro.PedroCommands.follow;



@Autonomous
@Disabled
public class HamAutoLeaveAndPark extends OpMode {

    private HamRobot robot;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    //*************************************************************
    // Creating the poses
    //*************************************************************
    // poses (x, y, heading/degrees)
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose parkPose = poseFactory.of(84, 24, 0);


    //*************************************************************
    // Creating the paths
    //*************************************************************

    // Drive directly from the start pose to the park pose.
    private Path startToPark() {
        return line(startPose, parkPose).linear(startPose, parkPose);
    }


    //*************************************************************
    // Creating the routine
    //*************************************************************
    // Follow the single path from start to park.
    private Command autoRoutine() {
        return follow(follower, startToPark());
    }

    @Override
    public void init()      // runs when the OpMode is initialized, but before the Start button is pressed.
    {
        // Reset Ivy and create the follower
        Scheduler.reset();
        robot = new HamRobot(hardwareMap);
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

    @Override
    public void stop() {
        robot.stopShooter();
    }
}