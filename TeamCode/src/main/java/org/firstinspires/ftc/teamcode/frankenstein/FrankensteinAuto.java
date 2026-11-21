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

import org.firstinspires.ftc.teamcode.frankenstein.FrankensteinRobot;
import org.firstinspires.ftc.teamcode.frankenstein.pedro.Constants;

@Autonomous
@Disabled
public class FrankensteinAuto extends OpMode {

    private FrankensteinRobot robot;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final ElapsedTime shooterTimer = new ElapsedTime();

    //*************************************************************
    // Creating the poses
    //*************************************************************
    // poses (x, y, heading/degrees)
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose shootPose1 = poseFactory.of(40, 0, -90);
    private final Pose shootPose2 = poseFactory.of(40, -15, -180);
    private final Pose shootPose3 = poseFactory.of(0, -15, -270);
    private final Pose parkPose = poseFactory.of(0, 0, 0);


    //*************************************************************
    // Creating the paths
    //*************************************************************

    // Drive directly from the start pose to the park pose.
    private Path startToShoot1()  { return line(startPose, shootPose1).linear(startPose, shootPose1); }
    private Path Shoot1ToShoot2() { return line(shootPose1, shootPose2).linear(shootPose1, shootPose2); }
    private Path Shoot2ToShoot3() { return line(shootPose2, shootPose3).linear(shootPose2, shootPose3); }
    private Path shoot3ToPark()   { return line(shootPose3, parkPose).linear(shootPose3, parkPose); }


    //*************************************************************
    // Creating the routine
    //*************************************************************
    // Follow the single path from start to park.
    private Command autoRoutine() {
        return sequential(

                follow(follower, startToShoot1()),
                Command.build()
                    .setStart(shooterTimer::reset)                      // start timer
                    //.setExecute(() -> robot.runShooter(true, 1.0))    // add in the code you want to execute
                    .setDone(() -> shooterTimer.seconds() >= 3.0),      // stop timer after 3 seconds

                follow(follower, Shoot1ToShoot2()),
                Command.build()
                    .setStart(shooterTimer::reset)                      // start timer
                    //.setExecute(() -> robot.runShooter(true, 1.0))    // add in the code you want to execute
                    .setDone(() -> shooterTimer.seconds() >= 3.0),      // stop timer after 3 seconds

                follow(follower, Shoot2ToShoot3()),
                Command.build()
                    .setStart(shooterTimer::reset)                      // start timer
                    //.setExecute(() -> robot.runShooter(true, 1.0))    // add in the code you want to execute
                    .setDone(() -> shooterTimer.seconds() >= 3.0),      // stop timer after 3 seconds

                follow(follower, shoot3ToPark())
        );
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
        //robot.stopShooter();
    }

}
