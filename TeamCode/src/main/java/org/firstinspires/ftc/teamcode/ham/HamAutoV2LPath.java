package org.firstinspires.ftc.teamcode.ham;

import org.firstinspires.ftc.teamcode.ham.pedro.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(
        name = "Ham: Auto V2 - L Path",
        group = "Ham"
)
@Disabled
public class HamAutoV2LPath extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose cornerPose = poseFactory.of(-84, 0, 0);
    private final Pose parkPose = poseFactory.of(-84, -24, 0);

    private Path startToCorner() {
        return line(startPose, cornerPose).linear(startPose, cornerPose);
    }

    private Path cornerToPark() {
        return line(cornerPose, parkPose).linear(cornerPose, parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToCorner()),
                follow(follower, cornerToPark())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
        telemetry.addLine("Ham V2 L-path ready");
        telemetry.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        telemetry.addData("X", "%.2f in", follower.pose().x());
        telemetry.addData("Y", "%.2f in", follower.pose().y());
        telemetry.addData("Heading", "%.2f deg", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
    }
}
