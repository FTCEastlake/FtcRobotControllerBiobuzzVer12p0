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
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(
        name = "Ham: Auto V3a - Short Park 12in",
        group = "Ham"
)
@Disabled
public class HamAutoV3aShortPark12 extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private static final double PARK_DISTANCE_INCHES = -12.0;
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose parkPose = poseFactory.of(PARK_DISTANCE_INCHES, 0, 0);

    private Path startToPark() {
        return line(startPose, parkPose).linear(startPose, parkPose);
    }

    private Command autoRoutine() {
        return follow(follower, startToPark());
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
        telemetry.addLine("Ham V3a 12-inch short park ready");
        telemetry.addData("Distance", "%.1f in", PARK_DISTANCE_INCHES);
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
