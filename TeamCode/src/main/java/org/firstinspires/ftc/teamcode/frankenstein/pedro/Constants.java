package org.firstinspires.ftc.teamcode.frankenstein.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
            new PinpointLocalizer(h, localizerConfig),
            new Mecanum(h, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }


    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftRear");
        c.backRightName.set("rightRear");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.68381440170168);
        c.yPodOffset.set(-6.585125660333108);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.1755952725167271);
            Controller secondaryTranslationalForward = Controller.proportional(0.06487776120283537);
            Controller primaryTranslationalLateral = Controller.proportional(0.23944956271831608);
            Controller secondaryTranslationalLateral = Controller.proportional(0.08847021521426443);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.01702520023914426));
            c.brake.set(Controller.proportionalFeedforward(0.014471420203272621));

            c.headingFeedback.set(Controller.proportional(2.7961271705977397));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04026424890721667, 0.008643237715838947));

            c.linearBrakeCoefficients.set(Matrix.diag(0.0626483940439343, 0.05455768658123209));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015564876852548447, 0.0019309918204540749));

            c.maxAchievableForwardVelocity.set(60.176941564460876);
            c.maxAchievableStrafeVelocity.set(53.78328715445131);
            c.naturalForwardDeceleration.set(33.42187154743324);
            c.naturalStrafeDeceleration.set(42.2969245262802);
        }
    );

}