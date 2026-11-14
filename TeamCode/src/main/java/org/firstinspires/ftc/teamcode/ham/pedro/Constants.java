package org.firstinspires.ftc.teamcode.ham.pedro;

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
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

        // Add this to enable manual break mode.
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.7522844930333425);
        c.yPodOffset.set(-2.02434810127799);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.18025348160185836);
                Controller secondaryTranslationalForward = Controller.proportional(0.06659884499015221);
                Controller primaryTranslationalLateral = Controller.proportional(0.27208845810850474);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10052941493359212);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01670053391822993));
                c.brake.set(Controller.proportionalFeedforward(0.01419545383049544));

                c.headingFeedback.set(Controller.proportional(2.9584087501721665));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.036137720328026854, 0.007509569806130337));

                c.linearBrakeCoefficients.set(Matrix.diag(0.03947076088189521, 0.06369599991316256));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002147946312652514, 0.0015515506785317072));

                c.maxAchievableForwardVelocity.set(62.49040666943081);
                c.maxAchievableStrafeVelocity.set(54.84700103958579);
                c.naturalForwardDeceleration.set(38.27130251003785);
                c.naturalStrafeDeceleration.set(49.38927066538115);
            }
    );

}
