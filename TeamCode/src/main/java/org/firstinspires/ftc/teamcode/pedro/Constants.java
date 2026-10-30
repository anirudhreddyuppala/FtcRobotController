package org.firstinspires.ftc.teamcode.pedro;

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


    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("left_front_drive");
        c.frontRightName.set("right_front_drive");
        c.backLeftName.set("left_back_drive");
        c.backRightName.set("right_back_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.130518515278974);
        c.yPodOffset.set(-7.656391774575542);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.19028971946819515);
                Controller secondaryTranslationalForward = Controller.proportional(0.07030696670854886);
                Controller primaryTranslationalLateral = Controller.proportional(0.24939443543725884);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09214457995202067);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01672050457818686));
                c.brake.set(Controller.proportionalFeedforward(0.014212428891458831));

                c.headingFeedback.set(Controller.proportional(2.547802485737746));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04124466005064541, 0.006002016834463993));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07448102021771424, 0.04818915875466976));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011272281872064028, 0.0014829440149361764));

                c.maxAchievableForwardVelocity.set(64.33952202470527);
                c.maxAchievableStrafeVelocity.set(51.89383557554082);
                c.naturalForwardDeceleration.set(28.08051414011434);
                c.naturalStrafeDeceleration.set(55.766591315905075);
            }
    );

    /*public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.18784501985743288);
                Controller secondaryTranslationalForward = Controller.proportional(0.06940371552594887);
                Controller primaryTranslationalLateral = Controller.proportional(0.23138278748407856);
                Controller secondaryTranslationalLateral = Controller.proportional(0.08548975731341774);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01567628934459508));
                c.brake.set(Controller.proportionalFeedforward(0.013324845942905818));

                c.headingFeedback.set(Controller.proportional(2.64084223832534));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04317878642244903, 0.005664717185761555));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07993714042165997, 0.07605528531697134));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.29100560753446E-4, 8.771652875536773E-4));

                c.maxAchievableForwardVelocity.set(64.95598329317632);
                c.maxAchievableStrafeVelocity.set(54.14145161880232);
                c.naturalForwardDeceleration.set(28.94900032921723);
                c.naturalStrafeDeceleration.set(60.14001064264417);
            }
    );*/
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h,drivetrainConfig),
                new Foresight(foresightConfig)
        );

    }
}