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

public final class Constants {

    private Constants() {
    }

    // Motor names/directions copied from your MecanumDrive class.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("rearLeft");
        c.backRightName.set("rearRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.8933607086421935);
        c.yPodOffset.set(-3.876790024164155);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Starter values: fine for localization/auto-aim. Replace with Foresight
    // AutoTune output before you use Pedro to follow paths in autonomous.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1590274127823455);
                Controller secondaryTranslationalForward = Controller.proportional(0.0587563797323468);
                Controller primaryTranslationalLateral = Controller.proportional(0.20941540583948795);
                Controller secondaryTranslationalLateral = Controller.proportional(0.07737339677498961);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01363330689406105));
                c.brake.set(Controller.proportionalFeedforward(0.011588310859951892));

                c.headingFeedback.set(Controller.proportional(2.0750606153797313));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0442202889145124, 0.0030021938404393286));

                c.linearBrakeCoefficients.set(Matrix.diag(0.050178201636501864, 0.06761673707017683));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013489866362790415, 0.0010137710031664388));

                c.maxAchievableForwardVelocity.set(77.98091924529521);
                c.maxAchievableStrafeVelocity.set(66.314134814283);
                c.naturalForwardDeceleration.set(31.954026140891106);
                c.naturalStrafeDeceleration.set(54.60707909812526);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}