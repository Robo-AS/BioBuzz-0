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
    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.backLeftName.set("rearLeft");
        c.frontRightName.set("frontRight");
        c.backRightName.set("rearRight");

        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

        c.manualBrakeMode.set(true);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint"); // must match the name in your Robot Configuration
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // TODO: measure your pod offsets (inches) or run the Pinpoint AutoTune.
        c.xPodOffset.set(-0.3257888508593942);
        c.yPodOffset.set(-0.14364017276313362);

        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Starter values: fine for localization/auto-aim. Replace with Foresight
    // AutoTune output before you use Pedro to follow paths in autonomous.
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryForward = Controller.proportional(0.30);
        Controller secondaryForward = Controller.proportional(0.10);
        Controller primaryLateral = Controller.proportional(0.30);
        Controller secondaryLateral = Controller.proportional(0.10);

        c.forwardTranslational.set(
                Controller.piecewise(secondaryForward).put(2.5, primaryForward));
        c.strafeTranslational.set(
                Controller.piecewise(secondaryLateral).put(2.5, primaryLateral));

        c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
        c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
        c.headingFeedback.set(Controller.proportional(5.258721785960744));
        c.headingBrakeCoefficients.set(
                Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));

        c.linearBrakeCoefficients.set(
                Matrix.diag(0.10605894992901523, 0.08719146175596092));
        c.quadraticBrakeCoefficients.set(
                Matrix.diag(0.0014663966976606565, 0.0013837064502458813));

        c.maxAchievableForwardVelocity.set(72.72923108818539);
        c.maxAchievableStrafeVelocity.set(52.34323936525474);
        c.naturalForwardDeceleration.set(85.01144677379789);
        c.naturalStrafeDeceleration.set(104.49787535782846);
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}