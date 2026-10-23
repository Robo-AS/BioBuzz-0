
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Pinpoint Pose Test")
public class PinpointPoseTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            Pose pose = follower.pose();

            telemetry.addData("X", pose.x());
            telemetry.addData("Y", pose.y());
            telemetry.addData("Heading", pose.heading());
            telemetry.update();
        }
    }
}
