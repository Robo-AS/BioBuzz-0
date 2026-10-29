
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pinpoint Pose Test")
public class PinpointPoseTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            telemetry.addData("X (in)", pinpoint.getPosX(DistanceUnit.CM));
            telemetry.addData("Y (in)", pinpoint.getPosY(DistanceUnit.CM));
            telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Encoder X", pinpoint.getEncoderX());
            telemetry.addData("Encoder Y", pinpoint.getEncoderY());

            telemetry.update();
        }
    }
}
