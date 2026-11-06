package org.firstinspires.ftc.teamcode.teleops;
import static org.firstinspires.ftc.teamcode.MecanumDrive.powerReduction;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp(name = "FieldCentric", group= "Linear Opmode")
public class FirstTeleOp extends LinearOpMode {

    GamepadEx driver;
    MecanumDrive body;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        driver = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        body = new MecanumDrive(hardwareMap);

        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        waitForStart();
        pinpoint.update();
        double startHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        runtime.reset();

        while (opModeIsActive()) {
            driver.readButtons();
            pinpoint.update();
            double heading = pinpoint.getHeading(AngleUnit.RADIANS) - startHeading;
            body.fieldCentric(driver, heading);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}