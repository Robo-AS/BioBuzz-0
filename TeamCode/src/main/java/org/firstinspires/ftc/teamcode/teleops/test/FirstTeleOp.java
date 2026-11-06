package org.firstinspires.ftc.teamcode.teleops.test;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/** Test tool: field-centric POWER drive using the raw Pinpoint heading (no Pedro). */
@TeleOp(name = "FieldCentric", group = "Linear Opmode")
public class FirstTeleOp extends RobotOpMode {

    private GamepadEx driver;
    private MecanumDrive body;
    private GoBildaPinpointDriver pinpoint;
    private final ElapsedTime runtime = new ElapsedTime();
    private double startHeading;

    @Override
    protected void initRobot() {
        driver = new GamepadEx(gamepad1);
        body = new MecanumDrive(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void onStart() {
        pinpoint.update();
        startHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        runtime.reset();

        schedule(new RunCommand(() -> {
            pinpoint.update();
            double heading = pinpoint.getHeading(AngleUnit.RADIANS) - startHeading;
            body.fieldCentric(driver, heading);
        }));
    }

    @Override
    protected void addTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }
}
