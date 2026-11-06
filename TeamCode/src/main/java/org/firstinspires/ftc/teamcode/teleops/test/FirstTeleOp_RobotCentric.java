package org.firstinspires.ftc.teamcode.teleops.test;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/** Test tool: robot-centric drive + raw Pinpoint position readout (no Pedro). */
@TeleOp(name = "RobotCentric", group = "Linear Opmode")
public class FirstTeleOp_RobotCentric extends RobotOpMode {

    private GamepadEx driver;
    private MecanumDrive body;
    private GoBildaPinpointDriver pinpoint;
    private final ElapsedTime runtime = new ElapsedTime();
    private boolean calibrating = true;

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
        pinpoint.resetPosAndIMU();

        schedule(new SequentialCommandGroup(
                // Wait for the Pinpoint to calibrate (max 1.5 s), robot does not drive meanwhile.
                new WaitUntilCommand(() -> {
                    pinpoint.update();
                    return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
                }).withTimeout(1500),
                new InstantCommand(() -> {
                    calibrating = false;
                    runtime.reset();
                }),
                new RunCommand(() -> {
                    pinpoint.update();
                    body.teleop(driver, telemetry);
                })
        ));
    }

    @Override
    protected void addTelemetry() {
        if (calibrating) {
            telemetry.addData("Status", "Calibrating odometry");
            return;
        }
        telemetry.addData("Robot X (mm)", pinpoint.getPosX(DistanceUnit.MM));
        telemetry.addData("Robot Y (mm)", pinpoint.getPosY(DistanceUnit.MM));
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }
}
