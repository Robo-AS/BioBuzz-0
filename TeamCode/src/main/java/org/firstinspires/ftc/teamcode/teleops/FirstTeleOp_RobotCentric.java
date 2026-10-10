package org.firstinspires.ftc.teamcode.teleops;
import static org.firstinspires.ftc.teamcode.MecanumDrive.powerReduction;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@TeleOp(name = "RobotCentric", group= "Linear Opmode")
public class FirstTeleOp_RobotCentric extends LinearOpMode {

    GamepadEx driver;
    MecanumDrive body;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        driver = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        body = new MecanumDrive(hardwareMap);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        waitForStart();
        if (isStopRequested()) return;

        pinpoint.resetPosAndIMU();
        ElapsedTime calibrationTime = new ElapsedTime();
        do {
            pinpoint.update();
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) break;
            telemetry.addData("Status", "Calibrating odometry");
            telemetry.update();
            sleep(20);
        } while (opModeIsActive() && calibrationTime.seconds() < 1.5);

        runtime.reset();

        while (opModeIsActive()) {
            driver.readButtons();
            pinpoint.update();
            body.teleop(driver, telemetry);

            telemetry.addData("Robot X (mm)", pinpoint.getPosX(DistanceUnit.MM));
            telemetry.addData("Robot Y (mm)", pinpoint.getPosY(DistanceUnit.MM));
            telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
