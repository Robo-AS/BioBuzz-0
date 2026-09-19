package org.firstinspires.ftc.teamcode.teleops;
import static org.firstinspires.ftc.teamcode.MecanumDrive.powerReduction;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@TeleOp(name = "TeleOpDrive", group= "Linear Opmode")
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

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            driver.readButtons();
            body.teleop(driver, telemetry);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}