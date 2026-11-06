package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Drive Encoder Test", group = "Test")
public class DriveEncoderTest extends OpMode {
    private MecanumDrive drive;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        drive.setDriveVelocityMode(true);
    }

    @Override
    public void loop() {
        drive.frontLeft.setVelocity(gamepad1.dpad_up ? 350.0 : 0.0);
        drive.frontRight.setVelocity(gamepad1.dpad_right ? 350.0 : 0.0);
        drive.rearLeft.setVelocity(gamepad1.dpad_left ? 350.0 : 0.0);
        drive.rearRight.setVelocity(gamepad1.dpad_down ? 350.0 : 0.0);

        telemetry.addData("FL ticks / speed", "%d / %.1f",
                drive.frontLeft.getCurrentPosition(), drive.frontLeft.getVelocity());
        telemetry.addData("FR ticks / speed", "%d / %.1f",
                drive.frontRight.getCurrentPosition(), drive.frontRight.getVelocity());
        telemetry.addData("RL ticks / speed", "%d / %.1f",
                drive.rearLeft.getCurrentPosition(), drive.rearLeft.getVelocity());
        telemetry.addData("RR ticks / speed", "%d / %.1f",
                drive.rearRight.getCurrentPosition(), drive.rearRight.getVelocity());

        telemetry.addData("FL max ticks/s",
                drive.frontLeft.getMotorType().getAchieveableMaxTicksPerSecond());
        telemetry.addData("FR max ticks/s",
                drive.frontRight.getMotorType().getAchieveableMaxTicksPerSecond());
        telemetry.addData("RL max ticks/s",
                drive.rearLeft.getMotorType().getAchieveableMaxTicksPerSecond());
        telemetry.addData("RR max ticks/s",
                drive.rearRight.getMotorType().getAchieveableMaxTicksPerSecond());

        telemetry.addData("FL PIDF",
                drive.frontLeft.getPIDFCoefficients(
                        com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("FR PIDF",
                drive.frontRight.getPIDFCoefficients(
                        com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("RL PIDF",
                drive.rearLeft.getPIDFCoefficients(
                        com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("RR PIDF",
                drive.rearRight.getPIDFCoefficients(
                        com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER));

        telemetry.addData("FL mode / power / port", "%s / %.2f / %d",
                drive.frontLeft.getMode(), drive.frontLeft.getPower(),
                drive.frontLeft.getPortNumber());
        telemetry.addData("RL mode / power / port", "%s / %.2f / %d",
                drive.rearLeft.getMode(), drive.rearLeft.getPower(),
                drive.rearLeft.getPortNumber());
        telemetry.addData("FR mode / power / port", "%s / %.2f / %d",
                drive.frontRight.getMode(), drive.frontRight.getPower(),
                drive.frontRight.getPortNumber());
        telemetry.addData("RR mode / power / port", "%s / %.2f / %d",
                drive.rearRight.getMode(), drive.rearRight.getPower(),
                drive.rearRight.getPortNumber());
        telemetry.update();
    }
}