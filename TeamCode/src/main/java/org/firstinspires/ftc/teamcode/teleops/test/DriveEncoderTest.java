package org.firstinspires.ftc.teamcode.teleops.test;

import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/** Test tool: dpad spins one wheel at 350 ticks/s each, shows encoder + PIDF data. */
@TeleOp(name = "Drive Encoder Test", group = "Test")
public class DriveEncoderTest extends RobotOpMode {
    private MecanumDrive drive;

    @Override
    protected void initRobot() {
        drive = new MecanumDrive(hardwareMap);
        drive.setDriveVelocityMode(true);
    }

    @Override
    protected void onStart() {
        schedule(new RunCommand(() -> {
            drive.frontLeft.setVelocity(gamepad1.dpad_up ? 350.0 : 0.0);
            drive.frontRight.setVelocity(gamepad1.dpad_right ? 350.0 : 0.0);
            drive.rearLeft.setVelocity(gamepad1.dpad_left ? 350.0 : 0.0);
            drive.rearRight.setVelocity(gamepad1.dpad_down ? 350.0 : 0.0);
        }));
    }

    @Override
    protected void addTelemetry() {
        motorLine("FL", drive.frontLeft);
        motorLine("FR", drive.frontRight);
        motorLine("RL", drive.rearLeft);
        motorLine("RR", drive.rearRight);

        maxLine("FL", drive.frontLeft);
        maxLine("FR", drive.frontRight);
        maxLine("RL", drive.rearLeft);
        maxLine("RR", drive.rearRight);

        pidfLine("FL", drive.frontLeft);
        pidfLine("FR", drive.frontRight);
        pidfLine("RL", drive.rearLeft);
        pidfLine("RR", drive.rearRight);

        portLine("FL", drive.frontLeft);
        portLine("RL", drive.rearLeft);
        portLine("FR", drive.frontRight);
        portLine("RR", drive.rearRight);
    }

    private void motorLine(String name, DcMotorEx m) {
        telemetry.addData(name + " ticks / speed", "%d / %.1f", m.getCurrentPosition(), m.getVelocity());
    }

    private void maxLine(String name, DcMotorEx m) {
        telemetry.addData(name + " max ticks/s", m.getMotorType().getAchieveableMaxTicksPerSecond());
    }

    private void pidfLine(String name, DcMotorEx m) {
        telemetry.addData(name + " PIDF", m.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
    }

    private void portLine(String name, DcMotorEx m) {
        telemetry.addData(name + " mode / power / port", "%s / %.2f / %d",
                m.getMode(), m.getPower(), m.getPortNumber());
    }
}
