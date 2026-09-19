package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.Constants.ROBOT_SPEED;
import java.util.Arrays;
import java.util.List;

public class MecanumDrive {
    public DcMotorEx frontRight, rearLeft, frontLeft, rearRight;
    private List <DcMotorEx> motors;
    double reverse = 1.0;
    public static double powerReduction = 20;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");

        motors = Arrays.asList(frontRight, rearLeft, frontLeft, rearRight);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void teleop(GamepadEx gamepad, Telemetry telemetry) {
        double x = -gamepad.getLeftX();
        double y = -gamepad.getLeftY();
        double r = -gamepad.getRightX();

        x = addons(x) * reverse;
        y = addons(y) * reverse;
        r = addons(r);

        double LeftFrontPower = (y + x + r);
        double RightFrontPower = (y - x - r);
        double LeftRearPower = (y - x + r);
        double RightRearPower = (y + x - r);

        frontLeft.setPower(clip(LeftFrontPower));
        frontRight.setPower(clip(RightFrontPower));
        rearLeft.setPower(clip(LeftRearPower));
        rearRight.setPower(clip(RightRearPower));
    }
    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value * ROBOT_SPEED;
    }
    double clip(double power) {
        return Math.max(-1.0, Math.min(power, 1.0));
    }
    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("---ROBOT CENTRIC DRIVE---");

        telemetry.addData("Direction Multiplier: ", reverse);
        telemetry.addData("Speed Multiplier: ", ROBOT_SPEED);

        telemetry.addData("LeftRear Position: ", rearLeft.getCurrentPosition());
        telemetry.addData("RightRear Position: ", rearRight.getCurrentPosition());
        telemetry.addData("LeftFront Position: ", frontLeft.getCurrentPosition());
        telemetry.addData("RightFront Position: ", frontRight.getCurrentPosition());

        telemetry.addData("LeftRear Power: ", rearLeft.getPower());
        telemetry.addData("RightRear Power: ", rearRight.getPower());
        telemetry.addData("LeftFront Power: ", frontLeft.getPower());
        telemetry.addData("RightFront Power: ", frontRight.getPower());
    }
}