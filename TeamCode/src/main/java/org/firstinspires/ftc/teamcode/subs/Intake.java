package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    public static double INTAKE_POWER = 0.8;
    public static double OUTTAKE_POWER = -1.0;

    private final DcMotorEx intakeMotor;
    private double lastPower = Double.NaN;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    /** Only talks to the hub when the power actually changes (saves loop time). */
    public void setPower(double power) {
        if (power != lastPower) {
            intakeMotor.setPower(power);
            lastPower = power;
        }
    }

    public void stop() {
        setPower(0);
    }

    // ---------------- Commands ----------------

    /** Runs the intake while scheduled, stops when it ends. Use with whenHeld / in autos. */
    public Command intakeCommand() {
        return new StartEndCommand(() -> setPower(INTAKE_POWER), this::stop, this);
    }

    public Command outtakeCommand() {
        return new StartEndCommand(() -> setPower(OUTTAKE_POWER), this::stop, this);
    }
}
