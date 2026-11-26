package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {

    public static final String FLYWHEEL_NAME = "flywheel";

    // ATENȚIE: Modifică dacă motorul tău nu are 28 ticks/rev
    public static final double TICKS_PER_REV = 28.0;

    private final DcMotor feeder;
    private final DcMotorEx flywheel;

    private boolean feederOn = false;
    private boolean flywheelOn = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);

        // Activează PID-ul intern
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(0.0005, 0.0, 0.0, 0.00021);
    }

    public void toggleFeeder() {
        setFeeder(!feederOn);
    }

    public boolean isFeederOn() {
        return feederOn;
    }

    public void toggleflywheel() {
        flywheelOn = !flywheelOn;
        if (!flywheelOn) {
            flywheel.setPower(0);
        }
    }

    public void setFeeder(boolean on) {
        feederOn = on;
        feeder.setPower(on ? 1.0 : 0.0);
    }

    public void updateFlywheelRpm(double targetRpm) {
        if (flywheelOn) {
            double ticksPerSecond = (targetRpm * TICKS_PER_REV) / 60.0;
            flywheel.setVelocity(ticksPerSecond);
        }
    }

    public void stopAll() {
        setFeeder(false);
        flywheelOn = false;
        flywheel.setPower(0);
    }
}