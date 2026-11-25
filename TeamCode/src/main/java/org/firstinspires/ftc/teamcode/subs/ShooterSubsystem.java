package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotor feeder, flywheel;

    private boolean feederOn = false;
    private boolean flywheelOn = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void toggleFeeder() {
        setFeeder(!feederOn);
    }

    public void toggleflywheel() {
        setFlywheel(!flywheelOn);
    }

    // Metode noi pentru Auto (control exact ON / OFF)
    public void setFeeder(boolean on) {
        feederOn = on;
        feeder.setPower(on ? 1.0 : 0.0);
    }

    public void setFlywheel(boolean on) {
        flywheelOn = on;
        flywheel.setPower(on ? 0.85 : 0.0);
    }

    public void stopAll() {
        setFeeder(false);
        setFlywheel(false);
    }
}