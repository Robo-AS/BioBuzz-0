package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ShooterSubsystem extends SubsystemBase {

    public static final String FLYWHEEL_NAME = "flywheel";
    public static final double TICKS_PER_REV = 28.0;

    // Exact coeficienții din Shooter Tuner
    public static double kP = 0.0005;
    public static double kF = 0.00021;

    private final DcMotor feeder;
    private final DcMotorEx flywheel;

    private boolean feederOn = false;
    private boolean flywheelOn = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);

        // Folosim RUN_WITHOUT_ENCODER + bucla noastră PIDF (exact ca în Shooter Tuner)
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            flywheel.setPower(0.0);
        }
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    public void setFeeder(boolean on) {
        feederOn = on;
        feeder.setPower(on ? 1.0 : 0.0);
    }

    public double getActualRpm() {
        return (flywheel.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public void updateFlywheelRpm(double targetRpm) {
        if (!flywheelOn) {
            flywheel.setPower(0.0);
            return;
        }

        double actualRpm = getActualRpm();
        double error = targetRpm - actualRpm;
        double power = (kF * targetRpm) + (kP * error);

        flywheel.setPower(Range.clip(power, 0.0, 1.0));
    }

    public void stopAll() {
        setFeeder(false);
        flywheelOn = false;
        flywheel.setPower(0.0);
    }
}