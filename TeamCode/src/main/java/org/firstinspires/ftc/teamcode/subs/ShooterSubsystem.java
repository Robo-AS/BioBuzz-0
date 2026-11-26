package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotor feeder, launcher;

    private boolean feederOn = false;
    private boolean launcherOn = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
    }

    public void toggleFeeder() {
        feederOn = !feederOn;
        feeder.setPower(feederOn ? 1.0 : 0.0);
    }

    public boolean isFeederOn() {
        return feederOn;
    }

    public void toggleLauncher() {
        launcherOn = !launcherOn;
        launcher.setPower(launcherOn ? 0.85 : 0.0);
    }
}