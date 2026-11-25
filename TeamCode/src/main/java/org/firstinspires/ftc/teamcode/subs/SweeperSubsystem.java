package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SweeperSubsystem extends SubsystemBase {

    private final CRServo sweeper;

    public static double DEFAULT_DOWN_POWER = 0.8;
    public static double DEFAULT_UP_POWER = -0.8;

    public SweeperSubsystem(HardwareMap hardwareMap) {
        sweeper = hardwareMap.get(CRServo.class, "sweeper");
    }

    public void setPower(double power) {
        sweeper.setPower(power);
    }

    public void godown() {
        sweeper.setPower(DEFAULT_DOWN_POWER);
    }

    public void goback() {
        sweeper.setPower(DEFAULT_UP_POWER);
    }

    public void stop() {
        sweeper.setPower(0.0);
    }

    public Command goDownCommand(long durationMs) {
        return new SequentialCommandGroup(
                new InstantCommand(this::godown, this),
                new WaitCommand(durationMs),
                new InstantCommand(this::stop, this)
        );
    }

    public Command goBackCommand(long durationMs) {
        return new SequentialCommandGroup(
                new InstantCommand(this::goback, this),
                new WaitCommand(durationMs),
                new InstantCommand(this::stop, this)
        );
    }
}
