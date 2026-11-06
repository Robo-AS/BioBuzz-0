package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** The "servo_intake" servo: switches between two positions (found with the Servo Tuner). */
public class IntakeServo extends SubsystemBase {

    public static double POSITION_A = 0.21;
    public static double POSITION_B = 0.9;

    private final Servo servo;
    private boolean atB = false;

    public IntakeServo(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "servo_intake");
    }

    public void toA() {
        servo.setPosition(POSITION_A);
        atB = false;
    }

    public void toB() {
        servo.setPosition(POSITION_B);
        atB = true;
    }

    public void toggle() {
        if (atB) toA(); else toB();
    }

    public boolean isAtB() {
        return atB;
    }

    // ---------------- Commands ----------------

    public Command toACommand() {
        return new InstantCommand(this::toA, this);
    }

    public Command toBCommand() {
        return new InstantCommand(this::toB, this);
    }

    public Command toggleCommand() {
        return new InstantCommand(this::toggle, this);
    }
}
