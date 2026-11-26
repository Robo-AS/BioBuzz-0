package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodServo extends SubsystemBase {

    private final Servo servo;

    // Limitele servo-ului (poți modifica MAX_POS dacă nu vrei să meargă chiar până la 1.0)
    private static final double MIN_POS = 0.0;
    private static final double MAX_POS = 0.5;
    private static final double STEP = 0.1;

    private double currentPosition = MIN_POS;

    public HoodServo(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "hood");
        servo.setPosition(currentPosition);
    }
    public void increment() {
        currentPosition = Math.min(MAX_POS, currentPosition + STEP);
        servo.setPosition(currentPosition);
    }
    public void decrement() {
        currentPosition = Math.min(MAX_POS, currentPosition - STEP);
        servo.setPosition(currentPosition);
    }

    /** Go straight to a position (clipped to MIN_POS..MAX_POS). */
    public void setPosition(double position) {
        currentPosition = Math.max(MIN_POS, Math.min(MAX_POS, position));
        servo.setPosition(currentPosition);
    }

    public double getPosition() {
        return currentPosition;
    }
}