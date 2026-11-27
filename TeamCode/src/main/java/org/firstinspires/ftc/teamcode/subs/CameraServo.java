package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Servo that points the camera. Robot Configuration: a Servo named "camera".
 * Goes to INIT_POS as soon as the OpMode is initialized.
 * If "camera" isn't in the configuration yet, it's simply skipped (the TeleOp still runs).
 */
public class CameraServo extends SubsystemBase {

    public static double INIT_POS = 0.8;

    private final Servo servo; // null if "camera" isn't configured
    private double currentPosition = INIT_POS;

    public CameraServo(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(Servo.class, "camera");
        if (servo != null) servo.setPosition(currentPosition);
    }

    /** Every loop (after START too): keep commanding the position so it stays at INIT_POS. */
    @Override
    public void periodic() {
        if (servo != null) servo.setPosition(currentPosition);
    }

    /** Go to a position (0.0 - 1.0). */
    public void setPosition(double position) {
        currentPosition = Math.max(0.0, Math.min(1.0, position));
        if (servo != null) servo.setPosition(currentPosition);
    }

    /** Back to INIT_POS. */
    public void center() {
        setPosition(INIT_POS);
    }

    public boolean isConfigured() {
        return servo != null;
    }

    public double getPosition() {
        return currentPosition;
    }
}
