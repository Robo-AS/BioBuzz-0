package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LedSubsystem extends SubsystemBase {
    private final Servo led;
    private double lastColor = -1;

    public static final double OFF    = 0.7925;
    public static final double RED    = 0.6525;
    public static final double ORANGE = 0.389;
    public static final double YELLOW = 0.6725;
    public static final double GREEN  = 0.611;
    public static final double BLUE   = 0.388;
    public static final double VIOLET = 0.333;
    public static final double WHITE  = 0.277;

    public LedSubsystem(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "led");
    }

    public void setColor(double color) {
        if (color != lastColor) {
            led.setPosition(color);
            lastColor = color;
        }
    }
}