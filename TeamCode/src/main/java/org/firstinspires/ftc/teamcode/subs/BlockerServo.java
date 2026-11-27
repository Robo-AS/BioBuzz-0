package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BlockerServo extends SubsystemBase {
    private final Servo blocker;
    public BlockerServo(HardwareMap hardwareMap)
    {
        blocker=hardwareMap.get(Servo.class,"servo_blocker");
    }
    public void blocked(){
        blocker.setPosition(0.3);
    }
    public void notblocked(){blocker.setPosition(0.1);}
}
