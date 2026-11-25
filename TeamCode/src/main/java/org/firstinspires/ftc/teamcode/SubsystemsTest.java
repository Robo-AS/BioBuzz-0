package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SubsystemsTest extends OpMode {
    boolean intakeState=false;
    boolean feederState=false;
    boolean flywheelState=false;
    private DcMotor Intake,Feeder,flywheel;
    public void init()
    {
        Intake=hardwareMap.get(DcMotor.class, "intake");
        Feeder=hardwareMap.get(DcMotor.class,"feeder");
        flywheel=hardwareMap.get(DcMotor.class,"flywheel");
    }
    public void loop()
    {
        if(gamepad1.bWasPressed())
            intakeState=!intakeState;
        if(intakeState)
            Intake.setPower(1);
        else
            Intake.setPower(0);
        if(gamepad1.xWasPressed())
            feederState=!feederState;
        if(feederState)
        {
            Feeder.setPower(1);
        }
        else
        {
            Feeder.setPower(0);
        }
        if (gamepad1.yWasPressed()){
            flywheelState=!flywheelState;
        }
        if(flywheelState)
            flywheel.setPower(1);
        else
            flywheel.setPower(0);

    }



}
