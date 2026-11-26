package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SubsystemsTest extends OpMode {
    boolean intakeState=false;
    boolean feederState=false;
    boolean launcherState=false;
    private DcMotor Intake,Feeder,Launcher;
    public void init()
    {
        Intake=hardwareMap.get(DcMotor.class, "intake");
        Feeder=hardwareMap.get(DcMotor.class,"feeder");
        Launcher=hardwareMap.get(DcMotor.class,"launcher");
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
            launcherState=!launcherState;
        }
        if(launcherState)
            Launcher.setPower(1);
        else
            Launcher.setPower(0);

    }



}
