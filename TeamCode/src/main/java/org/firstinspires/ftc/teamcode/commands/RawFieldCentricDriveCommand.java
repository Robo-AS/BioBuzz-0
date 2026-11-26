package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subs.DriveSubsystem;

/**
 * Free-roam drive WITHOUT motor encoders and WITHOUT motion profiling:
 * raw field-centric power straight from the sticks (RUN_WITHOUT_ENCODER).
 * Heading hold is kept (it uses the Pinpoint heading, not the motor encoders).
 */
public class RawFieldCentricDriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final GamepadEx driver;

    public RawFieldCentricDriveCommand(DriveSubsystem drive, GamepadEx driver) {
        this.drive = drive;
        this.driver = driver;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        MecanumDrive m = drive.mecanum();
        m.setDriveVelocityMode(false); // RUN_WITHOUT_ENCODER: plain power, no encoder speed control
        m.resetAim();
        drive.headingHold().reset();
    }

    @Override
    public void execute() {
        Pose pose = drive.pose();
        double turn = drive.headingHold().update(-driver.getRightX(), pose.heading(), drive.mecanum());
        drive.mecanum().fieldCentric(driver, pose.heading(), turn); // raw power, no accel/decel limits
    }
}
