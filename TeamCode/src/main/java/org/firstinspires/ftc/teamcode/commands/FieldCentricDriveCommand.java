package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subs.DriveSubsystem;

/**
 * DEFAULT drive command: field-centric velocity drive with the accel/decel profile and heading hold.
 * The scheduler re-runs initialize() every time it comes back (e.g. after auto aim is released),
 * which replaces the old previousAutoAim / justReleasedAutoAim bookkeeping.
 */
public class FieldCentricDriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final GamepadEx driver;

    public FieldCentricDriveCommand(DriveSubsystem drive, GamepadEx driver) {
        this.drive = drive;
        this.driver = driver;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        MecanumDrive m = drive.mecanum();
        m.setDriveVelocityMode(true);
        m.resetFreeRoamProfile();
        m.resetAim();
        drive.headingHold().reset();
    }

    @Override
    public void execute() {
        Pose pose = drive.pose();
        double turn = drive.headingHold().update(-driver.getRightX(), pose.heading(), drive.mecanum());
        drive.mecanum().fieldCentricVelocity(driver, pose.heading(), turn);
    }
}
