package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subs.DriveSubsystem;

/**
 * Auto aim: driver keeps translation, MecanumDrive's PD loop turns toward the goal.
 * Runs in power mode (same as before). When it ends, the default drive command
 * takes over again and switches back to velocity mode.
 */
public class AutoAimDriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final GamepadEx driver;

    public AutoAimDriveCommand(DriveSubsystem drive, GamepadEx driver) {
        this.drive = drive;
        this.driver = driver;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.mecanum().setDriveVelocityMode(false);
        drive.mecanum().resetFreeRoamProfile();
    }

    @Override
    public void execute() {
        Pose pose = drive.pose();
        drive.mecanum().fieldCentricAutoAim(driver, pose.x(), pose.y(), pose.heading(), true);
    }

    @Override
    public void end(boolean interrupted) {
        drive.mecanum().resetAim();
    }

    /** True when within tolerance of the goal — use it to gate the shooter later. */
    public boolean isAimed() {
        return drive.mecanum().aimedAtGoal;
    }
}
