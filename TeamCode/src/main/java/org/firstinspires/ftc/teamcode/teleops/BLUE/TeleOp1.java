package org.firstinspires.ftc.teamcode.teleops.BLUE;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subs.IntakeServo;
import org.firstinspires.ftc.teamcode.teleops.BaseTeleOp;

@TeleOp(name = "TeleOp1", group = "Test")
public class TeleOp1 extends BaseTeleOp {
    private static final PoseFactory P = PoseFactory.degrees();
    private static final Pose START = P.of(132, 9, 90);

    // Goal position (Pedro units, inches).
    // TODO check: y = 190 is outside the 144" field.
    private static final double GOAL_X = 84;
    private static final double GOAL_Y = 71;

    private IntakeServo intakeServo;

    @Override protected Pose startPose() { return START; }
    @Override protected double goalX() { return GOAL_X; }
    @Override protected double goalY() { return GOAL_Y; }

    @Override
    protected void configureBindings() {
        // B: intake on at full speed, press B again to stop.
        driver.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(intake.intakeCommand());

        // A: servo_intake switches between 0.21 and 0.9.
        intakeServo = new IntakeServo(hardwareMap);
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(intakeServo.toggleCommand());
    }

    @Override
    protected void onStart() {
        schedule(intakeServo.toACommand()); // servo goes to 0.21 when START is pressed
    }

    @Override
    protected void addTelemetry() {
        super.addTelemetry();
        telemetry.addData("Intake servo", intakeServo.isAtB()
                ? "B (" + IntakeServo.POSITION_B + ")"
                : "A (" + IntakeServo.POSITION_A + ")");
    }
}
