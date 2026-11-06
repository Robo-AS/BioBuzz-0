package org.firstinspires.ftc.teamcode.servotest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * TEST TOOL — find positions for the "servo_intake" servo.
 * Deliberately plain (no FTCLib, no command scheduler) so nothing can get in the way.
 *
 *   START        servo goes to POSITION_A and stays powered
 *   A            jump between POSITION_A and POSITION_B (big, easy-to-see move)
 *   dpad up      position + STEP (one step per press)
 *   dpad down    position - STEP
 *   B            intake motor on (full speed) / off
 *
 * Servo positions are 0.0 - 1.0 over the servo's full range
 * (e.g. 0.01 is only ~2.7 degrees on a 270-degree servo, ~1.8 on a 180-degree one).
 */
@TeleOp(name = "Servo Tuner", group = "Test")
public class ServoTuner extends LinearOpMode {

    private static final String SERVO_NAME = "servo_intake";
    private static final double POSITION_A = 0.21;
    private static final double POSITION_B = 0.9;
    private static final double STEP = 0.01;


    private static final double INTAKE_POWER = 1.0;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "servo_intake");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        boolean intakeOn = false, lastB = false;
        double position = POSITION_A;
        boolean lastA = false, lastUp = false, lastDown = false;

        telemetry.addLine("Servo Tuner ready — press START (the ▶ button).");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (a && !lastA) position = (position < 0.5) ? POSITION_B : POSITION_A;
            if (up && !lastUp) position += STEP;
            if (down && !lastDown) position -= STEP;
            position = Math.max(0.0, Math.min(1.0, position));
            lastA = a;
            lastUp = up;
            lastDown = down;

            servo.setPosition(position);

            boolean b = gamepad1.b;
            if (b && !lastB) intakeOn = !intakeOn;
            lastB = b;
            intake.setPower(intakeOn ? INTAKE_POWER : 0.0);

            telemetry.addData("Servo", "%s (port %d)", SERVO_NAME, servo.getPortNumber());
            telemetry.addData("Commanded position", "%.2f", position);
            telemetry.addData("Servo reports", "%.2f", servo.getPosition());
            telemetry.addData("Gamepad1 A / up / down", "%b / %b / %b", a, up, down);
            telemetry.addData("Intake", intakeOn ? "ON (full speed)" : "OFF");
            telemetry.update();
        }

        intake.setPower(0.0);
    }
}
