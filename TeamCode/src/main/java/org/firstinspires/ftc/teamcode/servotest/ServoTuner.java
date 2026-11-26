package org.firstinspires.ftc.teamcode.servotest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/*
 * TEST TOOL — continuous rotation servo "faras", moved to exact angles using its feedback wire
 * (Axon-style, analog input "farasEncoder").
 * Deliberately plain (no FTCLib, no command scheduler) so nothing can get in the way.
 *
 *   START        HAND mode: NO power to the servo, turn it by hand and watch "Angle".
 *                Where it is at START = 56° (ANGLE_A).
 *   Y            HAND mode on/off (no power <-> powered)
 *   A            powered: go to the other angle, 56° <-> 360° (first press goes to 360°)
 *   dpad up/down manual power +/- 0.1 (cancels the target; press A to go back to targets)
 *   B            intake motor on (full speed) / off
 *
 * Angles are measured from where the servo is at START (that spot = 56°), counted continuously
 * from the feedback wire, so 56 -> 360 is always exactly +304° of rotation, no wrap-around.
 */
@TeleOp(name = "Servo Tuner", group = "Test")
public class ServoTuner extends LinearOpMode {

    private static final String SERVO_NAME = "faras";
    private static final String FEEDBACK_NAME = "farasEncoder"; // analog input for the feedback wire

    // ---- The two target angles (feedback degrees) ----
    private static final double ANGLE_A = 56;   // start position (where the servo is at START)
    private static final double ANGLE_B = 380;

    // ---- Go-to-angle control ----
    /** If dpad up (positive power) makes the angle go DOWN, set this to true. */
    private static final boolean INVERT_FEEDBACK = false;
    private static final double KP = 0.005;              // power per degree of error (30° off -> 0.3)
    private static final double MIN_POWER = 0.05;       // enough to overcome the servo's deadband
    private static final double MAX_POWER = 1.0;          // full speed
    private static final double TOLERANCE_DEG = 2.0;

    private static final double STEP = 0.1;
    private static final double INTAKE_POWER = 1.0;

    @Override
    public void runOpMode() {
        // Works whether "faras" is configured as a Continuous Rotation Servo or as a plain Servo.
        CRServo crServo = hardwareMap.tryGet(CRServo.class, SERVO_NAME);
        Servo servo = crServo == null ? hardwareMap.get(Servo.class, SERVO_NAME) : null;
        AnalogInput feedback = hardwareMap.tryGet(AnalogInput.class, FEEDBACK_NAME);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

        boolean atB = false;            // false = target ANGLE_A, true = target ANGLE_B
        boolean goToTarget = false;
        boolean handMode = true;        // starts with NO power so it can be turned by hand
        boolean pwmOn = true;
        double power = 0.0;
        boolean intakeOn = false;
        boolean lastA = false, lastB = false, lastY = false, lastUp = false, lastDown = false;
        double lastRaw = Double.NaN;
        double travelled = 0;           // continuous rotation since START, in degrees

        telemetry.addData("Servo", "%s as %s", SERVO_NAME, crServo != null ? "Continuous Rotation Servo" : "Servo");
        telemetry.addData("Feedback", feedback != null ? FEEDBACK_NAME + " found" : "NOT FOUND — targets can't work");
        telemetry.addLine("Press START (▶). Where the servo is now = 56°.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // ---------------- Buttons (one step per press) ----------------
            if (gamepad1.y && !lastY) {
                handMode = !handMode;
                goToTarget = false;
                power = 0.0;
            }
            if (gamepad1.a && !lastA) {
                atB = !atB;                 // every press: go to the other angle (56 <-> 360)
                goToTarget = true;
                handMode = false;
            }
            if (gamepad1.dpad_up && !lastUp) { handMode = false; goToTarget = false; power += STEP; }
            if (gamepad1.dpad_down && !lastDown) { handMode = false; goToTarget = false; power -= STEP; }
            if (gamepad1.b && !lastB) intakeOn = !intakeOn;
            lastA = gamepad1.a; lastB = gamepad1.b; lastY = gamepad1.y;
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;

            // ---------------- Read the angle ----------------
            double deg = Double.NaN, angle = Double.NaN, target = atB ? ANGLE_B : ANGLE_A, error = 0;
            if (feedback != null) {
                deg = feedback.getVoltage() / feedback.getMaxVoltage() * 360.0;
                if (INVERT_FEEDBACK) deg = 360.0 - deg;
                if (!Double.isNaN(lastRaw)) {
                    double delta = deg - lastRaw;          // unwrap across the 360°/0° seam
                    if (delta > 180) delta -= 360;
                    if (delta < -180) delta += 360;
                    travelled += delta;
                }
                lastRaw = deg;
                angle = ANGLE_A + travelled;               // START position = 56°
                error = target - angle;
            }

            // ---------------- Control ----------------
            if (handMode) {
                power = 0.0;
            } else if (goToTarget) {
                if (feedback == null || Math.abs(error) <= TOLERANCE_DEG) {
                    power = 0.0;
                } else {
                    double out = KP * error;
                    out = Math.copySign(Math.max(Math.abs(out), MIN_POWER), out);
                    power = Math.max(-MAX_POWER, Math.min(MAX_POWER, out));
                }
            } else {
                power = Math.round(Math.max(-1.0, Math.min(1.0, power)) * 10) / 10.0; // clean 0.1 steps
            }

            if (handMode) {
                // NO signal at all -> the servo goes limp and can be rolled by hand.
                if (pwmOn) {
                    if (crServo instanceof CRServoImplEx) ((CRServoImplEx) crServo).setPwmDisable();
                    else if (crServo != null) crServo.setPower(0);
                    if (servo instanceof ServoImplEx) ((ServoImplEx) servo).setPwmDisable();
                    pwmOn = false;
                }
            } else {
                if (!pwmOn) {
                    if (crServo instanceof CRServoImplEx) ((CRServoImplEx) crServo).setPwmEnable();
                    if (servo instanceof ServoImplEx) ((ServoImplEx) servo).setPwmEnable();
                    pwmOn = true;
                }
                if (crServo != null) crServo.setPower(power);
                else servo.setPosition(0.5 + power / 2.0); // plain-Servo config: 0.5 = stop
            }
            intake.setPower(intakeOn ? INTAKE_POWER : 0.0);

            // ---------------- Telemetry ----------------
            telemetry.addData("Mode", handMode ? "HAND — no power, turn it by hand (A = go to target, Y = power on)"
                    : goToTarget ? "GO TO TARGET (A = switch, Y = hand mode)" : "manual (A = back to target, Y = hand mode)");
            telemetry.addData("Target", "%.0f°", target);
            if (feedback != null) {
                telemetry.addData("Angle", "%.1f°", angle);
                telemetry.addData("Error", "%.1f°%s", error,
                        goToTarget && Math.abs(error) <= TOLERANCE_DEG ? "  ✓ AT TARGET" : "");
                telemetry.addData("Feedback", "%.1f° raw  (%.3f V)", deg, feedback.getVoltage());
            } else {
                telemetry.addLine("No feedback: add the feedback wire as Analog Input \"" + FEEDBACK_NAME + "\"");
            }
            telemetry.addData("Servo power", handMode ? "OFF (no signal)" : String.format("%.2f", power));
            telemetry.addData("Intake (B)", intakeOn ? "ON (full speed)" : "OFF");
            telemetry.update();
        }

        if (crServo != null) crServo.setPower(0); else servo.setPosition(0.5);
        intake.setPower(0.0);
    }
}
