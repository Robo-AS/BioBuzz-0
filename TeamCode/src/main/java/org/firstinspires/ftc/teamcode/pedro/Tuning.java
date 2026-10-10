package org.firstinspires.ftc.teamcode.pedro;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

public class Tuning {

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

}
