package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Starts at the far position with 4 preloads: drive toward the HIVE, turn to face it, dump, park. */
@Configurable
@Autonomous(name="JoJa Auto Far", group="Competition")
public class JoJaAutoFar extends JoJaAutoBase {
    // All PLACEHOLDERS: tune on the field (live-editable in Panels)
    public static double APPROACH_POWER = 0.6;
    public static double APPROACH_S     = 1.8;
    public static double TURN_POWER     = 0.4;  // + = clockwise on Blue, mirrored on Red
    public static double TURN_S         = 0.5;
    public static double SHOOT_S        = 2.0;
    public static double PARK_POWER     = 0.5;
    public static double PARK_S         = 0.8;

    @Override
    protected void buildSteps() {
        spinUp();
        drive(APPROACH_POWER, 0, 0, APPROACH_S);
        drive(0, 0, TURN_POWER, TURN_S);
        shoot(SHOOT_S);
        stopFlywheel();
        drive(PARK_POWER, 0, 0, PARK_S);
    }
}
