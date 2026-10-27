package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Starts near the HIVE with 4 preloads: back into range, dump, strafe out to park. */
@Configurable
@Autonomous(name="JoJa Auto Near HIVE", group="Competition")
public class JoJaAutoNear extends JoJaAutoBase {
    // All PLACEHOLDERS: tune on the field (live-editable in Panels)
    public static double BACKUP_POWER = 0.5;
    public static double BACKUP_S     = 0.8;
    public static double SHOOT_S      = 2.0;
    public static double PARK_STRAFE_POWER = 0.5;  // + = right on Blue, mirrored on Red
    public static double PARK_S       = 1.2;

    @Override
    protected void buildSteps() {
        spinUp();
        drive(-BACKUP_POWER, 0, 0, BACKUP_S);
        shoot(SHOOT_S);
        stopFlywheel();
        drive(0, PARK_STRAFE_POWER, 0, PARK_S);
    }
}
