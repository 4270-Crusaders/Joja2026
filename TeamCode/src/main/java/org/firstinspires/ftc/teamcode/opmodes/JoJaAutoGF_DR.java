package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** From AutoPath/GF_D-R.pp.json (RED numbers; BLUE is flipped automatically). */
@Autonomous(name="JoJa GF_D-R", group="Pedro")
public class JoJaAutoGF_DR extends JoJaPedroAuto {
    @Override
    protected Pose startPose() {
        return pose(56.00, 8.00, 90);
    }

    @Override
    protected void buildSteps() {
        // Path 1 in the file only moves 0.1 in (56,8 -> 56.1,8), so it's skipped. Put start actions here.
        // Path 2
        follow(linear(90, 0), pt(55.64, 36.55), pt(34.60, 18.12), pt(34.49, 49.58), pt(10.28, 46.52));
        // Path 3 (visualizer had "reverse" on: use linearLongWay(0, 270) if it should turn the long way)
        follow(linear(0, 270), pt(42.09, 40.72), pt(10.21, 131.93), pt(61.37, 112.54), pt(57.95, 134.02));
        // Path 4
        follow(tangent(), pt(58.19, 118.21), pt(28.85, 75.08), pt(9.59, 95.45));
    }
}
