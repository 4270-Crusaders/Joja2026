package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** From AutoPath/2tipsolo.pp.json (RED numbers; BLUE is flipped automatically). Pedro 3, 2026-11-12. */
@Autonomous(name="JoJa 2TipSolo", group="Pedro")
public class JoJaAuto2TipSolo extends JoJaPedroAuto {
    @Override
    protected Pose startPose() {
        return pose(59.05, 10.37, 90);
    }

    @Override
    protected void buildSteps() {
        // The visualizer had "reverse" on for Paths 2, 3 and 5. They turn the SHORT way here
        // (Path 2 long way = a 315 deg spin). Use linearLongWay(...) on a path if it should go the long way.

        // Path 1
        follow(linear(90, 45), pt(47.27, 7.89), pt(36.82, 20.24), pt(13.22, 12.88));
        // Path 2
        follow(linear(45, 0), pt(8.23, 7.84));
        // Path 3
        follow(linear(0, 90), pt(33.88, 17.35), pt(59.20, 9.41));
        // Path 4
        follow(linear(90, 0), pt(58.39, 42.68), pt(28.83, 38.34), pt(9.33, 46.86));
        // Path 5
        follow(linear(0, 270), pt(34.48, 46.20), pt(15.26, 118.90), pt(58.22, 102.87), pt(59.28, 133.59));
        // Path 6
        follow(linear(270, 270), pt(57.18, 115.99), pt(45.13, 122.36), pt(46.30, 133.91));
        // Path 7
        follow(linear(270, 270), pt(45.81, 117.86), pt(60.13, 118.49), pt(59.63, 133.72));
        // Path 8
        follow(tangent(), pt(58.17, 97.64), pt(34.49, 132.04), pt(9.87, 117.52));
    }
}
