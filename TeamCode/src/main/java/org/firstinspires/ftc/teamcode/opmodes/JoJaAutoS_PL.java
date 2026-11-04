package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** From AutoPath/S_P-L.pp.json (RED numbers; BLUE is flipped automatically). */
@Autonomous(name="JoJa S_P-L", group="Pedro")
public class JoJaAutoS_PL extends JoJaPedroAuto {
    @Override
    protected Pose startPose() {
        // Visualizer start said 90 deg, but Path 1 starts at 270 deg, so the robot is placed at 270
        return pose(34.28, 132.69, 270);
    }

    @Override
    protected void buildSteps() {
        // Path 1 (270 -> 90 is a half turn: if it spins the wrong way, use linearLongWay(270, 90))
        follow(linear(270, 90), pt(35.30, 108.66), pt(87.89, 94.92), pt(47.26, 70.99), pt(70.19, 47.19), pt(58.48, 33.28));
        // Path 2
        follow(tangent(), pt(70.80, 47.30), pt(70.59, 71.70), pt(70.77, 118.63), pt(46.71, 118.50), pt(9.74, 118.08));
    }
}
