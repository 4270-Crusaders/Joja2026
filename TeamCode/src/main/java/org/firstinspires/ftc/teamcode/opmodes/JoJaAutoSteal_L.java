package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** From AutoPath/Steal-L.pp.json (RED numbers; BLUE is flipped automatically). */
@Autonomous(name="JoJa Steal-L", group="Pedro")
public class JoJaAutoSteal_L extends JoJaPedroAuto {
    @Override
    protected Pose startPose() {
        // Visualizer start said 90 deg, but Path 1 holds 0 deg, so the robot is placed at 0
        return pose(71.04, 133.73, 0);
    }

    @Override
    protected void buildSteps() {
        // Path 1
        follow(constant(0), pt(71.25, 118.71));
        // Path 2
        follow(linear(0, 270), pt(136.70, 130.70));
        // Path 3 (visualizer had "reverse" on: use linearLongWay(270, 180) if it should turn the long way)
        follow(linear(270, 180), pt(94.36, 130.53), pt(59.09, 118.56), pt(10.33, 117.98));
    }
}
