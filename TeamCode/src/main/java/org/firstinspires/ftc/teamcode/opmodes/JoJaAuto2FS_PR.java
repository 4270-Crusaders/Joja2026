package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** From AutoPath/2FS_P-R.pp.json (RED numbers; BLUE is flipped automatically). */
@Autonomous(name="JoJa 2FS_P-R", group="Pedro")
public class JoJaAuto2FS_PR extends JoJaPedroAuto {
    @Override
    protected Pose startPose() {
        // Visualizer start said 90 deg, but Path 1 starts at 270 deg, so the robot is placed at 270
        return pose(58.84, 10.58, 270);
    }

    @Override
    protected void buildSteps() {
        // Path 1
        follow(linear(270, 0), pt(34.58, 46.44), pt(9.33, 46.86));
        // Path 2 (visualizer had "reverse" on: use linearLongWay(0, 270) if it should turn the long way)
        follow(linear(0, 270), pt(34.48, 46.20), pt(15.26, 118.90), pt(58.22, 102.87), pt(59.28, 134.01));
        // Path 3
        follow(tangent(), pt(58.43, 103.46), pt(17.15, 126.54), pt(10.17, 117.62));
    }
}
