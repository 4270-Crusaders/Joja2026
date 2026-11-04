package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** From AutoPath/OF_P-L.pp.json (RED numbers; BLUE is flipped automatically). */
@Autonomous(name="JoJa OF_P-L", group="Pedro")
public class JoJaAutoOF_PL extends JoJaPedroAuto {
    @Override
    protected Pose startPose() {
        return pose(59.13, 133.31, 90);
    }

    @Override
    protected void buildSteps() {
        // Path 1 (drives backward)
        follow(tangentReversed(), pt(58.54, 118.57), pt(35.11, 118.50));
        // Path 2
        follow(linear(0, 270), pt(46.92, 118.42), pt(46.80, 129.73));
        // Path 3
        follow(linear(270, 270), pt(46.43, 109.73), pt(58.66, 109.31), pt(58.89, 133.82));
        // Path 4
        follow(tangent(), pt(58.90, 99.11), pt(22.64, 118.79), pt(9.75, 117.54));
    }
}
