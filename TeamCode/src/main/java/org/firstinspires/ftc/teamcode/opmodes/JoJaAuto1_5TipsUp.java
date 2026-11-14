package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * From AutoPath/1.5tips-UP.pp.json (RED numbers; BLUE is flipped automatically). 2026-11-14.
 *   Path 1 (no name) -> wait 15 s | Shoot1 -> shoot | Flower -> match loader down + intake 3 s |
 *   Shoot2 (match loader goes up while driving) -> shoot | Park
 */
@Autonomous(name="JoJa 1.5tips-UP", group="Pedro")
public class JoJaAuto1_5TipsUp extends JoJaPedroAuto {
    @Override
    protected Pose startPose() {
        // The visualizer start says 90 deg, but Path 1 is drawn at 270 deg, so the robot is placed at 270.
        return pose(60.30, 130.76, 270);
    }

    @Override
    protected void buildSteps() {
        path("", Then.NOTHING, constant(270),
                pt(66.78, 68.92));
        waitFor(15.0); // Path 1: wait 15 s at the end
        // 270 -> 90 is a half turn: either way is 180 deg, Pedro picks one
        path("Shoot1", Then.SHOOT, linear(270, 90),
                pt(79.96, 71.72), pt(59.52, 15.32));
        path("Flower", Then.FLOWER, linear(90, 270),
                pt(70.80, 47.30), pt(70.59, 71.70), pt(70.77, 118.63), pt(46.71, 118.50), pt(49.78, 126.40));
        path("Shoot2", Then.SHOOT, constant(270),      // match loader goes up as this path starts (after Flower)
                pt(47.00, 107.88), pt(60.33, 131.09));
        path("Park", Then.PARK, linear(270, 0),
                pt(12.85, 117.59));
    }
}
