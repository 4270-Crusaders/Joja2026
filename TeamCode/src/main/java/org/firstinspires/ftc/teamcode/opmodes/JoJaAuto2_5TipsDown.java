package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * From AutoPath/2.5tips-DOWN.pp.json (RED numbers; BLUE is flipped automatically). 2026-11-14.
 * Each path's NAME is what the robot does (see JoJaPedroAuto.Then).
 */
@Autonomous(name="JoJa 2.5tips-DOWN", group="Pedro")
public class JoJaAuto2_5TipsDown extends JoJaPedroAuto {
    @Override
    protected Pose startPose() {
        return pose(54.87, 8.70, 90);
    }

    @Override
    protected void buildSteps() {
        path("Shoot1", Then.SHOOT, constant(90),
                pt(58.50, 14.82));
        path("Garden", Then.NOTHING, constant(90),
                pt(47.27, 7.89), pt(36.82, 20.24), pt(20.20, 10.94), pt(9.88, 18.52));
        // match loader DOWN first, drive in, intake + indexer 3 s
        path("MatchLoader-DOWN", Then.LOADER_DOWN, constant(90),
                pt(9.00, 11.83));
        // drive out, then match loader UP
        path("Matchloader-UP", Then.LOADER_UP, constant(90),
                pt(10.14, 18.40));
        // 90 -> 270 is a half turn: either way is 180 deg, Pedro picks one
        path("Shoot2", Then.SHOOT, linear(90, 270),
                pt(34.48, 46.20), pt(8.16, 125.38), pt(58.22, 102.87), pt(60.33, 131.09));
        // Not in your step list: Flower1 ends at the top flower and goes straight to a shoot, so it does the
        // Flower action (match loader down + intake 3 s at the end). Change to Then.NOTHING to just drive.
        path("Flower1", Then.FLOWER, constant(270),
                pt(57.18, 115.99), pt(45.13, 122.36), pt(49.78, 126.40));
        path("Shoot3", Then.SHOOT, constant(270),      // match loader goes up as this path starts (after Flower1)
                pt(45.81, 117.86), pt(60.13, 118.49), pt(60.33, 131.09));
        path("Flower2", Then.NOTHING, linear(270, 0),
                pt(11.54, 99.41), pt(37.69, 41.93), pt(22.45, 47.15));
        path("Matchloader-DOWN2", Then.LOADER_DOWN, constant(0),
                pt(16.70, 46.50));
        path("Matchloader-UP2", Then.LOADER_UP, constant(0),
                pt(22.29, 46.73));
        path("Shoot4", Then.SHOOT, linear(0, 90),
                pt(38.86, 38.09), pt(58.20, 14.86));
        path("Park", Then.PARK, tangent(),
                pt(23.50, 36.86), pt(29.47, 94.86), pt(10.29, 93.92));
    }
}
