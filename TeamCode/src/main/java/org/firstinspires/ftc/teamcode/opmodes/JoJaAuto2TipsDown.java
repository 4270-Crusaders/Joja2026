package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * From AutoPath/2tips-DOWN.pp.json (RED numbers; BLUE is flipped automatically). 2026-11-14.
 * Each match loader stop is 3 parts: drive up to it | Matchloader-down: match loader DOWN, drive in,
 * intake + indexer 3 s | Matchloader-up: drive out, match loader UP.
 */
@Autonomous(name="JoJa 2tips-DOWN", group="Pedro")
public class JoJaAuto2TipsDown extends JoJaPedroAuto {
    @Override
    protected Pose startPose() {
        return pose(55.50, 8.28, 90);
    }

    @Override
    protected void buildSteps() {
        path("Shoot1", Then.SHOOT, constant(90),
                pt(58.40, 14.79));

        // Garden (bottom-left)
        path("Garden", Then.NOTHING, constant(90),
                pt(8.19, 12.94));
        path("Matchloader-DOWN", Then.LOADER_DOWN, constant(90),
                pt(8.85, 9.86));
        path("Matchloader-UP", Then.LOADER_UP, constant(90),
                pt(17.81, 16.59));
        path("Shoot2", Then.SHOOT, constant(90),
                pt(58.43, 15.48));

        // Flower 1 (left)
        path("Flower1", Then.NOTHING, linear(90, 0),
                pt(34.58, 46.44), pt(23.21, 47.07));
        path("Matchloader-down2", Then.LOADER_DOWN, constant(0),
                pt(16.70, 46.50));
        path("Matchloader-up2", Then.LOADER_UP, constant(0),
                pt(22.45, 47.15));
        path("Shoot3", Then.SHOOT, linear(0, 270),
                pt(34.48, 46.20), pt(15.26, 118.90), pt(58.22, 102.87), pt(60.33, 131.09));

        // Flower 2 (top)
        path("Flower2", Then.NOTHING, constant(270),
                pt(61.97, 122.42), pt(46.94, 118.72), pt(47.88, 120.38));
        path("Matchloader-down3", Then.LOADER_DOWN, constant(270),
                pt(49.78, 126.40));
        path("mathloader-up3", Then.LOADER_UP, constant(270),
                pt(47.15, 119.99));
        path("Shoot3", Then.SHOOT, constant(270),   // named Shoot3 again in the visualizer (the 4th shoot)
                pt(54.21, 116.66), pt(58.19, 112.01), pt(60.59, 131.03));

        path("Park", Then.PARK, tangent(),
                pt(58.43, 103.46), pt(24.88, 92.50), pt(11.21, 95.90));
    }
}
