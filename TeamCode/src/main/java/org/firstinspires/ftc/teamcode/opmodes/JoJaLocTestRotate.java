package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Turn 180 deg in place (counterclockwise) and log the pose. X/Y should barely move. See JoJaLocTest. */
@Autonomous(name="JoJa Loc Test 3 Rotate 180", group="Test")
public class JoJaLocTestRotate extends JoJaLocTest {
    @Override protected Move move() { return Move.ROTATE_180; }
}
