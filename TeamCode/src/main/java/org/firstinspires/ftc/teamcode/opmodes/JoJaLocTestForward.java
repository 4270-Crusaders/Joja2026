package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Drive forward 3 ft (MOVE_INCHES) and log the pose. See JoJaLocTest. */
@Autonomous(name="JoJa Loc Test 1 Forward 3ft", group="Test")
public class JoJaLocTestForward extends JoJaLocTest {
    @Override protected Move move() { return Move.FORWARD; }
}
