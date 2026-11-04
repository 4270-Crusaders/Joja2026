package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Strafe right 3 ft (MOVE_INCHES) and log the pose. See JoJaLocTest. */
@Autonomous(name="JoJa Loc Test 2 Strafe Right 3ft", group="Test")
public class JoJaLocTestStrafe extends JoJaLocTest {
    @Override protected Move move() { return Move.STRAFE_RIGHT; }
}
