package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadMotion;

public class GamepadMotionNotDetectedEvent extends GamepadMotionEvent {
    public GamepadMotionNotDetectedEvent(GamepadMotion motionControl) {
        super(motionControl, new Range<>(0f, 0f));
    }

}
