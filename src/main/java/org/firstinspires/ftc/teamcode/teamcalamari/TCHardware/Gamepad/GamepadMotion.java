package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadMotionDetectedEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadMotionNotDetectedEvent;

public interface GamepadMotion extends GamepadControl<Float> {
    GamepadMotionDetectedEvent getDetectedEvent();
    GamepadMotionNotDetectedEvent getNotDetectedEvent();
}
