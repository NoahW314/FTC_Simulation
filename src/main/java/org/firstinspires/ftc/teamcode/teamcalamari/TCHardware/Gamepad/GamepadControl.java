package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent;

public interface GamepadControl<T> {
    String getString();
    T getValue(Gamepad gamepad);
    boolean isNone();
    GamepadEvent<T> getEvent();
}
