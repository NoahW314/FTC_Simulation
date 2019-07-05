package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.function.Supplier;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadButtonEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadMotionEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadTrigger;

import java.util.ArrayList;
import java.util.List;

public class StateMachineUtils {
    public static<T> Supplier<T> getSupplier(final T value){
        return new Supplier<T>(){
            @Override
            public T get(){
                return value;
            }
        };
    }

    public static List<GamepadEvent> generateGamepadEvents(Gamepad gamepad){
        List<GamepadEvent> events = new ArrayList<>();
        for(GamepadButton button : GamepadButton.values()){
            events.add(new GamepadButtonEvent(gamepad, button));
        }
        for(GamepadJoystick joystick : GamepadJoystick.values()){
            events.add(new GamepadMotionEvent(gamepad, joystick));
        }
        for(GamepadTrigger trigger : GamepadTrigger.values()){
            events.add(new GamepadMotionEvent(gamepad, trigger));
        }
        return events;
    }
}
