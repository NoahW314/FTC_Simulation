package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.function.Supplier;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadButtonEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent.GamepadNumber;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadMotionEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadTrigger;

import com.qualcomm.robotcore.hardware.Gamepad;

public class StateMachineUtils {
    public static<T> Supplier<T> getSupplier(final T value){
        return new Supplier<T>(){
            @Override
            public T get(){
                return value;
            }
        };
    }

    public static List<GamepadEvent<?, ?>> generateGamepadEvents(Gamepad gamepad){
        return generateGamepadEvents(gamepad, GamepadNumber.ANY);
    }
    public static List<GamepadEvent<?, ?>> generateGamepadEvents(Gamepad gamepad, GamepadNumber number){
        List<GamepadEvent<?, ?>> events = new ArrayList<>();
        for(GamepadButton button : GamepadButton.values()){
        	if(button == GamepadButton.NONE) continue;
            events.add(new GamepadButtonEvent(gamepad, button, number));
        }
        for(GamepadJoystick joystick : GamepadJoystick.values()){
        	if(joystick == GamepadJoystick.NONE) continue;
            events.add(new GamepadMotionEvent(gamepad, joystick, number));
        }
        for(GamepadTrigger trigger : GamepadTrigger.values()){
        	if(trigger == GamepadTrigger.NONE) continue;
            events.add(new GamepadMotionEvent(gamepad, trigger, number));
        }
        return events;
    }
}
