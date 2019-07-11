package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent.GamepadNumber;

public interface GamepadControl<T extends Comparable<T>> {
    String getString();
    T getValue(Gamepad gamepad);
    boolean isNone();
    GamepadEvent<T, ?> getEvent(GamepadNumber number);
    
    public static <V extends Comparable<V>> void setValue(Gamepad gamepad, GamepadControl<V> control, V value) {
        if(control.getString().equals("")){throw new IllegalArgumentException("Can't set the value of NONE on a gamepad");}
    	try {
			Field field = Gamepad.class.getField(control.getString());
			field.set(gamepad, value);
		} catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e) {
			e.printStackTrace();
		}
    }
    public static <V extends Comparable<V>> Gamepad setValue(GamepadControl<V> control, V value) {
    	Gamepad gamepad = new Gamepad();
        if(control.getString().equals("")){throw new IllegalArgumentException("Can't set the value of NONE on a gamepad");}
    	try {
			Field field = Gamepad.class.getField(control.getString());
			field.set(gamepad, value);
		} catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e) {
			e.printStackTrace();
		}
    	return gamepad;
    }
}
