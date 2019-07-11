package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad;

import java.lang.reflect.Field;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadButtonClickedEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadButtonEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent.GamepadNumber;

import com.qualcomm.robotcore.hardware.Gamepad;

public enum GamepadButton implements GamepadControl<Boolean> {
    NONE(""),
    DPAD_UP("dpad_up"),
    DPAD_DOWN("dpad_down"),
    DPAD_LEFT("dpad_left"),
    DPAD_RIGHT("dpad_right"),
    A("a"),
    B("b"),
    X("x"),
    Y("y"),
    GUIDE("guide"),
    START("start"),
    BACK("back"),
    LEFT_BUMPER("left_bumper"),
    RIGHT_BUMPER("right_bumper"),
    LEFT_STICK_BUTTON("left_stick_button"),
    RIGHT_STICK_BUTTON("right_stick_button");


    String str;
    GamepadButton(String str){ this.str = str; }

    @Override
    public String getString() { return str; }

    @Override
    public Boolean getValue(Gamepad gamepad){
        if(getString().equals("")){throw new IllegalArgumentException("Can't get the value of NONE on a gamepad");}
        try {
            Field field = Gamepad.class.getField(getString());
            return field.getBoolean(gamepad);
        } catch(ReflectiveOperationException e){
            throw new IllegalStateException("The button "+getString()+" doesn't exist");
        }
    }

    @Override
    public boolean isNone(){
        return getString().equals("");
    }

    @Override
    public GamepadButtonEvent getEvent(GamepadNumber number) {
    	return getPressedEvent(number);
    }
    public GamepadButtonEvent getPressedEvent(GamepadNumber number){
        if(getString().equals("")) throw new IllegalArgumentException("Can't get the gamepad event of NONE");
        return new GamepadButtonEvent(this, true, number);
    }
    public GamepadButtonEvent getReleasedEvent(GamepadNumber number) {
        if(getString().equals("")) throw new IllegalArgumentException("Can't get the gamepad event of NONE");
        return new GamepadButtonEvent(this, false, number);
    }
    public GamepadButtonClickedEvent getClickedEvent(GamepadNumber number, boolean onDown) {
        if(getString().equals("")) throw new IllegalArgumentException("Can't get the gamepad event of NONE");
        return new GamepadButtonClickedEvent(this, onDown, number);
    }
}
