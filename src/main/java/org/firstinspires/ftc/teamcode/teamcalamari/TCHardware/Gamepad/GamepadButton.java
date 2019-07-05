package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadButtonEvent;

import java.lang.reflect.Field;

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
    public GamepadButtonEvent getEvent(){
        Gamepad gamepad = new Gamepad();
        if(getString().equals("")){throw new IllegalArgumentException("Can't set the value of NONE on a gamepad");}
        try {
            Field field = Gamepad.class.getField(getString());
            field.setBoolean(gamepad, true);
        } catch(ReflectiveOperationException e){
            throw new IllegalStateException("The button "+getString()+" doesn't exist");
        }

        return new GamepadButtonEvent(gamepad, this);
    }
}
