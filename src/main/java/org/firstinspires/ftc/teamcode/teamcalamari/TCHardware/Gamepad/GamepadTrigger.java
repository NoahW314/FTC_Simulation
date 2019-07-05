package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadMotionDetectedEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadMotionEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadMotionNotDetectedEvent;

import java.lang.reflect.Field;

public enum GamepadTrigger implements GamepadMotion{
    NONE(""),
    LEFT_TRIGGER("left_trigger"),
    RIGHT_TRIGGER("right_trigger");


    String str;
    GamepadTrigger(String str){ this.str = str; }

    @Override
    public String getString() { return str; }

    @Override
    public Float getValue(Gamepad gamepad){
        if(getString().equals("")){throw new IllegalArgumentException("Can't get the value of NONE on a gamepad");}
        try {
            Field field = Gamepad.class.getField(getString());
            return field.getFloat(gamepad);
        } catch(ReflectiveOperationException e){
            throw new IllegalStateException("The trigger "+getString()+" doesn't exist");
        }
    }

    @Override
    public boolean isNone(){
        return getString().equals("");
    }

    @Override
    public GamepadMotionEvent getEvent(){
        return new GamepadMotionEvent(this);
    }
    @Override
    public GamepadMotionDetectedEvent getDetectedEvent(){
        return new GamepadMotionDetectedEvent(this);
    }
    @Override
    public GamepadMotionNotDetectedEvent getNotDetectedEvent(){
        return new GamepadMotionNotDetectedEvent(this);
    }
}
