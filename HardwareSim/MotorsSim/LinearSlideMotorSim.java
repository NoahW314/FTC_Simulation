package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadTrigger;

/**Subclass of Motor used for motors that control linear slides*/
public class LinearSlideMotorSim extends MotorSim {

    public Direction extendDirection = Direction.FORWARD;
    public int fullExtension;
    public int fullRetraction;

	public LinearSlideMotorSim(MotorSim motor) {
		super(motor);
	}
	public LinearSlideMotorSim(MotorSim motor, Direction extendDirection){
	    this(motor);
	    this.extendDirection = extendDirection;
    }

    //extending is positive, retracting is negative
    public double adjustPowerSlide(double power){
        return extendDirection == Direction.FORWARD ? power : -power;
    }
    public void setLimits(int retraction, int extension){
	    fullRetraction = retraction;
	    fullExtension = extension;
    }

    public void manualOverride(GamepadJoystick joystick, double scale){
        if(joystick.isNone()) return;
        double val = joystick.getValue(gamepad);
        if(val == 0) return;
        setMode(RunMode.RUN_USING_ENCODER);
        resetPower(adjustPowerSlide(val*scale));
    }
    public void manualOverride(GamepadJoystick joystick){manualOverride(joystick, 1);}

    public void manualOverrideExtend(GamepadTrigger trigger, double scale){
        if(trigger.isNone()) return;
        double val = trigger.getValue(gamepad);
        if(val == 0) return;
        setMode(RunMode.RUN_USING_ENCODER);
        resetPower(adjustPowerSlide(val*scale));
    }
    public void manualOverrideExtend(GamepadTrigger trigger){manualOverrideExtend(trigger, 1);}
    public void manualOverrideRetract(GamepadTrigger trigger, double scale){
        if(trigger.isNone()) return;
        double val = trigger.getValue(gamepad);
        if(val == 0) return;
        setMode(RunMode.RUN_USING_ENCODER);
        resetPower(-adjustPowerSlide(val*scale));

    }
    public void manualOverrideRetract(GamepadTrigger trigger){manualOverrideRetract(trigger, 1);}

    public void manualOverrideExtend(GamepadButton button, double power){
        if(button.isNone()) return;
        if(button.getValue(gamepad)){
            setMode(RunMode.RUN_USING_ENCODER);
            resetPower(adjustPowerSlide(power));
        }

    }
    public void manualOverrideExtend(GamepadButton button){manualOverrideExtend(button, 1);}
    public void manualOverrideRetract(GamepadButton button, double power){
        if(button.isNone()) return;
        if(button.getValue(gamepad)){
            setMode(RunMode.RUN_USING_ENCODER);
            resetPower(-adjustPowerSlide(power));
        }
    }
    public void manualOverrideRetract(GamepadButton button){manualOverrideRetract(button, 1);}

    public void extendFully(GamepadButton button, double power){
	    if(button.isNone()) return;
	    if(button.getValue(gamepad)) {
            setMode(RunMode.RUN_TO_POSITION);
            setTargetPosition(fullExtension);
        }
        if(getMode() == RunMode.RUN_TO_POSITION && getTargetPosition() == fullExtension) {
            resetPower(power);
        }
    }
    public void extendFully(GamepadButton button){extendFully(button, 1);}
    public void retractFully(GamepadButton button, double power){
	    if(button.isNone()) return;
	    if(button.getValue(gamepad)){
	        setMode(RunMode.RUN_TO_POSITION);
	        setTargetPosition(fullRetraction);
        }
        if(getMode() == RunMode.RUN_TO_POSITION && getTargetPosition() == fullRetraction) {
            resetPower(power);
        }
    }
    public void retractFully(GamepadButton button){retractFully(button, 1);}

    public void retractFully(double power){
	    setMode(RunMode.RUN_TO_POSITION);
	    setTargetPosition(fullRetraction);
	    resetPower(power);
    }
}
