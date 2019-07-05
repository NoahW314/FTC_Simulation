package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadTrigger;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SwifferMotor extends MotorSimple {

	/**The direction the motor should run to sweep things in*/
	public Direction inDirection = Direction.FORWARD;

	private boolean inOn = false;
	private boolean outOn = false;

	public ElapsedTime timer = new ElapsedTime();
	private boolean inTime = false;
	private boolean outTime = false;
	public double runTime = 0;

	public SwifferMotor(MotorSimple motor) {
		super(motor);
	}
	public SwifferMotor(MotorSimple motor, Direction inDirection) {
		this(motor);
		this.inDirection = inDirection;
	}
	public SwifferMotor(String motorName, HardwareMap hwMap, Class<? extends MotorSimple> clazz, Direction inDirection) {
		super(motorName, hwMap, clazz);
		this.inDirection = inDirection;
	}

    /**Run the motor so as to pull things in*/
	public void runIn(double power) {
		power = Math.abs(power);
		setPower(inDirection == Direction.FORWARD ? power : -power);
	}
	/**Run the motor so as to pull things in*/
	public void runIn() {
		this.runIn(1);
	}
	
	/**Run the motor so as to push things out*/
	public void runOut(double power) {
		power = Math.abs(power);
		setPower(inDirection == Direction.FORWARD ? -power : power);
	}
	/**Run the motor so as to push things out*/
	public void runOut() {
		this.runOut(1);
	}

	public void turnOnIn(GamepadButton button, double power){
	    if(button.isNone())return;
	    if(button.getValue(gamepad)){
	        inOn = true;
	        outOn = false;
	    }
        if(inOn){ runIn(power); }
    }
    public void turnOnIn(GamepadButton button){turnOnIn(button, 1);}
    public void turnOnOut(GamepadButton button, double power){
	    if(button.isNone())return;
	    if(button.getValue(gamepad)){
	        outOn = true;
	        inOn = false;
	    }
	    if(outOn){ runOut(power); }
    }
    public void turnOnOut(GamepadButton button){turnOnOut(button, 1);}
    public void turnOff(GamepadButton button){
	    if(button.isNone())return;
	    if(button.getValue(gamepad)){
	        outOn = false;
	        inOn = false;
	        resetPower(0);
        }
    }

    public void runIn(GamepadButton button, double power){
	    if(button.isNone())return;
	    if(button.getValue(gamepad)) runIn(power);
    }
    public void runIn(GamepadButton button){runIn(button, 1);}
    public void runOut(GamepadButton button, double power){
	    if(button.isNone())return;
	    if(button.getValue(gamepad)) runOut(power);
    }
    public void runOut(GamepadButton button){runOut(button, 1);}

    public void runInTime(GamepadButton button, double power, double time){
	    if(button.isNone()) return;
	    if(button.getValue(gamepad) && !inTime){
	        inTime = true;
	        outTime = false;
	        runTime = time;
	        timer.reset();
        }
	    if(inTime && time == runTime) {
	    	if(timer.seconds() < time) { runIn(power); }
	    	else {inTime = false;}
	    }
    }
    public void runInTime(GamepadButton button, double time){runInTime(button, 1, time);}
    public void runOutTime(GamepadButton button, double power, double time){
	    if(button.isNone()) return;
	    if(button.getValue(gamepad) && !outTime){
	        outTime = true;
	        inTime = false;
	        runTime = time;
	        timer.reset();
        }
	    if(outTime && time == runTime) {
	    	if(timer.seconds() < time) { runOut(power); }
	    	else {outTime = false;}
	    }
    }
    public void runOutTime(GamepadButton button, double time){runOutTime(button, 1, time);}

    public void runIn(GamepadTrigger trigger){ runIn(trigger, 1); }
    public void runIn(GamepadTrigger trigger, double scale){
	    if(trigger.isNone()) return;
	    runIn(trigger.getValue(gamepad)*scale);
    }
    public void runOut(GamepadTrigger trigger){runOut(trigger, 1);}
    public void runOut(GamepadTrigger trigger, double scale){
	    if(trigger.isNone()) return;
	    runOut(trigger.getValue(gamepad)*scale);
    }

    public void run(GamepadJoystick joystick, double scale){
    	if(joystick.isNone()) return;
	    double val = joystick.getValue(gamepad)*scale;
	    setPower(inDirection == Direction.FORWARD ? val : -val);
    }
    public void run(GamepadJoystick joystick){run(joystick, 1);}
}
