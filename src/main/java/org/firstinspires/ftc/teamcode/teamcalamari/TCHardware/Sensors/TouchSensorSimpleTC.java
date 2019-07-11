package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.Gamepad;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchSensorSimpleTC extends SensorTemplateSimple<TouchSensor, Boolean> {

	public TouchSensorSimpleTC(TouchSensor sensor) {
		super(sensor);
	}

	@Override
	public Boolean getData() {
		if(linkedWithGamepad) return button.getValue(gamepad);
		return data;
	}
	
	public TouchSensorTC toComplex() {
		return new TouchSensorTC(null);//sensor
	}
	
	public Gamepad gamepad;
	public GamepadButton button;
	public boolean linkedWithGamepad = false;
	public void linkWithGamepad(Gamepad gamepad, GamepadButton button) {
		this.gamepad = gamepad;
		this.button = button;
		linkedWithGamepad = true;
	}
	public void unlinkFromGamepad() {
		this.gamepad = null;
		this.button = null;
		linkedWithGamepad = false;
	}

}
