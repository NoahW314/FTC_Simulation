package org.firstinspires.ftc.teamcode.teamcalamari.OpModeTests;

import org.firstinspires.ftc.teamcode.teamcalamari.OpModeSim.OpMode;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.TouchSensorTC;

public class GamepadTouchSensorLinkTest extends OpMode {
	
	TouchSensorTC sensor;
	
	public void init() {
		sensor = new TouchSensorTC(hardwareMap.touchSensor.get("touch"));
		sensor.linkWithGamepad(gamepad1, GamepadButton.BACK);
	}
	
	public void loop() {
		telemetry.addData("Back Button", gamepad1.back);
		telemetry.addData("Touch Sensor", sensor.getData() == 1.0);
	}

}
