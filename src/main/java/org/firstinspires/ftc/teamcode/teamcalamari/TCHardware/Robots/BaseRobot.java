package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Robots;

import java.util.HashMap;
import java.util.Iterator;

import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.Gamepad;

public abstract class BaseRobot {
	public Program program;
	public HashMap<String, Boolean> enabledDevices;
	
	public BaseRobot(Program program) {
		this.program = program;
	}
	public void init(HardwareMap hwMap) {
		enabledDevices = BaseRobot.hardwareMapToHashMap(hwMap);
	}

	public abstract void update(Gamepad gamepad1, Gamepad gamepad2);
	public void disable(String hwDevice) {
		enabledDevices.replace(hwDevice, false);
	}
	
	public void stop() {}
	
	/**Classes that extend BaseRobot should create their own list of programs by creating an enum that implements Program.
	A value from that enum can then be passed to the constructor.*/
	public interface Program{
		public OpModeType getOpModeType();
	}
	
	public static HashMap<String, Boolean> hardwareMapToHashMap(HardwareMap hwMap){
		Iterator<String> i = hwMap.iterator();
		HashMap<String, Boolean> hashMap = new HashMap<String, Boolean>(hwMap.size());
		while(i.hasNext()) {
			hashMap.put(i.next(), true);
		}
		
		return hashMap;
	}
}
