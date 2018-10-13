package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.RobotsSim;

import java.util.HashMap;
import java.util.Iterator;

import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.GamepadSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.HardwareMapSim;

public abstract class BaseRobotSim {
	public Program program;
	public HashMap<String, Boolean> enabledDevices;
	
	public BaseRobotSim(Program program) {
		this.program = program;
	}
	public void init(HardwareMapSim hwMap) {
		enabledDevices = BaseRobotSim.hardwareMapToHashMap(hwMap);
	}

	public abstract void update(GamepadSim gamepad1, GamepadSim gamepad2);
	public void disable(String hwDevice) {
		enabledDevices.replace(hwDevice, false);
	}
	
	/**Classes that extend BaseRobot should create their own list of programs by creating an enum that implements Program.
	A value from that enum can then be passed to the constructor.*/
	public interface Program{
		public OpModeType getOpModeType();
	}
	
	public static HashMap<String, Boolean> hardwareMapToHashMap(HardwareMapSim hwMap){
		Iterator<String> i = hwMap.iterator();
		HashMap<String, Boolean> hashMap = new HashMap<String, Boolean>(hwMap.size());
		while(i.hasNext()) {
			hashMap.put(i.next(), true);
		}
		
		return hashMap;
	}
}
