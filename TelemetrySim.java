package org.firstinspires.ftc.teamcode.teamcalamari.Simulation;

import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Set;

import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.HardwareDeviceSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.HardwareMapSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;

public class TelemetrySim{
	public TelemetrySim(OpModeType opModeType) {
		this.opModeType = opModeType;
	}
	
	public boolean logHardware = true;
	public boolean logMotors = true;
	public OpModeType opModeType;
	private String data = "";
	public String getData() {
		return data;
	}
	public HardwareMapSim hardwareMap;

	public void setHardwareMap(HardwareMapSim hwMapSim) {
		hardwareMap = hwMapSim;
	}
	
	public void hardwareUpdate(HashMap<HardwareDeviceSim, String> deviceMap) {
		if(opModeType == OpModeType.TELE) {
			if(logHardware) {
				logHardware(deviceMap.entrySet());
			}
			moveHardware(deviceMap.keySet());
		}
	}
	
	public void logHardware(Set<Entry<HardwareDeviceSim, String>> hardwareMapEntries) {
		for(Entry<HardwareDeviceSim, String> entry : hardwareMapEntries) {
			boolean isMotor = (entry.getKey().getClass() == MotorSim.class);
			if(!isMotor || (isMotor && logMotors)) {
				addData(entry.getValue(), entry.getKey().log(entry.getValue()));
			}
		}
	}
	public void moveHardware(Set<HardwareDeviceSim> hdSet) {
		for(HardwareDeviceSim h : hdSet) {
			h.move();
		}
	}
	
	public void addData(String str, Object value) {
		data+=str+": "+value.toString()+"\n";
	}
	
	public void update() {
		if(hardwareMap != null) {
			hardwareUpdate(hardwareMap.deviceMap);
		}
		if(!data.equals("")) {
			System.out.println(formatTelemetryData(data));
		}
		data = "";
	}
	
	private String formatTelemetryData(String str) {
		return str;
	}

	public void logHardware(boolean b) {
		logHardware = b;
	}

	public void logMotors(boolean b) {
		logMotors = b;
	}

	public void setAutoClear(boolean b) {
		//does nothing since data isn't cleared away automatically
	}

}
