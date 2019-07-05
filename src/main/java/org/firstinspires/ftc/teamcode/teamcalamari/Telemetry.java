package org.firstinspires.ftc.teamcode.teamcalamari;

import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Set;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareDevice;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;

public class Telemetry{
	public Telemetry(OpModeType opModeType) {
		this.opModeType = opModeType;
	}
	
	public boolean logHardware = true;
	public boolean logMotors = true;
	public OpModeType opModeType;
	private String data = "";
	public String getData() {
		return data;
	}
	public HardwareMap hardwareMap;

	public void setHardwareMap(HardwareMap hwMapSim) {
		hardwareMap = hwMapSim;
	}
	
	public void hardwareUpdate(HashMap<com.qualcomm.robotcore.hardware.HardwareDevice, String> deviceMap) {
		if(opModeType == OpModeType.TELE) {
			if(logHardware) {
				logHardware(deviceMap.entrySet());
			}
			moveHardware(deviceMap.keySet());
		}
	}
	
	public void logHardware(Set<Entry<com.qualcomm.robotcore.hardware.HardwareDevice, String>> hardwareMapEntries) {
		for(Entry<com.qualcomm.robotcore.hardware.HardwareDevice, String> entry : hardwareMapEntries) {
			boolean isMotor = (entry.getKey().getClass() == Motor.class);
			if(!isMotor || (isMotor && logMotors)) {
				addData(entry.getValue(), ((HardwareDevice)entry.getKey()).log(entry.getValue()));
			}
		}
	}
	public void moveHardware(Set<com.qualcomm.robotcore.hardware.HardwareDevice> hdSet) {
		for(com.qualcomm.robotcore.hardware.HardwareDevice h : hdSet) {
			((HardwareDevice)h).move();
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
