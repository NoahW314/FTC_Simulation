package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public interface HardwareDeviceSim extends HardwareDevice {
	void move();
	String log(String deviceName);
}
