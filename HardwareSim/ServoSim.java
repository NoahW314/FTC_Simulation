package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim;

import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.Range;

public class ServoSim extends ServoImpl implements HardwareDeviceSim {
	private double currentPosition = 0;
	private double lastPosition = 0;
	private double targetPosition = 0;

	public ServoSim(ServoController controller, int portNumber) {
		super(controller, portNumber);
		resetDeviceConfigurationForOpMode();
	}
	
	@Override public Manufacturer getManufacturer() {
		return Manufacturer.Other;
	}
	@Override public String getDeviceName() {
		return "Servo Simulation";
	}
	@Override public String getConnectionInfo() {
		return "Connection info does not exist for a simulated servo";
	}
	@Override public void move() {
		lastPosition = currentPosition;
		currentPosition = targetPosition;
	}
	@Override public String log(String deviceName) {
		if(lastPosition != targetPosition) {
			return " moved to the position "+targetPosition;
		}
		else {
			return " did not move";
		}
	}
	
	
	@Override
	protected void internalSetPosition(double position) {
		targetPosition = position;
	}
	@Override
	synchronized public double getPosition() {
		double position = currentPosition;
	    if (direction == Direction.REVERSE) position = reverse(position);
	    double scaled = Range.scale(position, limitPositionMin, limitPositionMax, MIN_POSITION, MAX_POSITION);
	    return Range.clip(scaled, MIN_POSITION, MAX_POSITION);
	}
	private double reverse(double position) {
		return MAX_POSITION - position + MIN_POSITION;
	}
}
