package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DcMotorController implements com.qualcomm.robotcore.hardware.DcMotorController, HardwareDevice {
	
	private DcMotorSimple motor;
	protected double power = 0;
	protected Direction direction = Direction.FORWARD;
	protected MotorConfigurationType motorType = new MotorConfigurationType();
	protected ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;
	protected int targetPosition = 0;
	protected int currentPosition = 0;
	protected RunMode mode = RunMode.RUN_USING_ENCODER;
	
	public DcMotorController() {}
	public DcMotorController(DcMotorSimple m) {
		motor = m;
	}
	public DcMotorSimple getMotor() {
		return motor;
	}

	

    public void setDirection(int motor, Direction direction) { this.direction = direction;}

    public Direction getDirection(int motor) { return direction; }

	@Override
	public void setMotorType(int motor, MotorConfigurationType motorType) {
		this.motorType = motorType;
	}

	@Override
	public MotorConfigurationType getMotorType(int motor) {
		return motorType;
	}

	@Override
	public void setMotorMode(int motor, RunMode mode) {
		this.mode = mode;
		if(mode == RunMode.STOP_AND_RESET_ENCODER) {
			currentPosition = 0;
		}
	}

	@Override
	public RunMode getMotorMode(int motor) {
		return mode;
	}

	@Override
	public void setMotorPower(int motor, double power) {
		this.power = power;
	}

	@Override
	public double getMotorPower(int motor) {
		double power = this.power;
	    if (getMotorMode(motor) == RunMode.RUN_TO_POSITION) {
	        power = Math.abs(power);
	    }
	    return power;
	}

	@Override
	public boolean isBusy(int motor) {
		return power == 0;
	}

	@Override
	public void setMotorZeroPowerBehavior(int motor, ZeroPowerBehavior zeroPowerBehavior) {
		this.zeroPowerBehavior = zeroPowerBehavior;
	}

	@Override
	public ZeroPowerBehavior getMotorZeroPowerBehavior(int motor) {
		return zeroPowerBehavior;
	}

	@Override
	public boolean getMotorPowerFloat(int motor) {
		return getMotorZeroPowerBehavior(motor) == ZeroPowerBehavior.FLOAT && getMotorPower(motor) == 0.0;
	}

	@Override
	public void setMotorTargetPosition(int motor, int position) {
		targetPosition = position;
	}

	@Override
	public int getMotorTargetPosition(int motor) {
		return targetPosition;
	}

	@Override
	public int getMotorCurrentPosition(int motor) {
		return currentPosition;
	}

	@Override
	public void resetDeviceConfigurationForOpMode(int motor) {
	}
	
	
	@Override
	public void move() {
		switch(this.getMotorMode(0)) {
			case RUN_TO_POSITION:
				int reverse = 1;
				if(currentPosition > targetPosition) {reverse = -1;}
				if(currentPosition == targetPosition) {reverse = 0;}
				power*=reverse;
				internalMove();
				power*=reverse;
				break;
			case RUN_USING_ENCODER:
			case RUN_WITHOUT_ENCODER:
				internalMove();
				break;
			case STOP_AND_RESET_ENCODER:
				currentPosition = 0;
				break;
			default:
				break;
		}
	}
	 
	private void internalMove() {
		//~224 for neverest motors
		//~192 for rev core hex motors
		//~396 for unspecified motor
		 
		double tickSpeed;
		if((tickSpeed = this.getMotorType(0).getAchieveableMaxTicksPerSecond()/2) != 0) {
			currentPosition+=power*tickSpeed/10;
		} 
		else {
			currentPosition+=power*224;
		}
	}
	
	@Override
	public String log(String deviceName) {
		DecimalFormat df = new DecimalFormat("#.####");
		df.setRoundingMode(RoundingMode.HALF_UP);
		String log = "";
		switch(this.getMotorMode(0)) {
			case RUN_TO_POSITION:
				log+=" is running at a speed of "+df.format(power)+" from "+currentPosition+" to "+targetPosition;
				break;
			case RUN_USING_ENCODER:
				log+=" is running at a speed of "+df.format(power);
				break;
			case RUN_WITHOUT_ENCODER:
				log+=" is running at a power of "+df.format(power);
				break;
			case STOP_AND_RESET_ENCODER:
				log+=" is resetting its encoders";
				break;
			default:
				break;
		}
		return log;
	}
	
	@Override
	public Manufacturer getManufacturer() {
		return Manufacturer.Other;
	}

	@Override
	public String getDeviceName() {
		return "Motor Controller Simulation";
	}

	@Override
	public String getConnectionInfo() {
		return "Connection Info doesn't exist for a Simulated Motor Controller";
	}

	@Override
	public int getVersion() {
		return 0;
	}

	@Override
	public void resetDeviceConfigurationForOpMode() {
	}

	@Override
	public void close() {
	}

}
