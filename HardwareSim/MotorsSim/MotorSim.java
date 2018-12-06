package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DcMotorControllerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.HardwareDeviceSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.HardwareMapSim;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Core.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

public class MotorSim extends MotorSimpleSim implements DcMotor {
	
	protected MotorConfigurationType motorType = new MotorConfigurationType();
	protected ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;
	protected int targetPosition = 0;
	protected int currentPosition = 0;
	protected RunMode mode = RunMode.RUN_USING_ENCODER;
	/**The tick offset of the motor at the start.  If set, all encoder ticks (both setting and getting)
    should be using the old system.  So if the startingTicks is 100, and you want the motor to go to a position
    100 ticks away from its position at the start, input 200 as the target position.  If you want the motor to go
    to its position at the start, input 100 as the target position*/
	public int startingTicks = 0;
	
	public MotorSim motorObject;
	
	public MotorSim() {
		motorType.processAnnotation(UnspecifiedMotor.class.getDeclaredAnnotation(MotorType.class));
	}
	
	public MotorSim(MotorSim m) {
		this();
		motorObject = m;
	}
	
	public MotorSim(HardwareMapSim hwMap, String name) {
		this(hwMap.dcMotor.get(name));
		hwMap.dcMotor.replace(name, this, this.motorObject);
	}
	
	//Hardware Device

	@Override
	public String getDeviceName() {
	   return "Motor Simulation";
	}
	 
	@Override
	public void move() {
		switch(this.getMode()) {
			case RUN_TO_POSITION:
				int reverse = 1;
				if(currentPosition > targetPosition) {reverse = -1;}
				if(currentPosition == targetPosition) {reverse = 0;}
				internalPower*=reverse;
				internalMove();
				internalPower*=reverse;
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
		if((tickSpeed = this.getMotorType().getAchieveableMaxTicksPerSecond()/2) != 0) {
			currentPosition+=internalPower*tickSpeed/10;
		} 
		else {
			currentPosition+=internalPower*224;
		}
	}
	 
	@Override
	public String log(String deviceName) {
		DecimalFormat df = new DecimalFormat("#.####");
		df.setRoundingMode(RoundingMode.HALF_UP);
		String log = "";
		switch(this.getMode()) {
			case RUN_TO_POSITION:
				log+=" is running at a speed of "+df.format(internalPower)+" from "+currentPosition+" to "+targetPosition;
				break;
			case RUN_USING_ENCODER:
				log+=" is running at a speed of "+df.format(internalPower);
				break;
			case RUN_WITHOUT_ENCODER:
				log+=" is running at a power of "+df.format(internalPower);
				break;
			case STOP_AND_RESET_ENCODER:
				log+=" is resetting its encoders";
				break;
			default:
				break;
		}
		return log;
	}
	
	 //DcMotor
	 
	 public MotorConfigurationType getMotorType() {
		return motorType;
	}
	 public void setMotorType(MotorConfigurationType motorType) {
		this.motorType = motorType;
	}
	
	 synchronized public double getPower() {
	    double power = internalPower;
	    if (getMode() == RunMode.RUN_TO_POSITION) {
	        power = Math.abs(power);
	    }
	    return power;
	}
	 public boolean isBusy() {
		return internalPower == 0;
	}
	
	 public synchronized void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
	    this.zeroPowerBehavior = zeroPowerBehavior;
	}
	 public synchronized ZeroPowerBehavior getZeroPowerBehavior() {
	    return zeroPowerBehavior;
	}
	
	 public synchronized boolean getPowerFloat() {
	    return getZeroPowerBehavior() == ZeroPowerBehavior.FLOAT && getPower() == 0.0;
	 }
	 
	 synchronized public void setTargetPosition(int position) {
	    internalSetTargetPosition(position-startingTicks);
	 }
	 
	 protected void internalSetTargetPosition(int position) {
	    targetPosition = position;
	}
	 synchronized public int getTargetPosition() {
	    return targetPosition+startingTicks;
	}
	 synchronized public int getCurrentPosition() {
		 return currentPosition+startingTicks;
	}
	
	 synchronized public void setMode(RunMode mode) {
		this.mode = mode;
		if(mode == RunMode.STOP_AND_RESET_ENCODER) {
			currentPosition = 0;
		}
	}
	 public RunMode getMode() {
	    return mode;
	}
	 
	 public DcMotorController getController() {
		 return new DcMotorControllerSim();
	 }
	 
	 public int getPortNumber() {
		 return 0;
	 }
	 
	 public void setPowerFloat() {
		 setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
		 resetPower(0);
		 run();
	 }
	 
	 //Motor
		
	/**Converts an array of DcMotors to Motors.  Used in the Drive interface.*/
	public static Motor[] DcMotorsToMotors(DcMotor[] dcMotors) {
		Motor[] motors = new Motor[dcMotors.length];
		for(int i = 0; i < dcMotors.length; i++) {
			motors[i] = new Motor(dcMotors[i]);
		}
		return motors;
	}
}
