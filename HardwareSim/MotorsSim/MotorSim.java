package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DcMotorControllerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.HardwareDeviceSim;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.Range;

public class MotorSim implements HardwareDeviceSim {
	
	private double internalPower = 0;
	private double power = 0;
	private MotorConfigurationType motorType = new MotorConfigurationType();
	private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;
	private int targetPosition = 0;
	private int currentPosition = 0;
	private RunMode mode = RunMode.RUN_USING_ENCODER;
	private Direction direction = Direction.FORWARD;
	
	public MotorSim() {
		motorType.processAnnotation(UnspecifiedMotor.class.getDeclaredAnnotation(MotorType.class));
	}
	
	public MotorSim(MotorSim m) {
		this();
	}
	 
	 //Hardware Device
	 
	 public Manufacturer getManufacturer() {
	    return Manufacturer.Other;
	}

	 public String getDeviceName() {
	    return "Motor Simulation";
	}

	 public String getConnectionInfo() {
	    return "Connection info does not exist for a simulated motor";
	}
	 
	 public int getVersion() {
		 return 1;
	 }
	 
	 @Override
	public void resetDeviceConfigurationForOpMode() {
		 throw new IllegalStateException("Why are you calling this method which does not exist?");
	 }
	 
	 public void close() {}
	 
	 public void move() {
		 switch(this.getMode()) {
			case RUN_TO_POSITION:
				int reverse = 1;
				if(currentPosition > targetPosition) {reverse = -1;}
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
	
	public void setPowerInternally(double power) {
	    internalPower = power;
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
	    internalSetTargetPosition(position);
	 }
	 
	 protected void internalSetTargetPosition(int position) {
	    targetPosition = position;
	}
	 synchronized public int getTargetPosition() {
	    return targetPosition;
	}
	 synchronized public int getCurrentPosition() {
		 return currentPosition;
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
	  
	  synchronized public void setDirection(Direction direction) {
	    this.direction = direction;
	  }

	  public Direction getDirection() {
	    return direction;
	  }
	 
	 //Motor
	 
	 /**Adds the given power to <code>power</code> which will be used to set the motor power when run() is called*/
		public void setPower(double power) {
			this.power+=power;
		}
		/**Resets <code>power</code> to the given power*/
		public void resetPower(double power) {
			this.power = power;
		}
		/**Returns the power that will currently be set to the motor should run be called.
		getPower() will return the power last set to the motor by run*/
		public double getCurrentPower() {
			return power;
		}
		/**Set the power of the motor*/
		public void run() {
			setPowerInternally(power);
			power = 0;
		}
		
		public void normalizePower() {
			power = Range.clip(power, -1, 1);
		}
		public static void normalizePowers(Motor...motors) {
			//determine the maximum wheel power
	        double maxi = Math.abs(motors[0].getCurrentPower());
	        for(int i = 1; i < motors.length; i++){
	            if(Math.abs(motors[i].getCurrentPower()) > maxi){
	                maxi = Math.abs(motors[i].getCurrentPower());
	            }
	        }

	        /*divide all the motor powers by the maximum power to preserve
	        the ratios between the wheels while keeping the powers under 1*/
	        if(maxi != 0 && maxi > 1){
	            for(int i = 0; i < motors.length; i++){
	                motors[i].resetPower(motors[i].getCurrentPower()/maxi);
	            }
	        }
		}
		
		/**Converts an array of DcMotors to Motors.  Used in the Drive interface.*/
		public static Motor[] DcMotorsToMotors(DcMotor[] dcMotors) {
			Motor[] motors = new Motor[dcMotors.length];
			for(int i = 0; i < dcMotors.length; i++) {
				motors[i] = new Motor(dcMotors[i]);
			}
			return motors;
		}
}
