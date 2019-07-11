package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**Wrapper class for DcMotors*/
public class Motor extends MotorSimple implements DcMotor {
	/**The tick offset of the motor at the start.  If set, all encoder ticks (both setting and getting)
    should be using the old system.  So if the startingTicks is 100, and you want the motor to go to a position
    100 ticks away from its position at the start, input 200 as the target position.  If you want the motor to go
    to its position at the start, input 100 as the target position*/
	public int startingTicks = 0;
	
	public Motor() {super();}
	public Motor(DcMotor motor) {
        super(motor);
        controller.setMotorZeroPowerBehavior(0, DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public Motor(String motorName, HardwareMap hwMap){
	    this(motorName, hwMap, false);
    }
    public Motor(String motorName, HardwareMap hwMap, boolean usingEncoders){
        super(motorName, hwMap, MotorSimple.class);
	    if(usingEncoders){
	        controller.setMotorMode(0, RunMode.STOP_AND_RESET_ENCODER);
	        controller.setMotorMode(0, RunMode.RUN_USING_ENCODER);
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

    @Override
    public MotorConfigurationType getMotorType() { return controller.getMotorType(0); }

    @Override
    public void setMotorType(MotorConfigurationType motorType) { controller.setMotorType(0, motorType);}

    @Override
    public DcMotorController getController() { return controller; }

    @Override
    public int getPortNumber() { return 0; }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) { controller.setMotorZeroPowerBehavior(0, zeroPowerBehavior);}

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() { return controller.getMotorZeroPowerBehavior(0); }

    @Override
    @Deprecated
    public void setPowerFloat() { controller.setMotorPower(0, 0); controller.setMotorZeroPowerBehavior(0, ZeroPowerBehavior.FLOAT);}

    @Override
    public boolean getPowerFloat() { return controller.getMotorPowerFloat(0);}

    @Override
    public void setTargetPosition(int position) { controller.setMotorTargetPosition(0, position-startingTicks);}

    @Override
    public int getTargetPosition() { return controller.getMotorTargetPosition(0)+startingTicks; }

    @Override
    public boolean isBusy() { return controller.isBusy(0); }

    @Override
    public int getCurrentPosition() { return controller.getMotorCurrentPosition(0)+startingTicks; }

    @Override
    public void setMode(RunMode mode) { controller.setMotorMode(0, mode);}

    @Override
    public RunMode getMode() { return controller.getMotorMode(0); }
    
    //Hardware Device

  	@Override
  	public String getDeviceName() {
  	   return "Motor Simulation";
  	}
  	 
  	@Override
  	public void move() {
  		controller.move();
  	}
  	 
  	@Override
  	public String log(String deviceName) {
  		return controller.log(deviceName);
  	}
    
}