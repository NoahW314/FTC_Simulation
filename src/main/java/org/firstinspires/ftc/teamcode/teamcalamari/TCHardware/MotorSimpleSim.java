package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.Gamepad;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class MotorSimpleSim implements HardwareDevice, DcMotorSimple {
    protected Gamepad gamepad;
    protected double internalPower = 0;
	protected double power = 0;
	protected Direction direction = Direction.FORWARD;
	
	public MotorSimpleSim motorObject;

    public MotorSimpleSim(){}
    public MotorSimpleSim(MotorSimpleSim m) {
		this();
		motorObject = m;
	}
    public <T extends MotorSimpleSim> MotorSimpleSim(String motorName, HardwareMap hwMap, Class<T> clazz){
        this(hwMap.get(clazz, motorName));
        hwMap.replace(motorName, this, this.motorObject);
    }

    public void setGamepad(Gamepad gamepad){
        this.gamepad = gamepad;
    }


    @Override
    public void setDirection(Direction direction) { this.direction = direction;}

    @Override
    public Direction getDirection() { return direction; }
    
    public void setPowerInternally(double power) {
	    internalPower = power;
	}

    @Override
    public double getPower(){
	    return internalPower;
	}

    /**Adds the given power to <code>power</code> which will be used to set the motor power when run() is called*/
    @Override
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

    /**Stop the motor*/
    public void stop() {
        this.resetPower(0);
        this.run();
    }

    /**Clips the power to a range of (-1, 1)*/
    public void normalizePower() { power = Range.clip(power, -1, 1); }
    public static void normalizePowers(MotorSimpleSim...motors) {
        //determine the maximum wheel power
        double maxi = Math.abs(motors[0].getCurrentPower());
        for (int i = 1; i < motors.length; i++) {
            if (Math.abs(motors[i].getCurrentPower()) > maxi) {
                maxi = Math.abs(motors[i].getCurrentPower());
            }
        }

        /*divide all the motor powers by the maximum power to preserve
        the ratios between the wheels while keeping the powers under 1*/
        if (maxi != 0 && maxi > 1) {
            for (MotorSimpleSim motor : motors) {
                motor.resetPower(motor.getCurrentPower() / maxi);
            }
        }
    }


    //Hardware Device interface

    @Override
    public Manufacturer getManufacturer() { return Manufacturer.Other; }

    @Override
    public String getDeviceName() { return "Motor Simple Simulation"; }

    @Override
    public String getConnectionInfo() { return "Connection info does not exist for a simulated motor"; }

    @Override
    public int getVersion() { return 1; }

    @Override
    public void resetDeviceConfigurationForOpMode() { 
    	//TODO: is this really the right approach?  Someone might actually want to call this method
    	throw new IllegalStateException("Why are you calling this method which does not exist?");
    }

    @Override
    public void close() {}
    
	@Override
	public void move() {}
	
	@Override
	public String log(String deviceName) {
		DecimalFormat df = new DecimalFormat("#.####");
		df.setRoundingMode(RoundingMode.HALF_UP);
		String log = " is running at a power of "+df.format(internalPower);
		return log;
	}
}
