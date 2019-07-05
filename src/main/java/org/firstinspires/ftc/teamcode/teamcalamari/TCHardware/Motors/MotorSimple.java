package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DcMotorController;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareDevice;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.Gamepad;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class MotorSimple implements HardwareDevice, DcMotorSimple {
    protected Gamepad gamepad;
    protected double power;
	
	protected DcMotorController controller;

    public MotorSimple(){}
    public MotorSimple(DcMotorSimple m) {
		this();
		controller = new DcMotorController(m);
	}
    public <T extends DcMotorSimple> MotorSimple(String motorName, HardwareMap hwMap, Class<T> clazz){
        this(hwMap.get(clazz, motorName));
        hwMap.replace(motorName, this, this.controller.getMotor());
    }

    public void setGamepad(Gamepad gamepad){
        this.gamepad = gamepad;
    }


    @Override
    public void setDirection(Direction direction) { controller.setDirection(0, direction);}

    @Override
    public Direction getDirection() { return controller.getDirection(0); }
    
    public void setPowerInternally(double power) {
	    controller.setMotorPower(0, power);
	}

    @Override
    public double getPower(){
	    return controller.getMotorPower(0);
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
    @Deprecated
    public double getCurrentPower() {
        return power;
    }
    public double getNextPower() {
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

    /**Clips the power to a range of (-{@code speed}, {@code speed})*/
    public void normalizePower(double speed) {
        speed = Range.clip(speed, -1, 1);//paranoia
        power = Range.clip(power, -speed, speed);
    }
    public void normalizePower(){normalizePower(1);}
    public static void normalizePowers(double speed, MotorSimple...motors) {
        //determine the maximum wheel power
        double maxi = Math.abs(motors[0].getNextPower());
        for (int i = 1; i < motors.length; i++) {
            if (Math.abs(motors[i].getNextPower()) > maxi) {
                maxi = Math.abs(motors[i].getNextPower());
            }
        }

        /*divide all the motor powers by the maximum power to preserve
        the ratios between the wheels while keeping the powers under 1*/
        if (maxi != 0 && maxi > 1) {
            for (MotorSimple motor : motors) {
                motor.resetPower(motor.getNextPower()*speed/maxi);
            }
        }
    }
    public static void normalizePowers(MotorSimple... motors){normalizePowers(1, motors);}


    //Hardware Device interface

    @Override
    public Manufacturer getManufacturer() { return Manufacturer.Other; }

    @Override
    public String getDeviceName() { return "Motor Simple Simulation"; }

    @Override
    public String getConnectionInfo() { return "Connection info does not exist for a Simulated motor"; }

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
		String log = " is running at a power of "+df.format(controller.getMotorPower(0));
		return log;
	}
}
