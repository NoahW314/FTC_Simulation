package org.firstinspires.ftc.teamcode.teamcalamari;

import com.qualcomm.hardware.motors.NeveRest60Gearmotor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

class Test implements Runnable {

    volatile boolean keepRunning = true;

    public void run() {
        System.out.println("Starting to loop.");
        while (keepRunning) {
            System.out.println("Running loop...");
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
            }
        }
        System.out.println("Done looping.");
    }

    public static void main(String[] args) {

        /*Test test = new Test();
        Thread t = new Thread(test);
        t.start();

        Scanner s = new Scanner(System.in);
        while (!s.hasNextLine()) {
        	
        }

        test.keepRunning = false;
        t.interrupt();  // cancel current sleep.*/
    	
    	MotorConfigurationType motorType = new MotorConfigurationType();
    	motorType.processAnnotation(NeveRest60Gearmotor.class.getDeclaredAnnotation(MotorType.class));
    	motorType.processAnnotation(RevRoboticsCoreHexMotor.class.getDeclaredAnnotation(MotorType.class));
    	System.out.println(motorType.getTicksPerRev());
    	System.out.println(motorType.getGearing());
    	System.out.println(motorType.getMaxRPM());
    }
}