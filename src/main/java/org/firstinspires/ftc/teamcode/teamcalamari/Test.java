package org.firstinspires.ftc.teamcode.teamcalamari;

import java.util.ArrayList;
import java.util.List;

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
    	
    	/*MotorConfigurationType motorType = new MotorConfigurationType();
    	motorType.processAnnotation(NeveRest60Gearmotor.class.getDeclaredAnnotation(MotorType.class));
    	motorType.processAnnotation(RevRoboticsCoreHexMotor.class.getDeclaredAnnotation(MotorType.class));
    	System.out.println(motorType.getTicksPerRev());
    	System.out.println(motorType.getGearing());
    	System.out.println(motorType.getMaxRPM());*/
    	
    	
    	List<Range<Boolean>> ranges = new ArrayList<>();
    	ranges.add(new Range<Boolean>(Boolean.FALSE, Boolean.TRUE));
    	ranges.add(new Range<Boolean>(Boolean.TRUE, Boolean.FALSE));
    	ranges.add(new Range<Boolean>(Boolean.FALSE, Boolean.FALSE));
    	ranges.add(new Range<Boolean>(Boolean.TRUE, Boolean.TRUE));
    	Boolean[] booleans = new Boolean[] {Boolean.FALSE, Boolean.TRUE};
    	
    	for(Range<Boolean> r : ranges) {
    		for(Boolean b : booleans) {
    			System.out.println(r.toString()+"  "+b);
    			System.out.println(r.contains(b));
    			System.out.println();
    		}
    	}
    }
}