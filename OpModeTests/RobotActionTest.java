package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeTests;

import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotActionLinear;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotActionSynchronous;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotActionLinearSynchronous;
import org.firstinspires.ftc.teamcode.teamcalamari.AutonomousNavigation.AutonomousNavigation;
import org.firstinspires.ftc.teamcode.teamcalamari.AutonomousNavigation.AutonomousNavigation.RobotStates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotActionTest extends LinearOpMode {


    AutonomousNavigation nav = new AutonomousNavigation(1,0,0,0,0, this);

    RobotAction action1 = new RobotAction() {

    	ElapsedTime timer = new ElapsedTime();
    		
    	@Override
    	public boolean start() {
    		timer.reset();
    		return false;
    	}
    	
        @Override
        public boolean act() {
            System.out.println("Acting...1");
            return timer.seconds() > 1;
        }
    };
    RobotAction action2 = new RobotAction() {

    	ElapsedTime timer = new ElapsedTime();
    		
    	@Override
    	public boolean start() {
    		timer.reset();
    		return false;
    	}
    	
        @Override
        public boolean act() {
            System.out.println("Acting...2");
            return timer.seconds() > 2;
        }
    };
    RobotAction action3 = new RobotAction() {

    	ElapsedTime timer = new ElapsedTime();
    		
    	@Override
    	public boolean start() {
    		timer.reset();
    		return false;
    	}
    	
        @Override
        public boolean act() {
            System.out.println("Acting...3");
            return timer.seconds() > 3;
        }
    };
    
    RobotAction testAction = new RobotActionSynchronous(action1, action2, action3);
    
    @Override
    public void runOpMode() throws InterruptedException {

    	nav.actionFirst.set(0, true);
        nav.actions.set(0, testAction);
    	
        nav.updateHeading(new Angle(0));
        nav.initialize(null);

        while(nav.robotState.equals(RobotStates.ACTION)) {
        	nav.run();
        	Thread.sleep(100);
        }
        
        System.out.println("Finished");

    }
}
