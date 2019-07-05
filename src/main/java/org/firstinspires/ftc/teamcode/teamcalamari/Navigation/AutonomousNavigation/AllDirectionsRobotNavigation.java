package org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation;

import java.util.List;

import org.firstinspires.ftc.teamcode.teamcalamari.OpModeSim.OpMode;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.AllDirectionsDrive;

public class AllDirectionsRobotNavigation extends AutonomousRobotNavigation {

	public AllDirectionsRobotNavigation(OpMode opMode, List<Point> points, List<Transition> transitions, AllDirectionsDrive drive) {
	    super(opMode, points, transitions, drive);
	}
	
	@Override
	public void runDriving() {
		if(i >= points.size()) {
			robotState = RobotStates.FINISHING;
		}
		else {
		    Transition t = transitions.get(i-1);

			if(t.action != null) {
				if(firstDrive) {
					if(!t.action.start()) {
						t.action.act();
					}
					firstDrive = false;
				}
				else {
					t.action.act();
				}
			}
			
			if(t.turnSpeed != null) drive.turnSpeed = t.turnSpeed;
			else throw new NullPointerException("Autonomous Navigation turn speed cannot be null");
			if(t.driveSpeed != null) drive.driveSpeed = t.driveSpeed;
			else throw new NullPointerException("Autonomous Navigation drive speed cannot be null");
			if(t.speed != null) drive.speed = t.speed;
			else throw new NullPointerException("Autonomous Navigation speed cannot be null");
			
			if(heading == null) throw new IllegalStateException("The heading must be updated and AutonomousNavigation.initialize must be called before values of heading can be accessed");
			if(prevSetHeading == null) throw new IllegalStateException("AutonomousNavigation.initialize must be called before calling AutonomousNavigation.run");
			if(position == null) throw new NullPointerException("AutonomousNavigation.updatePosition must be called before calling AutonomousNavigation.run");
			if(points.get(i).target == null) throw new NullPointerException("Autonomous Navigation target cannot be null");
			
			if(t.turn == null) {
			    if(t.distTillP != null){
			        if(drive.driveTo(position, points.get(i).target, heading, t.distTillP, t.continuous)){
			            firstDrive = true;
			            nextState();
                    }
                }
			    else if(drive.driveTo(position, points.get(i).target, heading)) {
					firstDrive = true;
					nextState();
				}
			}
			else {
				prevSetHeading = t.turn.copy();
				if(t.turnToRequired) {
					if(((AllDirectionsDrive)drive).turnDriveTo(position, points.get(i).target, heading, prevSetHeading)) {
						nextState();
					}
				}
				else {
					if(((AllDirectionsDrive)drive).driveToWithTurn(position, points.get(i).target, heading, prevSetHeading)) {
						nextState();
					}
				}
			}
		}
	}
	
}
