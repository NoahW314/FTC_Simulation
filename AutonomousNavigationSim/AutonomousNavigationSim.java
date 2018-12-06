package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.AutonomousNavigationSim;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim.DriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim.OpModeSim;

public class AutonomousNavigationSim {
	/**The targets that we want the robot to move to as it goes through its autonomous routine.
	The first element in this list is the starting position of the robot*/
	public ArrayList<VectorF> targets;
	
	/**The speeds at which the robot moves throughout the routine.
	Corresponds to speed in the drivetrain.
	Element <code>n</code> of <code>speeds</code> is the speed when the robot
	 is moving from element <code>n</code> to element <code>n+1</code> in <code>targets</code>*/
	public ArrayList<Double> speeds;
	
	/**The values for driveSpeed when moving between targets*/
	public ArrayList<Double> driveSpeeds;
	
	/**The angle at which the robot should be before moving on to the next target.
	Values of <code>null</code> indicate that no stationary turn should be performed.
	The robot will turn to element <code>n</code> after reaching element <code>n</code> of <code>targets</code>.*/
	public ArrayList<Angle> stationaryTurns;
	
	/**The speeds at which the robot turns during stationary turns.
	Corresponds to speed in the drivetrain.*/
	public ArrayList<Double> stationaryTurnSpeeds;
	
	/**The values for turnSpeed during stationary turns*/
	public ArrayList<Double> stationaryTurnsTurnSpeeds;
	
	/**Robot actions to take before moving to the next target.
	<code>null</code> values indicate that there is no action to take.
	Element <code>n</code> of <code>actions</code> will be run after reaching element <code>n</code> of targets.
	Whether element <code>n</code> of <code>actions</code> is run before or after the robot has turned to 
	element <code>n</code> of <code>stationaryTurns</code> is determined by element <code>n</code> of <code>actionFirst</code>*/
	public ArrayList<RobotAction> actions;
	
	/**Element <code>n</code> of <code>actionFirst</code> determines whether element <code>n</code> of <code>actions</code> 
	is run before or after the robot has turned to element <code>n</code> of <code>stationaryTurns</code>.
	A true value means that the action will be run first.  A false value means that the turn will be completed first.
	This value will have no effect if the value of <code>actions</code> is <code>null</code>*/
	public ArrayList<Boolean> actionFirst;
	
	/**Robot actions to perform while moving between targets.
	<code>null</code> values indicate that there is no action to take.
	Element <code>n</code> of <code>drivingActions</code> will be run while the robot is
	moving from element <code>n</code> to element <code>n+1</code> of <code>targets</code>.<br><br>
	<strong>IMPORTANT: If the action does not finish before the robot reaches the target, the action will not be completed.</strong>*/
	public ArrayList<RobotAction> drivingActions;
	
	/**The drivetrain of the robot*/
	protected DriveSim drive = null;
	
	/**The current position of the robot*/
	protected VectorF position = null;
	
	/**The current heading of the robot*/
	protected Angle heading = null;
	
	/**The starting heading of the robot, as measured from the field axes to the robot wheel axes*/
	protected Angle startingHeading = null;
	
	/**The element number for the array lists*/
	public int i = 0;
	
	/**The action the robot is currently taking*/
	public RobotStates robotState = RobotStates.NONE;
	
	/**The last heading that was set*/
	public Angle prevSetHeading = null;
	
	/**The number of targets that the robot will drive to.
	Should equal targets.size()*/
	public int targetNumber;
	
	/**Whether or not this is the first time through the runDriving() method this target*/
	protected boolean firstDrive = true;
	
	/**Whether or not this is the first time through the runAction() method this target*/
	protected boolean firstAction = true;
	
	/**The OpMode that is running this navigation*/
	private OpModeSim opMode;
	
	/**The code that is run after the navigation is finished but before it calls requestOpModeStop.
	If the navigation doesn't finish, some other code calls requestOpModeStop, or the stop button is pushed on the driver station,
	this code will not be run.*/
	public Runnable finalCode;
	
	/**The action at which, if set, the robot will stay at for the rest of the opmode.*/
	public RobotStates stickState;
	/**The i value at which, if set, when the robot reaches the stickAction it will stay at for the rest of the opmode.*/
	public int stickI = -1;
	
	public AutonomousNavigationSim(int targetNumber, double dSpeed, double dDriveSpeed, double dStatTurnSpeed, double dStatTurnTurnSpeed, OpModeSim opMode) {
		this.targetNumber = targetNumber;
		this.opMode = opMode;
		
		setDefaultArrayListValues(dSpeed, dDriveSpeed, dStatTurnSpeed, dStatTurnTurnSpeed);
	}
	/**Sets the default values for all the array lists*/
	protected void setDefaultArrayListValues(double dSpeed, double dDriveSpeed, double dStatTurnSpeed, double dStatTurnTurnSpeed) {
		targets = setDefaultArrayListValue(null, targetNumber);
		speeds = setDefaultArrayListValue(dSpeed, targetNumber-1);
		driveSpeeds = setDefaultArrayListValue(dDriveSpeed, targetNumber-1);
		stationaryTurns  = setDefaultArrayListValue(null, targetNumber);
		stationaryTurnSpeeds  = setDefaultArrayListValue(dStatTurnSpeed, targetNumber);
		stationaryTurnsTurnSpeeds = setDefaultArrayListValue(dStatTurnTurnSpeed, targetNumber);
		actions = setDefaultArrayListValue(null, targetNumber);
		actionFirst = setDefaultArrayListValue(false, targetNumber);
		drivingActions = setDefaultArrayListValue(null, targetNumber-1);
	}
	/**Returns an array list filled with <code>targetNum</code> <code>defaultValue</code>s*/
	public static <T> ArrayList<T> setDefaultArrayListValue(T defaultValue, int targetNum) {
		ArrayList<T> list = new ArrayList<T>(targetNum);
		for(int i = 0; i < targetNum; i++) {
			list.add(defaultValue);
		}
		return list;
	}
	
	/**Initializes some fields*/
	public void initialize(DriveSim drive) {
		this.drive = drive;
		
		if(heading == null) throw new IllegalStateException("The heading must be updated before AutonomousNavigation.initialize can be called");
		else prevSetHeading = heading;
		
		nextState();
	}
	
	/**Sets the position*/
	public void updatePosition(VectorF pose) {position = pose;}
	/**Sets the heading*/
	public void updateHeading(Angle heading) {this.heading = heading;}
	/**Gets the position*/
	public VectorF getPosition() {return position;}
	/**Gets the heading*/
	public Angle getHeading() {return heading;}
	
	/**Sets the position and the heading*/
	public void update(VectorF pose, Angle heading) {
		position = pose;
		this.heading = heading;
	}
	
	
	/**Sets the position*/
	public void updatePosition(VectorF pose, boolean absolute){
	    if(absolute){position = pose;}
	    else{position = pose.added(targets.get(0));}
    }
    /**Sets the heading*/
    public void updateHeading(Angle heading, boolean absolute){
	    if(absolute){this.heading = heading;}
	    else{this.heading = Angle.add(heading, startingHeading);}
    }
    /**Sets the position and the heading*/
    public void update(VectorF pose, Angle heading, boolean firstUpdate) {
        position = pose;
        this.heading = heading;
        if(firstUpdate){
            targets.set(0, position);
            startingHeading = heading;
        }
    }
	
	/**Performs the autonomous routine*/
	public void run() {
		switch(robotState){
			case ACTION:
				runAction();
				break;
			case DRIVING:
				runDriving();
				break;
			case STATIONARY_TURN:
				runStationaryTurn();
				break;
			case FINISHING:
				if(finalCode != null) {
					finalCode.run();
				}
				opMode.requestOpModeStop();
				nextState();
				break;
			case FINISHED:
				break;
			default:
				if(i == 0 && targets.size() != 0){throw new IllegalStateException("AutonomousNavigation.initialize must be called before calling AutonomousNavigation.run");}
				break;
		}
	}
		
	public void runStationaryTurn() {
		if(stationaryTurns.get(i) != null) {
			prevSetHeading = stationaryTurns.get(i);
			
			if(stationaryTurnsTurnSpeeds.get(i) != null) drive.turnSpeed = stationaryTurnsTurnSpeeds.get(i);
			else throw new NullPointerException("Autonomous Navigation stationary turn turnSpeed cannot be null");
			
			if(stationaryTurnSpeeds.get(i) != null) drive.speed = stationaryTurnSpeeds.get(i);
			else throw new NullPointerException("Autonomous Navigation stationary turn speed cannot be null");
			
			if(heading == null) throw new IllegalStateException("The heading must be updated and AutonomousNavigation.initialize must be called before values of heading can be accessed");
			
			if(drive.turnTo(heading, stationaryTurns.get(i))) {
				nextState();
			}
		}
		else {
			nextState();
		}
	}
	public void runDriving() {
		if(i >= targets.size()) {
			robotState = RobotStates.FINISHING;
		}
		else {
			if(drivingActions.get(i-1) != null) {
				if(firstDrive) {
					if(!drivingActions.get(i-1).start()) {
						drivingActions.get(i-1).act();
					}
					firstDrive = false;
				}
				else {
					drivingActions.get(i-1).act();
				}
			}
			
			if(speeds.get(i-1) != null) drive.speed = speeds.get(i-1);
			else throw new NullPointerException("Autonomous Navigation speed cannot be null");
			
			if(driveSpeeds.get(i-1) != null) drive.driveSpeed = driveSpeeds.get(i-1);
			else throw new NullPointerException("Autonomous Navigation driveSpeed cannot be null");
			
			if(position == null) throw new NullPointerException("AutonomousNavigation.updatePosition must be called before calling AutonomousNavigation.run");
			if(heading == null) throw new NullPointerException("AutonomousNavigation.updateHeading must be called before calling AutnonomousNavigation.run");
			if(targets.get(i) == null) throw new NullPointerException("Autonomous Navigation target cannot be null");
			
			if(drive.driveTo(position, targets.get(i), heading)) {
				firstDrive = true;
				nextState();
			}
		}
	}
	public void runAction() {
		if(actions.get(i) != null) {
			if(firstAction) {
				if(!actions.get(i).start()) {
					if(actions.get(i).act()) {
						nextState();
					}
				}
				firstAction = false;
			}
			else {
				if(actions.get(i).act()) {
					firstAction = true;
					nextState();
				}
			}
		}
		else {
			nextState();
		}
	}
	
	public void nextState() {
		//only move onto the next state if we are not to be stuck here
		if(stickState != null && stickI != -1) {
			if(stickState.equals(robotState) && stickI == i){
				return;
			}
		}
		
		switch(robotState) {
			case ACTION:
				if(actionFirst.get(i)) {robotState = RobotStates.STATIONARY_TURN;}
				else {robotState = RobotStates.DRIVING; i++;}
				break;
				
			case DRIVING:
				if(actionFirst.get(i)) {robotState = RobotStates.ACTION;}
				else {robotState = RobotStates.STATIONARY_TURN;}
				break;
				
			case NONE:
				if(actionFirst.get(0)) {robotState = RobotStates.ACTION;}
				else{robotState = RobotStates.STATIONARY_TURN;}
				break;
				
			case STATIONARY_TURN:
				if(!actionFirst.get(i)) {robotState = RobotStates.ACTION;}
				else {robotState = RobotStates.DRIVING; i++;}
				break;
				
			case FINISHING:
				robotState = RobotStates.FINISHED;
				break;
				
			default:
				throw new IllegalArgumentException("There is no next robot state to move to!");
		}
	}

	/**Enum for what actions the robot might take*/
	public static enum RobotStates{ACTION, STATIONARY_TURN, DRIVING, FINISHING, FINISHED, NONE}
}
