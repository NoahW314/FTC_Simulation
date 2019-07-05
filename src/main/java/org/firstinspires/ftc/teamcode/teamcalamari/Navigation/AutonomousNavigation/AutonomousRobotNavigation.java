package org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation;



import static org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation.AutonomousRobotNavigation.RobotStates.ACTION;

import java.util.List;

import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeSim.OpMode;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.Drive;

public class AutonomousRobotNavigation {
    public List<Point> points;
    public List<Transition> transitions;

    /**The drivetrain of the robot*/
    public Drive drive;

    /**The current position of the robot*/
    protected Position position = null;

    /**The current heading of the robot, as measured from the field axes to the robot wheel axes*/
    protected Angle heading = null;

    /**The starting heading of the robot, as measured from the field axes to the robot wheel axes*/
    protected Angle startingHeading = null;

    /**The element number for the array lists*/
    public int i = 0;

    /**The state the robot is currently in*/
    public RobotStates robotState = RobotStates.NONE;

    /**The last heading that was set*/
    public Angle prevSetHeading = null;

    /**Whether or not this is the first time through the runDriving() method this target*/
    protected boolean firstDrive = true;

    /**Whether or not this is the first time through the runAction() method this target*/
    protected boolean firstAction = true;

    /**The OpMode that is running this navigation*/
    private OpMode opMode;

    /**The code that is run after the navigation is finished but before it calls requestOpModeStop.
     If the navigation doesn't finish, some other code calls requestOpModeStop, or the stop button is pushed on the driver station,
     this code will not be run.*/
    public Runnable finalCode;

    /**The state in which, if set, the robot will stay in for the rest of the opmode.*/
    public RobotStates stickState;
    /**The i value at which, if set, when the robot reaches the stickAction it will stay at for the rest of the opmode.*/
    public int stickI = -1;

    public AutonomousRobotNavigation(OpMode opMode, List<Point> points, List<Transition> transitions, Drive drive) {
        this.opMode = opMode;
        this.points = points;
        this.transitions = transitions;
        this.drive = drive;
        if(points.size() != transitions.size()+1) throw new IllegalArgumentException("There should be one more point than there are transitions!");
    }

    /**Initializes some fields*/
    public void initialize(Position pose, Angle heading, boolean firstUpdate) {
        update(pose, heading, firstUpdate);

        prevSetHeading = heading;

        nextState();
    }
    /**Initializes some fields*/
    public void initialize(Position pose, Angle heading){
        initialize(pose, heading, false);
    }

    /**Sets the position*/
    public void updatePosition(Position pose) {position = pose;}
    /**Sets the heading*/
    public void updateHeading(Angle heading) {this.heading = heading;}
    /**Gets the position*/
    public Position getRobotPosition() {return position.copy();}
    /**Gets the heading*/
    public Angle getHeading() {return heading;}
    /**Gets the starting heading*/
    public Angle getStartingHeading(){ return startingHeading; }

    /**Sets the position and the heading*/
    public void update(Position pose, Angle heading) {
        update(pose, heading, false);
    }
    /**Sets the position*/
    public void updatePosition(Position pose, boolean absolute){
        if(absolute){position = pose;}
        else{position = pose.added(points.get(0).target);}
    }
    /**Sets the heading*/
    public void updateHeading(Angle heading, boolean absolute){
        if(absolute){this.heading = heading;}
        else{this.heading = Angle.add(heading, startingHeading);}
    }
    /**Sets the position and the heading*/
    public void update(Position pose, Angle heading, boolean firstUpdate) {
        position = pose;
        this.heading = heading;
        if(firstUpdate){
            points.get(0).target = position.copy();
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
            case NONE:
                if(i == 0 && points.size() != 0){throw new IllegalStateException("AutonomousNavigation.initialize must be called before calling AutonomousNavigation.run");}
                break;
            default:
                break;
        }
    }

    public void runStationaryTurn() {
        if(points.get(i).turn != null) {
            Point p = points.get(i);
            prevSetHeading = p.turn.copy();

            if(p.turnTurnSpeed != null) drive.turnSpeed = p.turnTurnSpeed;
            else throw new NullPointerException("Autonomous Navigation stationary turn turnSpeed cannot be null");

            if(p.turnSpeed != null) drive.speed = p.turnSpeed;
            else throw new NullPointerException("Autonomous Navigation stationary turn speed cannot be null");

            if(heading == null) throw new IllegalStateException("The heading must be updated and AutonomousNavigation.initialize must be called before values of heading can be accessed");


            if(p.angleTillP != null){
                if(drive.turnTo(heading, p.turn, p.angleTillP, p.continuous)){
                    nextState();
                }
            }
            else if(drive.turnTo(heading, p.turn)) {
                nextState();
            }
        }
        else {
            nextState();
        }
    }
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

            if(t.speed != null) drive.speed = t.speed;
            else throw new NullPointerException("Autonomous Navigation speed cannot be null");

            if(t.driveSpeed != null) drive.driveSpeed = t.driveSpeed;
            else throw new NullPointerException("Autonomous Navigation driveSpeed cannot be null");

            if(position == null) throw new NullPointerException("AutonomousNavigation.updatePosition must be called before calling AutonomousNavigation.run");
            if(heading == null) throw new NullPointerException("AutonomousNavigation.updateHeading must be called before calling AutononomousNavigation.run");
            if(points.get(i).target == null) throw new NullPointerException("Autonomous Navigation target cannot be null");


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
    }
    public void runAction() {
        Point p = points.get(i);
        if(p.action != null) {
            if(firstAction) {
                if(!p.action.start()) {
                    if(p.action.act()) {
                        firstAction = true;
                        nextState();
                        return;
                    }
                }
                firstAction = false;
            }
            else {
                if(p.action.act()) {
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
        if(stickState != null && stickI != -1){
            //only move onto the next action if we are not to be stuck here
            if(stickState.equals(robotState) && stickI == i){
                return;
            }
        }
        Point p = null;
        if(robotState == ACTION || robotState == RobotStates.DRIVING || robotState == RobotStates.STATIONARY_TURN){
            if(i >= points.size())
                throw new IllegalStateException("i is greater than or equal to the number of points we have to go to!  We can't get the next point.");
            p = points.get(i);
        }
        switch(robotState) {
            case ACTION:
                if(p.actionFirst) {robotState = RobotStates.STATIONARY_TURN;}
                else {robotState = RobotStates.DRIVING; i++;}
                break;

            case DRIVING:
                if(p.actionFirst) {robotState = ACTION;}
                else {robotState = RobotStates.STATIONARY_TURN;}
                break;

            case NONE:
                if(points.get(0).actionFirst) {robotState = ACTION;}
                else{robotState = RobotStates.STATIONARY_TURN;}
                break;

            case STATIONARY_TURN:
                if(!p.actionFirst) {robotState = ACTION;}
                else {robotState = RobotStates.DRIVING; i++;}
                break;

            case FINISHING:
                robotState = RobotStates.FINISHED;
                break;

            default:
                throw new IllegalArgumentException("There is no next robot state to move to!");
        }
    }

    /**Enum for what states the robot might be in*/
    public enum RobotStates{ACTION, STATIONARY_TURN, DRIVING, FINISHING, FINISHED, NONE}
}
