package org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;

/**Base class for actions that the robot will perform.
This class was designed to be used for non-driving actions, though driving actions would also work*/
public abstract class RobotAction {

    public RobotActionState state = RobotActionState.NOT_STARTED;

	/**Run the first time through the robot action.
	@return Returns true if the act method should NOT be called this loop.
    A false value means that the act method will be called this loop.*/
	public boolean start() { return false; }
	
	/**Be sure to set the power of any motors used to zero before saying the action is completed
	@return If the action has finished*/
	public abstract boolean act();

	/**Run after the act method returns true.
    Should be used to perform any needed shutdown logic, such as stopping motors.
    By default does nothing.*/
	public void stop(){}


    public enum RobotActionState{NOT_STARTED, STARTING, ACTING, STOPPING, FINISHED}
}
