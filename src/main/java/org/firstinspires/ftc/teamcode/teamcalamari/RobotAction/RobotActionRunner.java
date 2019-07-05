package org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;

/**Runs a robot action outside of an autonomous routine*/
public class RobotActionRunner {

    private boolean firstCall = true;
    public RobotAction action;
    private boolean finished = false;

    public RobotActionRunner(RobotAction robotAction){
        action = robotAction;
    }
    public RobotActionRunner(RobotActionRunner runner){action = runner.action;}

    /**Runs the robot action.  If the action has finished then it will not be run, unless the force parameter is set to true
     @param force set to true to force the action to run, even if it has finished running
     @return if the action has finished*/
    public boolean run(boolean force){
        //exit the method if the action has finished, unless we are forcing the action to run
        if(isFinished() && !force) return true;

        //if this is the first time running this action call the start method
        if(firstCall){
            firstCall = false;
            //run the start method
            //if it returns true, exit the method, returning false (since the action has not finished)
            //otherwise continue and call the act method
            action.state = RobotAction.RobotActionState.STARTING;
            if(action.start()) {
                return false;
            }
        }

        //run the action
        action.state = RobotAction.RobotActionState.ACTING;
        if(action.act()) {
            stop();
        }

        return isFinished();
    }

    /**Runs the robot action.  If the action has finished, then it will not be run.
     @see RobotActionRunner#run(boolean) */
    public boolean run(){
        return run(false);
    }

    /**Stop the running of the action and call the action's stop method
     * Future calls to the run method will not run the action, unless the force parameter is set to true.
     * If stop has been called and then run called with the force parameter set to true,
     * calling stop again will cause the action's stop method to be run again.*/
    public void stop(){
        action.state = RobotAction.RobotActionState.STOPPING;
        action.stop();
        action.state = RobotAction.RobotActionState.FINISHED;
        finished = true;
    }

    public boolean isFinished(){return finished;}
}
