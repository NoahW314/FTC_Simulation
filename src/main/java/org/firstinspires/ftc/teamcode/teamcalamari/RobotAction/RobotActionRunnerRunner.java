package org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;

/**Runnable for running a {@link RobotActionRunner} in a separate thread.*/
public class RobotActionRunnerRunner implements Runnable {
    private RobotActionRunner runner;
    private boolean abort = false;

    public RobotActionRunnerRunner(RobotActionRunner runner){ this.runner = runner; }

    @Override
    public void run() {
        do{
            runner.run();
        }
        while(!runner.isFinished() && !abort);
        runner.stop();
    }

    /**Used to abort the runner if needed.*/
    public void abort(){
        abort = true;
    }
}
