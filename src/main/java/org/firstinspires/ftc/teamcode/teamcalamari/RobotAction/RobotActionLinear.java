package org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;

import java.util.Arrays;
import java.util.List;

/**Performs multiple in linear order.  It starts with the first running it every time the act method until it finishes.
 * It then moves on to the next action, returning true when all actions have been completed.*/
public class RobotActionLinear extends RobotActionMulti {

    private boolean runStart = false;
    
    public RobotActionLinear(RobotAction...actions) { this(Arrays.asList(actions)); }
    public RobotActionLinear(List<RobotAction> actions){super(actions);}

    @Override
    public boolean start(){
        actions.get(numCompleted).state = RobotActionState.STARTING;
        return actions.get(numCompleted).start();
    }

    @Override
    public boolean act() {
        if(runStart){
            runStart = false;
            if(start()) {
                return false;
            }
        }
        actions.get(numCompleted).state = RobotActionState.ACTING;
        if(actions.get(numCompleted).act()){
            actions.get(numCompleted).state = RobotActionState.STOPPING;
            actions.get(numCompleted).stop();
            actions.get(numCompleted).state = RobotActionState.FINISHED;
            numCompleted++;
            runStart = true;
        }
        return numCompleted >= actions.size();
    }
}
