package org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**Performs multiple actions at the same time.  Each time the start or act method is called
 * all the actions start or act methods are called in the order they were given.*/
public class RobotActionLinearSynchronous extends RobotActionMulti {

    private List<Boolean> actionCompleted = new ArrayList<>();
    private List<Boolean> startOverridden = new ArrayList<>();

    public RobotActionLinearSynchronous(RobotAction... actions){ this(Arrays.asList(actions)); }
    public RobotActionLinearSynchronous(List<RobotAction> actions) {
        super(actions);
        for (int i = 0; i < actions.size(); i++) {
            actionCompleted.add(false);
            startOverridden.add(false);
        }
    }

    @Override
    public void add(RobotAction action){
        super.add(action);
        actionCompleted.add(false);
        startOverridden.add(false);
    }

    @Override
    public boolean start(){
        for(int i = 0; i < actions.size(); i++){
            actions.get(i).state = RobotActionState.STARTING;
            if(actions.get(i).start()){
                startOverridden.set(i, true);
            }
        }
        return false;
    }

    @Override
    public boolean act() {
        for(int i = 0; i < actions.size(); i++){
            if(!actionCompleted.get(i)) {
                if(!startOverridden.get(i)) {
                    actions.get(i).state = RobotActionState.ACTING;
                    if (actions.get(i).act()) {
                        actions.get(i).state = RobotActionState.STOPPING;
                        actions.get(i).stop();
                        actions.get(i).state = RobotActionState.FINISHED;
                        numCompleted++;
                        actionCompleted.set(i, true);
                    }
                }
                else{startOverridden.set(i, false);}
            }
        }
        return numCompleted >= actionCompleted.size();
    }
}
