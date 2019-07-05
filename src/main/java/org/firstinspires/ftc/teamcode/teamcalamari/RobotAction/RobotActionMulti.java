package org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**Base class for robot actions that performs multiple robot actions.
Basically contains a list of actions to perform and the number of actions of completed*/
public abstract class RobotActionMulti extends RobotAction {

    public List<RobotAction> actions = new ArrayList<>();
    public int numCompleted = 0;

    public RobotActionMulti(RobotAction... actions){ this(Arrays.asList(actions)); }
    public RobotActionMulti(List<RobotAction> actions){ this.actions.addAll(actions); }

    public void add(RobotAction action) { this.actions.add(action); }
}
