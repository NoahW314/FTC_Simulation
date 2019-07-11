package org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction.RobotAction;

public class Transition {
    Double speed;
    Double driveSpeed;
    DistanceMeasure distTillP;
    boolean continuous = true;
    RobotAction action;
    Angle turn;
    Double turnSpeed;
    Boolean turnToRequired;

    private Transition(){}

    public static Transition createDefaultSettings(Double speed, Double driveSpeed, Double turnSpeed, Boolean turnToRequired){
        Transition t = new Transition();
        t.speed = speed;
        t.driveSpeed = driveSpeed;
        t.turnToRequired = turnToRequired;
        t.turnSpeed = turnSpeed;
        return t;
    }
    public static Transition createDefaultSettings(Double speed, Double driveSpeed, Boolean turnToRequired){
        return createDefaultSettings(speed, driveSpeed, 0d, turnToRequired);
    }

    public Transition copy(){
        Transition t = new Transition();
        t.speed = this.speed;
        t.driveSpeed = this.driveSpeed;
        t.distTillP = this.distTillP == null ? null : this.distTillP.copy();
        t.continuous = this.continuous;
        t.turnSpeed = this.turnSpeed;
        t.turnToRequired = this.turnToRequired;
        t.turn = this.turn == null ? null : this.turn.copy();
        t.action = this.action;
        return t;
    }
    @Override
    public String toString(){
        return "Speed: "+speed+"\n"+
                "Turn: "+turn+"\n"+
                "Drive Speed: "+driveSpeed+"\n"+
                "Turn Speed: "+turnSpeed;
    }

    public static class TransitionBuilder{
        private Transition trans;
        private Transition defaultTrans;

        public Transition build(){
            //copy the point to a temporary variable
            Transition temp = trans.copy();
            //reset the point variable so the builder can be reused
            trans = defaultTrans.copy();

            //return the original point
            return temp;
        }

        public TransitionBuilder(Transition defaultSettings){
            defaultTrans = defaultSettings.copy();
            trans = defaultSettings.copy();
        }
        public TransitionBuilder(Double speed, Double driveSpeed, Boolean turnToRequired){
            this(createDefaultSettings(speed, driveSpeed, turnToRequired));
        }


        public TransitionBuilder addSpeed(Double speed){
            this.trans.speed = speed;
            return this;
        }
        public TransitionBuilder addDriveSpeed(Double driveSpeed){
            this.trans.driveSpeed = driveSpeed;
            return this;
        }
        public TransitionBuilder addDistTillP(DistanceMeasure distTillP){
            this.trans.distTillP = distTillP == null ? null : distTillP.copy();//paranoia
            return this;
        }
        public TransitionBuilder addDistTillP(double value, DistanceUnit unit){
            this.trans.distTillP = new DistanceMeasure(value, unit);
            return this;
        }
        public TransitionBuilder addContinuous(boolean continuous){
            this.trans.continuous = continuous;
            return this;
        }
        public TransitionBuilder addTurnSpeed(Double turnSpeed){
            this.trans.turnSpeed = turnSpeed;
            return this;
        }
        public TransitionBuilder addTurn(Angle turn){
            this.trans.turn = turn.copy();
            return this;
        }
        public TransitionBuilder addTurnToRequired(Boolean turnToReq){
            this.trans.turnToRequired = turnToReq;
            return this;
        }
        public TransitionBuilder addAction(RobotAction action){
            this.trans.action = action;
            return this;
        }
    }
}
