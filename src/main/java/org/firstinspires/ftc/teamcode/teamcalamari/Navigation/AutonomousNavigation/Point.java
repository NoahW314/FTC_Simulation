package org.firstinspires.ftc.teamcode.teamcalamari.Navigation.AutonomousNavigation;

import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction.RobotAction;

public class Point {
    Position target;
    Angle turn;
    Double turnSpeed;
    Double turnTurnSpeed;
    Angle angleTillP;
    boolean continuous = true;
    RobotAction action;
    Boolean actionFirst;

    private Point(){}

    public static Point createDefaultSettings(Double turnSpeed, Double turnTurnSpeed, Boolean actionFirst,
                                              Angle angleTillP, boolean continuous){
        Point p = new Point();
        p.actionFirst = actionFirst;
        p.turnSpeed = turnSpeed;
        p.turnTurnSpeed = turnTurnSpeed;
        p.angleTillP = angleTillP == null ? null : angleTillP.copy();//paranoia
        p.continuous = continuous;
        return p;
    }
    public static Point createDefaultSettings(Double turnSpeed, Boolean actionFirst, Angle angleTillP, boolean continuous){
        return createDefaultSettings(turnSpeed, 0d, actionFirst, angleTillP, continuous);
    }
    public static Point createDefaultSettings(Double turnSpeed, Double turnTurnSpeed, Boolean actionFirst){
        return createDefaultSettings(turnSpeed, turnTurnSpeed, actionFirst, null, false);
    }

    public Point copy(){
        Point p = new Point();
        p.target = this.target;
        p.turn = this.turn == null ? null : this.turn.copy();
        p.turnSpeed = this.turnSpeed;
        p.turnTurnSpeed = this.turnTurnSpeed;
        p.action = this.action;
        p.actionFirst = this.actionFirst;
        p.angleTillP = this.angleTillP == null ? null : this.angleTillP.copy();
        return p;
    }
    @Override
    public String toString(){
        return "Target: "+target+"\n"+
                "Turn: "+turn+"\n";
    }

    public static class PointBuilder{
        private Point point;
        private Point defaultPoint;

        public Point build(Position target){
            //set the target point
            point.target = target;
            //copy the point to a temporary variable
            Point temp = point.copy();
            //reset the point variable so the builder can be reused
            point = defaultPoint.copy();

            //return the original point
            return temp;
        }

        public PointBuilder(Point defaultSettings){
            defaultPoint = defaultSettings.copy();
            point = defaultSettings.copy();
        }
        public PointBuilder(Double turnSpeed, Double turnTurnSpeed, Boolean actionFirst,
                            Angle angleTillP, boolean continuous){
            this(createDefaultSettings(turnSpeed, turnTurnSpeed, actionFirst, angleTillP, continuous));
        }
        public PointBuilder(Double turnSpeed, Double turnTurnSpeed, Boolean actionFirst){
            this(createDefaultSettings(turnSpeed, turnTurnSpeed, actionFirst));
        }
        public PointBuilder(Double turnSpeed, Boolean actionFirst, Angle angleTillP, boolean continuous){
            this(createDefaultSettings(turnSpeed, actionFirst, angleTillP, continuous));
        }


        public PointBuilder addTurn(Angle turn){
            this.point.turn = turn.copy();
            return this;
        }
        public PointBuilder addTurnSpeed(Double turnSpeed) {
            this.point.turnSpeed = turnSpeed;
            return this;
        }
        public PointBuilder addTurnTurnSpeed(Double turnTurnSpeed){
            this.point.turnTurnSpeed = turnTurnSpeed;
            return this;
        }
        public PointBuilder addAngleTillP(Angle angleTillP){
            this.point.angleTillP = angleTillP;
            return this;
        }
        public PointBuilder addContinuous(boolean continuous){
            this.point.continuous = continuous;
            return this;
        }
        public PointBuilder addAction(RobotAction action){
            this.point.action = action;
            return this;
        }
        public PointBuilder addActionFirst(Boolean actionFirst){
            this.point.actionFirst = actionFirst;
            return this;
        }
    }
}
