package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.States;

import org.firstinspires.ftc.robotcore.external.function.Supplier;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction.RobotActionMotor;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;

public class MotorState extends StateMachine.StateBase {
        //object to run the robot action
        private RobotAction action;

        public MotorState(String name, RobotActionMotor action, StateMachine.StateTransition... transitions){
            super(name, transitions);
            this.action = action;
        }
        public MotorState(String name, Motor m, Supplier<Double> supplier, StateMachine.StateTransition... transitions){
            this(name, new RobotActionMotor(m, supplier), transitions);
        }
        
        public void run(){
            if(action == null) return;
            action.act();
        }
        public void reset(){}
    }