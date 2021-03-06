package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;

public class StateMachine {
    /**Map of the name of a state to that state*/
    private Map<String, State> states = new HashMap<>();
    private State currentState;

    public State getCurrentState() {return currentState;}
    
    public StateMachine(State... states){
        for(State state : states) {
            this.states.put(state.getName(), state);
        }
    }

    public void addStates(State... states){
        for(State state : states){
            this.states.put(state.getName(), state);
        }
    }

    public void start(String startState){
        currentState = states.get(startState);
        currentState.onTriggered(); //paranoia
    }

    public void handleEvent(Event<?> event){
        String nextState = currentState.handleEvent(event);
        if(nextState == null) return;
        if(!currentState.getName().equals(nextState)){
            currentState = states.get(nextState);
            currentState.onTriggered();
        }
    }

    public void run() {
        currentState.run();
    }


    public interface State {
        void run();
        String handleEvent(Event<?> event);
        String getName();
        void addTransitions(StateTransition... transitions);
        void onTriggered();
    }

    public static abstract class StateBase implements State {
        //name of this state
        protected String name;
        //transitions from this state
        protected List<StateTransition> stateTransitions;

        public StateBase(String name, StateTransition... transitions){
            this.name = name;
            stateTransitions = new ArrayList<>(Arrays.asList(transitions));
        }

        public String handleEvent(Event<?> event){
            String nextState = null;
            for(StateTransition t : stateTransitions){
            	//Check if the event required for a transition is met in the event given
                if(t.event.check(event)){
                    if(nextState != null) throw new IllegalStateException("Two StateTransitions are present for a single event!");
                    nextState = t.state;
                }
            }
            return nextState;
        }

        public void addTransitions(StateTransition... transitions){
            stateTransitions.addAll(new ArrayList<>(Arrays.asList(transitions)));
        }

        public String getName(){return name;}
        public String toString(){return name;}
    }
    public static class StateTransition {
        //event to be triggered by change
        private Event<?> event;
        //state to change to upon event trigger
        private String state;

        public StateTransition(Event<?> transitionEvent, String nextState){
            event = transitionEvent.copy();
            state = nextState;
        }
        
        
        public Event<?> getEvent(){
        	return event;
        }
        public String toString() {
        	return "State: "+state+"  "+
        			"Event: "+event.toString()+"\n";
        }
    }
}
