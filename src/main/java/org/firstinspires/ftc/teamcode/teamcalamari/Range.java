package org.firstinspires.ftc.teamcode.teamcalamari;

import java.util.ArrayList;
import java.util.List;

/**Inclusive on both ends*/
public class Range<T extends Comparable<T>> {
    private T low, high;
    private List<T> excludeValues = new ArrayList<>();
    public Range(T low, T high){
        this.low = low;
        this.high = high;
    }
    /**Used to create an unbounded range*/
    public Range(T value, boolean isLow) {
    	if(isLow) low = value;
    	else high = value;
    }

    public boolean contains(T value){
    	if(low == null) return value.compareTo(high) <= 0 && !excludeValues.contains(value);
    	if(high == null) return value.compareTo(low) >= 0 && !excludeValues.contains(value);
        return value.compareTo(low) >= 0 && value.compareTo(high) <= 0 && !excludeValues.contains(value);
    }
    
    public void addExclusion(T value) {
    	excludeValues.add(value);
    }

    public T getUpper() {return high;}
    public T getLower() {return low;}

    public Class<? extends Comparable> getParameterClass(){
        return low.getClass();
    }

    public Range<T> copy(){
        return new Range<>(low, high);
    }
    
    public String toString() {
    	return low+" "+high;
    }
}
