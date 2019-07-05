package org.firstinspires.ftc.teamcode.teamcalamari;

/**Inclusive on both ends*/
public class Range<T extends Comparable<T>> {
    private T low, high;
    public Range(T low, T high){
        this.low = low;
        this.high = high;
    }

    public boolean contains(T value){
        return value.compareTo(low) >= 0 && value.compareTo(high) <= 0;
    }

    public T getUpper() {return high;}
    public T getLower() {return low;}

    public Class<? extends Comparable> getParameterClass(){
        return low.getClass();
    }

    public Range<T> copy(){
        return new Range<>(low, high);
    }
}
