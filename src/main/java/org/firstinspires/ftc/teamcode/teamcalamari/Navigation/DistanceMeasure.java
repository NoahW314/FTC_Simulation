package org.firstinspires.ftc.teamcode.teamcalamari.Navigation;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**Represents a measure of distance in a certain unit*/
public class DistanceMeasure implements Comparable<DistanceMeasure> {
    public double value;
    public DistanceUnit unit;

    public DistanceMeasure(double value, DistanceUnit unit){
        this.value = value;
        this.unit = unit;
    }
    public DistanceMeasure(double value){this(value, DistanceUnit.INCH);}
    public DistanceMeasure(DistanceUnit unit){this(0, unit);}


    public double getValue(DistanceUnit newUnit){
        return newUnit.fromUnit(unit, value);
    }

    public void toUnit(DistanceUnit newUnit){
        value = newUnit.fromUnit(unit, value);
    }
    public DistanceMeasure createWithUnit(DistanceUnit newUnit){
        return new DistanceMeasure(newUnit.fromUnit(unit, value), newUnit);
    }

    public DistanceMeasure mult(double scalar) {
    	return new DistanceMeasure(value*scalar, unit);
    }
    public double div(DistanceMeasure m){
        DistanceMeasure newM = m.createWithUnit(unit);
        return value/newM.value;
    }

    public boolean greaterThan(DistanceMeasure m){
        DistanceMeasure newM = m.createWithUnit(unit);
        return value > newM.value;
    }
    public boolean lessThan(DistanceMeasure m){
        DistanceMeasure newM = m.createWithUnit(unit);
        return value < newM.value;
    }

    public DistanceMeasure copy(){return new DistanceMeasure(value, unit);}

    @Override
    public String toString(){
        return value+" "+unit;
    }
    
	@Override
	public int compareTo(DistanceMeasure m) {
		if(greaterThan(m)) return 1;
		if(lessThan(m)) return -1;
		return 0;
	}
}
