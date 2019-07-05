package org.firstinspires.ftc.teamcode.teamcalamari.Navigation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**Class which stores values for both degree and radian measures of an angle thereby eliminating the confusion caused
by using doubles to store angle values.  This was a big problem because the Math class trig functions work in radians while 
it is more convenient for the programmer to use degrees.*/

public class Angle {
	//shorthands
	public static final double PI = Math.PI;
	public static final double TwoPI = 2*Math.PI;
	
	//----------------------------------------------------------------------------------------------
	//Basic Stuff
	//----------------------------------------------------------------------------------------------
	
	//angle values
	private double degree;
	private double radian;
	
	//getters for the angle values
	public double getDegree() {return degree;}
	public double getRadian() {return radian;}
	
	//setters for the angle values
	public void setDegree(double degrees) {
		this.degree = degrees;
		this.radian = degrees*PI/180; 
	}
	public void setRadian(double radians) {
		this.radian = radians;
		this.degree = radians*180/PI;
	}
	/**sets the value of this angle in the given angle unit*/
	public void setUnit(double angle, AngleUnit unit) {
		switch(unit) {
			case DEGREES:
				setDegree(angle);
				break;
			case RADIANS:
				setRadian(angle);
				break;
			default:
				throw new IllegalArgumentException(unit+" is not a valid angle unit");
		}
	}
	
	/**The axes that this angle is measured relative to*/
	private AxesSet axes = AxesSet.UNKNOWN;
	public AxesSet getAxes() {return axes;}
	public void setAxesSet(AxesSet set) {
		axes = set;
	}
	/**Returns a new angle equivalent to this angle, but in the given axes set*/
	public Angle toAxesSet(AxesSet set) {
		return AxesSet.toAxesSet(set, this);
	}
	/**Converts this angle to the given axes set*/
	public void convertToAxesSet(AxesSet set) {
		this.setAxesSet(set);
		this.setDegree(AxesSet.toAxesSet(set, this).degree);
	}
	
	//constructors
	/**The default angle unit is degrees*/
	public Angle(double angle) {
		this(angle, AngleUnit.DEGREES);
	}
	public Angle(double angle, AngleUnit unit) {
		this(angle, unit, AxesSet.UNKNOWN);
	}
	public Angle(double angle, AngleUnit unit, AxesSet axes) {
		setUnit(angle, unit);
		this.axes = axes;
	}
	
	//----------------------------------------------------------------------------------------------
	//Basic Math
	//----------------------------------------------------------------------------------------------

	//Most of these math functions will modify the angle from which the method was called and return it
	
	
	/**Adds <code>angle</code> to <code>this</code> and returns this angle as the sum.*/
	public Angle add(Angle angle){
	    double angleInDegrees = this.degree+angle.degree;
        this.setDegree(angleInDegrees);
	    return this;
    }
	/**Subtracts <code>angle</code> from <code>this</code> and returns this angle as the difference*/
	public Angle subtract(Angle angle) {
		this.setDegree(this.degree-angle.degree);
		return this;
	}
	/**Multiplies <code>this</code> by <code>scalar</code> and returns this angle as the product.*/
	public Angle mult(double... scalars) {
	    double angleInDegrees = this.degree;
	    for(double scalar : scalars){
	        angleInDegrees*=scalar;
        }
		this.setDegree(angleInDegrees);
		return this;
	}
	/**Divides <code>this</code> by <code>angle</code> canceling out the angle unit and returning a double.*/
	public double div(Angle angle) {
		return this.degree/angle.degree;
	}
	/**Takes the absolute value of the angle value and returns this angle*/
	public Angle abs() {
		this.setDegree(Math.abs(this.degree));
		return this;
	}
	public Angle negate(){
	    this.setDegree(-this.degree);
	    return this;
    }
	
	/**Should be used with angles who have been passed through convertAngle for best results
	@return if this angle is greater than the given angle*/
	public boolean greaterThan(Angle angle) {
		return this.degree > angle.degree;
	}
	/**Should be used with angles who have been passed through convertAngle for best results
	@return if this angle is less than the given angle*/
	public boolean lessThan(Angle angle){
	    return this.degree < angle.degree;
    }
	
	//----------------------------------------------------------------------------------------------
	//Basic Static Math
	//----------------------------------------------------------------------------------------------
	
	//Most of these math functions will return a new angle with the results without modifying the given angles
	
	/**Adds <code>angle1</code> and <code>angle2</code> and returns a new angle as the sum.*/
	public static Angle add(Angle angle1, Angle angle2) {
	    double angleInDegrees = angle1.degree+angle2.degree;
		return new Angle(angleInDegrees, AngleUnit.DEGREES);
	}
	public static Angle subtract(Angle angle1, Angle angle2) {
		return new Angle(angle1.getDegree()-angle2.getDegree(), AngleUnit.DEGREES);
	}
	public static Angle mult(Angle angle, double... scalars) {
	    double angleInDegrees = angle.degree;
	    for(double scalar : scalars){
	        angleInDegrees*=scalar;
        }
		return new Angle(angleInDegrees, AngleUnit.DEGREES);
	}
	/**Takes the absolute value of the angle value and returns a new angle*/
	public static Angle abs(Angle angle) {
		return new Angle(Math.abs(angle.degree), AngleUnit.DEGREES);
	}
	public static Angle negate(Angle angle) {
		return new Angle(-angle.degree, AngleUnit.DEGREES);
	}
	
	//----------------------------------------------------------------------------------------------
	//Angle Conversions
	//----------------------------------------------------------------------------------------------
	
	/**converts the angle to a range of [-180, 180) and returns it.
	also converts the radian measure to a range of [-PI, PI)*/
    public double to180(){
        while(degree >= 180){
            degree-=360;
            radian-=TwoPI;
        }
        while(degree < -180){
            degree+=360;
            radian+=TwoPI;
        }
        return degree;
    }
	/**converts the angle to a range of [0, 360) and returns it
	also converts the radian measure to a range of [0, TwoPI)*/
    public double to360(){
    	while(degree >= 360) {
    		degree-=360;
    		radian-=TwoPI;
    	}
    	while(degree < 0) {
    		degree+=360;
    		radian+=TwoPI;
    	}
        return degree;
    }
    /**converts the angle to a range of [-PI, PI) and returns it.
    also converts the degree measure to a range of [-180, 180)*/
    public double toPI() {
    	while(radian >= PI) {
    		radian-=TwoPI;
    		degree-=360;
    	}
    	while(radian < -PI) {
    		radian+=TwoPI;
    		degree+=360;
    	}
    	return radian;
    }
    /**converts the angle to a range of [0, TwoPI) and returns it.
    also converts the degree measure to a range of [0, 360)*/
    public double to2PI() {
    	while(radian >= TwoPI) {
    		radian-=TwoPI;
    		degree-=360;
    	}
    	while(radian < 0) {
    		radian+=TwoPI;
    		degree+=360;
    	}
    	return radian;
    }
    /**converts the angle measures to a range of [0, 360) and [0, TwoPI) and returns the measure specified by <code>unit</code>*/
    public double toZero(AngleUnit unit) {
    	switch(unit) {
    		case RADIANS:
    			return to2PI();
    		case DEGREES:
    			return to360();
    		default:
    			throw new IllegalArgumentException(unit+" is not a valid angle unit");
    	}
    }
    /**converts the angle measures to a range of [-180, 180) and [-PI, PI) and returns the measure specified by <code>unit</code>*/
    public double toNonZero(AngleUnit unit) {
    	switch(unit){
			case DEGREES:
				return to180();
			case RADIANS:
				return toPI();
			default:
				throw new IllegalArgumentException(unit+" is not a valid angle unit");
    	}
    }
    
    //----------------------------------------------------------------------------------------------
  	//Static Angle Conversions
  	//----------------------------------------------------------------------------------------------
    
    /**returns the angle measure in a range of [0, 360) and [0, TwoPI)*/
    public static Angle toZero(Angle angle) {
    	double degree = angle.degree;
    	while(degree >= 360) {
    		degree-=360;
    	}
    	while(degree < 0) {
    		degree+=360;
    	}
        return new Angle(degree, AngleUnit.DEGREES);
    }
    /**returns the angle measure in a range of [-180, 180) and [-PI, PI)*/
    public static Angle toNonZero(Angle angle) {
    	double degree =  angle.degree;
        while(degree >= 180){
            degree-=360;
        }
        while(degree < -180){
            degree+=360;
        }
        return new Angle(degree, AngleUnit.DEGREES);
    }
	/**returns an angle equal to the first angle that it is within 180 degrees and PI radians of the second angle*/
    public static Angle convertAngle (Angle angle1, Angle angle2){
    	double degree = angle1.degree;
    	while(degree-angle2.degree >= 180) {
    		degree-=360;
    	}
    	while(degree-angle2.degree < -180) {
    		degree+=360;
    	}
        return new Angle(degree, AngleUnit.DEGREES);
    }
    
    @Override
    public String toString() {
    	return degree+" degrees "+axes;
    }
    //paranoia
    public Angle copy(){
        return Angle.add(this, new Angle(0));
    }
}
