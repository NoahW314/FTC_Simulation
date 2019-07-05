package org.firstinspires.ftc.teamcode.teamcalamari.Navigation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**Enum attached to an angle indicating which axes the angle is measured from.*/
public enum AxesSet {
	/**Angle is measured from the field coordinate system axes*/
	FIELD_AXES(0),
	/**Angle is measured from the robot axes, as defined by the order the wheel were listed in*/
	ROBOT_WHEEL_AXES(1),
	/**Angle is measured from the user defined robot axes.  
	So for an AllDirectionsDrive the offset between these axes and ROBOT_WHEEL_AXES will be the headOffset of the robot*/
	ROBOT_USER_AXES(2),
	/**Angle is measured from the accelerometer axes*/
	ACCELEROMETER_AXES(3),
	/**Should be used when there is an additional set of axes from which angles can be measured which is not included here. 
	This should not be used to represent two different sets of axes in a single OpMode. 
	If the axes from which the angle is measured is not cared about use {@link AxesSet#UNKNOWN UNKNOWN}*/
	OTHER(4),
	/**Used as the default. Should be used when the axes the angle is measured from is either not known or not cared about*/
	UNKNOWN(-1);
	
	/**How much this set of axes is offset from its relative axes.
	The angle is measured from these axes to the relative axes.
	For field axes this value will be null, since the field axes are not defined relative to others
	@see AxesSet#relativeAxes relativeAxes
	@see AxesSet#setOffset setOffset*/
	private Angle offset = null;
	/**The id of the axes which these axes are measured relative to.<br>
	The default value is -1, since -1 is the id of AxesSet.UNKNOWN.<br>
	The default value for ACCELEROMETER_AXES is 1, since the accelerometer will likely be mounted directly to the robot.<br>
	The default value for ROBOT_AXES is 0, since the robot's heading is most useful when defined relative to the field.<br> 
	For field axes this value will be 0, since the field axes are not defined relative to others
	@see AxesSet#offset offset
	@see AxesSet#setOffset setOffset*/
	private int relativeAxes = -1;
	
	public final int id;
	private AxesSet(int i) {
		id = i;
		if(id == 0) {
			relativeAxes = 0;
			offset = new Angle(0);}
		if(id == 1) {relativeAxes = 0;}
		if(id == 2) {relativeAxes = 1;}
		if(id == 3) {relativeAxes = 1;}
	}
	
	public static AxesSet getAxesById(int id) {
		for(AxesSet axes: AxesSet.values()) {
			if(axes.id == id) {return axes;}
		}
		throw new IllegalArgumentException(id+" is not a valid axes set id");
	};
	
	public void setRelativeAxes(int axesID) {
		if(this != AxesSet.FIELD_AXES) {
			relativeAxes = axesID;
		}
	}
	public int getRelativeAxes() {
		return relativeAxes;
	}
	
	/**{@code offset} is the offset of these axes and its relative axes 
	as measured from these axes to the relative axes.
	As the field axes are the most absolute axes (it is the only set of axes not defined relative to others),
	the offset need not be called for field axes and will have not effect if called
	@see AxesSet#offset offset
	@see AxesSet#relativeAxes relativeAxes*/
	public void setOffset(Angle offset) {
		if(this != AxesSet.FIELD_AXES) {
			this.offset = offset;
		}
	}
	public Angle getOffset() {
		return this.offset;
	}
	
	/**Returns a new angle equivalent to the angle given, but in the AxesSet {@code set}.*/
	public static Angle toAxesSet(AxesSet set, Angle angle) {
		//if we don't know the starting axes set we can't convert to any other axes set
		if(angle.getAxes() == AxesSet.UNKNOWN) {throw new IllegalArgumentException("Cannot convert angle from AxesSet.UNKNOWN");}
		
		Angle newAngle = new Angle(angle.getDegree(), AngleUnit.DEGREES, set);
		
		switch(set.relativeAxes) {
			case -1:
				//Although it is highly unlikely that someone will convert an angle to an UNKNOWN AxesSet we still allow it.
				//If the conversion is not to AxesSet.UNKNOWN then the relative axes of the axes set has not been set
				if(set.id != -1){throw new IllegalArgumentException("The AxesSet "+set+" has not had its relative axes set");}
				break;
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
				//convert the angle to the field axes by following the parent axes
				newAngle = convertToField(newAngle, angle.getAxes());
				//convert the angle from the field axes to the AxesSet set by adding the result
				//of converting an angle measure of zero from the AxesSet set to the field axes
				newAngle = convertFieldToAxes(newAngle, set);
				break;
			default: throw new IllegalArgumentException("AxesSet set does not contain valid relative axes");
		}
		return newAngle;
	}
	/**Converts and returns the given angle from the starting axes to the field axes*/
	private static Angle convertToField(Angle angle, AxesSet startingAxes) {
		//trace the parent axes up the chain, converting the angle as we go along, until we reach the top, field axes
		AxesSet axes = startingAxes;
		while(axes.id != 0) {
			angle.subtract(axes.offset);
			axes = getAxesById(axes.relativeAxes);
		}
		return angle;
	}
	/**Converts and returns the given angle from the field axes to the ending axes*/
	private static Angle convertFieldToAxes(Angle angle, AxesSet endingAxes) {
		Angle angleDiff = convertToField(new Angle(0), endingAxes);
		angle.add(angleDiff);
		return angle;
	}

}
