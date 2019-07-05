package org.firstinspires.ftc.teamcode.teamcalamari.Navigation;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**Location represents an object's (most likely a robot's) position and orientation*/
public class Location {
    /**DEPRECATED - use getPosition() instead*/
    @Deprecated
	public Position position;
    /**DEPRECATED - use getOrientation() instead*/
    @Deprecated
	public Orientation orientation;

    public org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position getPosition(){
        return new org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position(position);
    }
    public void setPosition(org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position pose){
    	position = pose.toNativePosition();
    }

    public Orientation getOrientation() { return orientation; }
    public void setOrientation(Orientation o) {
    	orientation = o;
    }

    public Location() {
		position = new Position(DistanceUnit.INCH,0,0,0,0);
		orientation = new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,0,0,0,0);
	}
	public Location(Position pose) {
		position = pose;
	}
	public Location(Orientation orient) {
		orientation = orient;
	}
	public Location(Position pose, Orientation orient) {
		position = pose;
		orientation = orient;
	}
	
	public static Location openGLMatrixToLocation(OpenGLMatrix matrix, DistanceUnit dUnit) {
	    if(matrix == null){return null;}
		Location location = new Location();
		VectorF translation = matrix.getTranslation();
		location.position = new Position(dUnit, translation.get(0), translation.get(1), translation.get(2), 0);
		location.orientation = Orientation.getOrientation(matrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
		return location;
	}
	
	public static OpenGLMatrix locationToOpenGLMatrix(Location location) {
		Position pose = location.position;
		Orientation orient = location.orientation;
		return OpenGLMatrix.translation((float)pose.x, (float)pose.y, (float)pose.z)
							.multiplied(Orientation.getRotationMatrix(orient.axesReference, orient.axesOrder, orient.angleUnit,
								orient.firstAngle, orient.secondAngle, orient.thirdAngle));
	}
	
	@Override
	public String toString() {
		return position.toString()+"\n"+orientation.toString();
	}
	public Location copy(){ return new Location(position, orientation); }
}
