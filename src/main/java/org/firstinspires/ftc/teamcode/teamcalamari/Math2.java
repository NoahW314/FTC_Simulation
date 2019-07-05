package org.firstinspires.ftc.teamcode.teamcalamari;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;

/**
 * Created by Tavis on 2/5/2018.
 */

/**A class for several math things not included in the java Math class*/
public class Math2{

	//----------------------------------------------------------------------------------------------
	//Angle Methods
	//----------------------------------------------------------------------------------------------
    
	/**converts <code>angle</code> to a range of [-180,180)*/
    public static double to180(double angle){
        while(angle >= 180){
            angle-=360;
        }
        while(angle < -180){
            angle+=360;
        }
        return angle;
    }
	/**converts <code>angle</code> to a range of [0,360)*/
    public static double to360(double angle){
    	while(angle >= 360){
            angle-=360;
        }
        while(angle < 0){
            angle+=360;
        }
        return angle;
    }
    /**converts <code>angle</code> to a range of [-PI, PI)*/
    public static double toPI(double angle) {
    	while(angle >= Math.PI) {
    		angle-=(2*Math.PI);
    	}
    	while(angle < -Math.PI) {
    		angle+=(2*Math.PI);
    	}
    	return angle;
    }
    /**converts <code>angle</code> to a range of [0, 2PI)*/
    public static double to2PI(double angle) {
    	return toPI(angle)+Math.PI;
    }
    /**converts <code>angle</code> to a range of [0,360) of [0,2PI) depending on the angle unit, <code>unit</code>*/
    public static double toZero(double angle, AngleUnit unit) {
    	switch(unit) {
    		case RADIANS:
    			return to2PI(angle);
    		case DEGREES:
    			return to360(angle);
    		default:
    			throw new IllegalArgumentException(unit+" is not a valid angle unit");
    	}
    }
	/**converts <code>angle</code> so that it is within 180 degrees of <code>angle2</code>*/
    public static double convertAngles (double angle, double angle2){
    	while(angle-angle2 >= 180) {
    		angle-=360;
    	}
    	while(angle-angle2 < -180) {
    		angle+=360;
    	}
        return angle;
    }
    
    //----------------------------------------------------------------------------------------------
    //Rounding Methods
    //----------------------------------------------------------------------------------------------
    
    /**rounds <code>n</code> to the nearest multiple of <code>b</code>*/
    public static double round(double n, double b){
    	return (b != 0) ? Math.round(n/b)*b : n;
    }
    /**rounds <code>n</code> to the nearest multiple of <code>b</code>*/
    public static float round(float n, float b){
    	return (b != 0) ? Math.round(n/b)*b : n;
    }
    
    /**performs the Math2.round method on all values of the Acceleration*/
    public static Acceleration round(Acceleration accel, double b) {
    	accel.xAccel = round(accel.xAccel, b);
    	accel.yAccel = round(accel.yAccel, b);
    	accel.zAccel = round(accel.zAccel, b);
    	return accel;
    }
    /**performs the Math2.round method on all values of the Velocity*/
    public static Velocity round(Velocity velo, double b) {
    	velo.xVeloc = round(velo.xVeloc, b);
    	velo.yVeloc = round(velo.yVeloc, b);
    	velo.zVeloc = round(velo.zVeloc, b);
    	return velo;
    }
    /**performs the Math2.round method on all values of the Position*/
    public static Position round(Position pose, double b) {
    	pose.x = round(pose.x, b);
    	pose.y = round(pose.y, b);
    	pose.z = round(pose.z, b);
    	return pose;
    }
    /**performs the Math2.round method on all values of the VectorF*/
    public static VectorF round(VectorF v, float b) {
    	for(int i = 0; i < v.length(); i++) {
    		v.put(i, round(v.get(i), b));
    	}
    	return v;
    }
    /**performs the Math2.round method on the angle*/
    public static Angle round(Angle angle, Angle b) {
		return new Angle(Math2.round(angle.getDegree(), b.getDegree()), AngleUnit.DEGREES);
	}
    
    /**rounds all values of the Acceleration to the nearest integer*/
    public static Acceleration round(Acceleration accel) {
    	return round(accel, 1);
    }
    /**rounds all values of the Velocity to the nearest integer*/
    public static Velocity round(Velocity velo) {
    	return round(velo, 1);
    }
    /**rounds all values of the Position to the nearest integer*/
    public static Position round(Position pose) {
    	return round(pose, 1);
    }
    /**rounds all values of the VectorF to the nearest integer*/
    public static VectorF round(VectorF v) {
    	return round(v, 1);
    }
    
    //----------------------------------------------------------------------------------------------
    //Absolute Value Methods
    //----------------------------------------------------------------------------------------------
    
    /**Applies <code>Math.abs()</code> to each element*/
    public static VectorF abs(VectorF v) {
    	for(int i = 0; i < v.length(); i++) {
    		v.put(i, Math.abs(v.get(i)));
    	}
    	return v;
    }
    
    //----------------------------------------------------------------------------------------------
    //Other Methods
    //----------------------------------------------------------------------------------------------
    
    /**Returns the maximum value from the given doubles*/
    public static double max(double...d) {
    	double max = d[0];
    	for(int i = 1; i < d.length; i++) {
    		if(d[i] > max) {max = d[i];}
    	}
    	return max;
    }
    
}