package org.firstinspires.ftc.teamcode.teamcalamari.Navigation;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Acceleration extends SpatialBase<Acceleration> {
    public Acceleration(double x, double y, double z, DistanceUnit unit){ super(x,y,z,unit); }
    public Acceleration(double x, double y, DistanceUnit unit){super(x,y,unit);}
    public Acceleration(double x, DistanceUnit unit){super(x, unit);}
    public Acceleration(DistanceUnit unit){super(unit);}
    public Acceleration(double x, double y, double z){super(x,y,z,DistanceUnit.INCH);}
    public Acceleration(double x, double y){super(x,y);}
    public Acceleration(double x){super(x);}
    public Acceleration(){super();}
    public Acceleration(org.firstinspires.ftc.robotcore.external.navigation.Acceleration accel){
        super(accel.xAccel, accel.yAccel, accel.zAccel, accel.unit);
        setTimestamp(accel.acquisitionTime);
    }
    public Acceleration(VectorF vectorF){ super(vectorF);}
    public Acceleration(VectorF vectorF, DistanceUnit unit){super(vectorF, unit);}

    @Override
    protected Acceleration create(double x, double y, double z, DistanceUnit unit) {
        return new Acceleration(x,y,z,unit);
    }

    public org.firstinspires.ftc.robotcore.external.navigation.Acceleration toNativeAcceleration(){
        return new org.firstinspires.ftc.robotcore.external.navigation.Acceleration(unit, x, y, z, 0);
    }
}
