package org.firstinspires.ftc.teamcode.teamcalamari.Navigation;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Velocity extends SpatialBase<Velocity> {
    public Velocity(double x, double y, double z, DistanceUnit unit){ super(x,y,z,unit); }
    public Velocity(double x, double y, DistanceUnit unit){super(x,y,unit);}
    public Velocity(double x, DistanceUnit unit){super(x, unit);}
    public Velocity(DistanceUnit unit){super(unit);}
    public Velocity(double x, double y, double z){super(x,y,z,DistanceUnit.INCH);}
    public Velocity(double x, double y){super(x,y);}
    public Velocity(double x){super(x);}
    public Velocity(){super();}
    public Velocity(org.firstinspires.ftc.robotcore.external.navigation.Velocity velocity){
        super(velocity.xVeloc, velocity.yVeloc, velocity.zVeloc, velocity.unit);
        setTimestamp(velocity.acquisitionTime);
    }
    public Velocity(VectorF vectorF){ super(vectorF);}
    public Velocity(VectorF vectorF, DistanceUnit unit){super(vectorF, unit);}


    @Override
    protected Velocity create(double x, double y, double z, DistanceUnit unit) {
        return new Velocity(x,y,z,unit);
    }

    public org.firstinspires.ftc.robotcore.external.navigation.Velocity toNativeVelocity(){
        return new org.firstinspires.ftc.robotcore.external.navigation.Velocity(unit, x, y, z, 0);
    }
}
