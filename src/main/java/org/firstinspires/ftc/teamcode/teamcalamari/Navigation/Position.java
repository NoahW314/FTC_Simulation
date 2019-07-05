package org.firstinspires.ftc.teamcode.teamcalamari.Navigation;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Position extends SpatialBase<Position>{
    public Position(double x, double y, double z, DistanceUnit unit){ super(x,y,z,unit); }
    public Position(double x, double y, DistanceUnit unit){super(x,y,unit);}
    public Position(double x, DistanceUnit unit){super(x, unit);}
    public Position(DistanceUnit unit){super(unit);}
    public Position(double x, double y, double z){super(x,y,z,DistanceUnit.INCH);}
    public Position(double x, double y){super(x,y);}
    public Position(double x){super(x);}
    public Position(){super();}
    public Position(org.firstinspires.ftc.robotcore.external.navigation.Position pose){
        super(pose.x, pose.y, pose.z, pose.unit);
        setTimestamp(pose.acquisitionTime);
    }
    public Position(VectorF vectorF){ super(vectorF);}
    public Position(VectorF vectorF, DistanceUnit unit){super(vectorF, unit);}


    @Override
    protected Position create(double x, double y, double z, DistanceUnit unit) {
        return new Position(x,y,z,unit);
    }

    public org.firstinspires.ftc.robotcore.external.navigation.Position toNativePosition(){
        return new org.firstinspires.ftc.robotcore.external.navigation.Position(unit, x, y, z, 0);
    }
}
