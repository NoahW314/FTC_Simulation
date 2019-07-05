package org.firstinspires.ftc.teamcode.teamcalamari.Navigation;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Math2;

/**Base class for classes that represent spatial information like position, velocity, or acceleration*/
public abstract class SpatialBase<T extends SpatialBase<T>>{
    public double x, y, z;
    public DistanceUnit unit;
    private long timestamp;


    public SpatialBase(double x, double y, double z, DistanceUnit unit){
        this.x = x;
        this.y = y;
        this.z = z;
        this.unit = unit;
    }
    public SpatialBase(double x, double y, DistanceUnit unit){this(x,y,0,unit);}
    public SpatialBase(double x, DistanceUnit unit){this(x,0,unit);}
    public SpatialBase(DistanceUnit unit){this(0, unit);}
    public SpatialBase(double x, double y, double z){this(x,y,z,DistanceUnit.INCH);}
    public SpatialBase(double x, double y){this(x,y,0);}
    public SpatialBase(double x){this(x,0);}
    public SpatialBase(){this(0,0,0,DistanceUnit.INCH);}
    public SpatialBase(VectorF vector){this(vector, DistanceUnit.INCH);}
    public SpatialBase(VectorF vector, DistanceUnit unit){this(vector.get(0), vector.get(1), vector.get(2), unit);}

    /**Can be used to set the timestamp on the current object as with a normal setter,
     * or it can be called after construction, like so.<br><br>
     * <code>SpatialBase spatialBase = new SpatialBase().setTimestamp(System.nanoTime());</code>*/
    public T setTimestamp(long time){
        timestamp = time;
        return (T)this;
    }
    public long getTimestamp(){return timestamp;}

    protected abstract T create(double x, double y, double z, DistanceUnit unit);

    public double magnitude(){
        return Math.sqrt(x*x+y*y+z*z);
    }

    public void negate(){
        scale(-1);
    }
    public void add(T t){
        T t2 = t.createWithUnit(unit);
        x+=t2.x;
        y+=t2.y;
        z+=t2.z;
    }
    public void add(T t, DistanceUnit resultUnit){
        this.toUnit(resultUnit);
        add(t);
    }
    public void subtract(T t){
        T t2 = t.createWithUnit(unit);
        add(t2.negated());
    }
    public void subtract(T t, DistanceUnit resultUnit){
        this.toUnit(resultUnit);
        subtract(t);
    }
    public void round(){round(1);}
    public void round(double b){
        x = Math2.round(x, b);
        y = Math2.round(y, b);
        z = Math2.round(z, b);
    }
    public void scale(double scalar){
        x*=scalar;
        y*=scalar;
        z*=scalar;
    }


    public T negated(){ return this.scaled(-1); }
    public T added(T t){ return added(t, unit); }
    public T added(T t, DistanceUnit resultUnit){
        T t2 = t.createWithUnit(resultUnit);
        return this.create(x+t2.x, y+t2.y, z+t2.z, resultUnit);
    }
    public T subtracted(T t){ return subtracted(t, unit); }
    public T subtracted(T t, DistanceUnit resultUnit){ return added(t.negated(), resultUnit); }
    public T rounded(){ return rounded(1); }
    public T rounded(double b){ return this.create(Math2.round(x, b), Math2.round(y, b), Math2.round(z, b), unit); }
    public T scaled(double scalar){ return this.create(x*scalar, y*scalar, z*scalar, unit); }


    /**Convert the object to the given units*/
    public void toUnit(DistanceUnit newUnit){
        x = newUnit.fromUnit(unit, x);
        y = newUnit.fromUnit(unit, y);
        z = newUnit.fromUnit(unit, z);
        unit = newUnit;
    }
    /**Create a new object with the given units*/
    public T createWithUnit(DistanceUnit newUnit){
        return this.create(newUnit.fromUnit(unit, x), newUnit.fromUnit(unit, y), newUnit.fromUnit(unit, z), unit);
    }

    public T copy(){ return this.createWithUnit(this.unit); }


    public VectorF toVectorF(DistanceUnit unit){
        T spatialBase = this.createWithUnit(unit);
        return new VectorF((float)spatialBase.x, (float)spatialBase.y, (float)spatialBase.z);
    }
    public VectorF toVectorF(){
        return toVectorF(unit);
    }


    @Override
    public String toString(){
        return x+" "+y+" "+z+" "+unit.toString();
    }
}
