package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersAccelIntegration;

import  org.ejml.data.DMatrixRMaj;
import  org.ejml.data.DMatrixSparseCSC;
import  org.ejml.dense.row.CommonOps_DDRM;
import  org.ejml.sparse.csc.CommonOps_DSCC;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;

import java.util.ArrayList;

public class TotalVariationFilter extends IterantFilter {
	
	private double[] smoothingFactor;
	private boolean iterating = false;
	private int measNum;
	ArrayList<ArrayList<Double>> y = new ArrayList<ArrayList<Double>>();
	ArrayList<ArrayList<Double>> dy = new ArrayList<ArrayList<Double>>();
	
	public TotalVariationFilter(double[] smoothing) {
		smoothingFactor = new double[3];
		for(int i = 0; i < 3; i++) {
			smoothingFactor[i] = smoothing[i];
			y.add(new ArrayList<Double>());
			dy.add(new ArrayList<Double>());
		}
	}
	
	@Override
	public void update(Acceleration linearAcceleration) {
		currValue = linearAcceleration;
		values.add(linearAcceleration);
		ArrayList<Acceleration> accelerations = run();
		
		velocity = initialVelocity;
		position = initialPosition;
		prevAccel = null;
		prevVelocity = null;
		
		for(int i = 0; i < accelerations.size()-1; i++) {
			filtValues.set(i, accelerations.get(i));
			integrate(accelerations.get(i));
		}
	}

	private void integrate(Acceleration acceleration) {
		if (prevAccel != null && prevAccel.acquisitionTime != 0){
	        Velocity deltaVelocity = meanIntegrate(acceleration, prevAccel);
	        velocity = plus(velocity, deltaVelocity);
		}
		if (prevVelocity != null &&  prevVelocity.acquisitionTime != 0){
	        Position deltaPosition = meanIntegrate(velocity, prevVelocity);
	        position = plus(position, deltaPosition);
        }
		
		prevAccel = acceleration;
		prevVelocity = velocity;
	}
	
	public ArrayList<Acceleration> run() {
		int measNum = values.size()-1;
		DMatrixRMaj[] Xk1A = new DMatrixRMaj[3];
		
		for(int j = 0; j < 3; j++) {
			while(iterations[j] > 0) {
				Xk1A[j] = iterate(j);
			
				iterations[j]-=1;
				iterating = true;
				System.out.println("Iterating "+(j));
			}
		}
		
		ArrayList<Acceleration> accelerationList = new ArrayList<Acceleration>();
		for(int i = 0; i < measNum; i++) {
			accelerationList.add(
					DoubleArrayToAccel(new double[]{Xk1A[0].get(i, 0), Xk1A[1].get(i, 0), Xk1A[2].get(i, 0)}, currValue.acquisitionTime));
		}
		iterating = false;
		return accelerationList;
	}
	
	private DMatrixRMaj iterate(int j) {
		
		double[] differences = new double[measNum-1];
		double[][] ya = new double[measNum][1];
		double[][] dya = new double[measNum-1][1];
		for(int i = 0; i < measNum-1 && !iterating; i++) {
			dy.get(j).add(AccelAxisFromInt(values.get(i+1), j+1)-AccelAxisFromInt(values.get(i), j+1));
		}
		for(int i = 0; i < measNum && !iterating; i++) {
			y.get(j).add(AccelAxisFromInt(values.get(i), j+1));
		}
		
		for(int i = 0; i < measNum-1; i++){
			differences[i] = Math.abs(AccelAxisFromInt(values.get(i+1), j+1)-AccelAxisFromInt(values.get(i), j+1))/smoothingFactor[j]+2;
			dya[i][0] = dy.get(j).get(i);
			ya[i][0] = y.get(j).get(i);
		}
		ya[measNum-1][0] = y.get(j).get(measNum-1);
		
		DMatrixSparseCSC diagM = CommonOps_DSCC.diag(differences);
		
		DMatrixSparseCSC DDT = new DMatrixSparseCSC(measNum-1, measNum-1);
		DMatrixSparseCSC DT = new DMatrixSparseCSC(measNum, measNum-1);
		for(int i = 0; i < measNum-2; i++) {
			DDT.set(i, i+1, -1);
			DDT.set(i+1, i, -1);
		}
		for(int i = 0; i < measNum-1; i++) {
			DT.set(i, i, -1);
			DT.set(i+1, i, 1);
		}
		DMatrixSparseCSC addM = new DMatrixSparseCSC(measNum-1, measNum-1);
		CommonOps_DSCC.add(1, diagM, 1, DDT, addM, null, null);
		DMatrixRMaj invM = new DMatrixRMaj(measNum-1, measNum-1);
		CommonOps_DSCC.invert(addM, invM);
		DMatrixRMaj prod1 = new DMatrixRMaj(measNum, measNum-1);
		CommonOps_DSCC.mult(DT, invM, prod1);
		DMatrixRMaj prod2 = new DMatrixRMaj(measNum, 1);
		DMatrixRMaj Dy = new DMatrixRMaj(dya);
		CommonOps_DDRM.mult(prod1, Dy, prod2);
		DMatrixRMaj Xk1 = new DMatrixRMaj(measNum, 1);
		DMatrixRMaj Y = new DMatrixRMaj(ya);
		CommonOps_DDRM.subtract(Y, prod2, Xk1);
		return Xk1;
	}

}
