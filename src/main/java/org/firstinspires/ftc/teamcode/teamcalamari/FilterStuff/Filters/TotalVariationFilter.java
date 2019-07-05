package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters;

import  org.ejml.data.DMatrixRMaj;
import  org.ejml.data.DMatrixSparseCSC;
import  org.ejml.dense.row.CommonOps_DDRM;
import  org.ejml.sparse.csc.CommonOps_DSCC;

import java.util.ArrayList;

public class TotalVariationFilter extends IterantFilter<ArrayList<Double>> {
	
	private double[] smoothingFactor;
	private boolean iterating = false;
	ArrayList<ArrayList<Double>> y = new ArrayList<ArrayList<Double>>();
	ArrayList<ArrayList<Double>> dy = new ArrayList<ArrayList<Double>>();
	
	public TotalVariationFilter(double[] smoothing) {
		smoothingFactor = new double[smoothing.length];
		for(int i = 0; i < smoothing.length; i++) {
			smoothingFactor[i] = smoothing[i];
		}
	}

	public ArrayList<Double> run(ArrayList<Double[]> values, int measNum, int j) {
			if(!iterating) {
				y.add(new ArrayList<Double>());
				dy.add(new ArrayList<Double>());
			}
			double[] differences = new double[measNum-1];
			double[][] ya = new double[measNum][1];
			double[][] dya = new double[measNum-1][1];
			for(int i = 0; i < measNum-1 && !iterating; i++) {
				dy.get(j).add(values.get(i+1)[j+1]-values.get(i)[j+1]);
			}
			for(int i = 0; i < measNum && !iterating; i++) {
				y.get(j).add(values.get(i)[j+1]);
			}
			
			for(int i = 0; i < measNum-1; i++){
				differences[i] = Math.abs(values.get(i+1)[j+1]-values.get(i)[j+1])/smoothingFactor[j]+2;
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
			
			iterations[j]-=1;
			if(iterations[j] > 0) {
				iterating = true;
				System.out.println("Iterating "+(j));
				return this.run(values, measNum, j);
			}
		
		ArrayList<Double> accelerationList = new ArrayList<Double>();
		for(int i = 0; i < measNum; i++) {
			accelerationList.add(Xk1.get(i, 0));
		}
		iterating = false;
		return accelerationList;
	}

	@Override
	public ArrayList<Double> run(ArrayList<Double[]> values, int measNum) {
		return null;
	}

}
