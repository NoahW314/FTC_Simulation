package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff;

import org.ejml.simple.SimpleMatrix;
import static org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters.KalmanFilterDropSensor.*;

public class SimpleMatrixTest {
	public static void main(String[] args) {
		double[][] matD = new double[5][5];
		for(int i = 0; i < 25; i++) {
			matD[(i-i%5)/5][i%5] = i+1;
		}
		
		SimpleMatrix mat = new SimpleMatrix(matD);
		SimpleMatrix fullMat = mat.copy();
		mat.print();
		
		removeRow(mat, 1);
		removeColumn(mat, 1);
		mat.print();
		
		removeRow(mat, 2);
		removeColumn(mat, 2);
		mat.print();
		
		addRow(mat, fullMat, 1, new int[] {0, -1, 1, -1, 2});
		addColumn(mat, fullMat, 1, new int[] {0, 1, 2, -1, 3});
		mat.print();
	}
}
