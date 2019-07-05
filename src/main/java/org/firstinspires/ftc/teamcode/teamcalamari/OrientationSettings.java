package org.firstinspires.ftc.teamcode.teamcalamari;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**class for the settings used by the Orientation class:<br> AxesReference, AxesOrder, and AngleUnit*/
public class OrientationSettings {
	public AxesReference axesReference = AxesReference.EXTRINSIC;
	public AxesOrder axesOrder = AxesOrder.XYZ;
	public AngleUnit angleUnit = AngleUnit.DEGREES;
	
	public OrientationSettings(AxesReference axesReference, AxesOrder axesOrder, AngleUnit angleUnit) {
		this.axesReference = axesReference;
		this.axesOrder = axesOrder;
		this.angleUnit = angleUnit;
	}
	public OrientationSettings() {}
	
	@Override
	public String toString() {
		return axesReference+" "+axesOrder+" "+angleUnit;
	}
}