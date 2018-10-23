package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim;

import static org.firstinspires.ftc.robotcore.internal.system.AppUtil.WEBCAM_CALIBRATIONS_DIR;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.Location;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import android.app.Activity;
import android.view.ViewGroup;

//TODO: add more sophisticated simulation of Vuforia
public class VuforiaSimTC{

	public ArrayList<VuforiaSimTC.Trackable> trackables = new ArrayList<>(5);
	private OpenGLMatrix phoneLocation;
	
	public VuforiaSimTC(VuforiaSimTC.Parameters parameters, String trackableFileName) throws ParserConfigurationException, SAXException, IOException {
		File file = new File("FtcRobotController/src/main/assets/"+trackableFileName+".xml");
		DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
		DocumentBuilder db = dbf.newDocumentBuilder();
		Document document = db.parse(file);
		Element el = document.getDocumentElement();
		NodeList nodeList = el.getElementsByTagName("Tracking").item(0).getChildNodes();
		for(int i = 0; i < nodeList.getLength(); i++) {
			String nodeName;
			if(!(nodeName = nodeList.item(i).getNodeName()).equals("#text")) {
				//We assume that Vuforia loads the targets in the order that they appear in 
				//in the xml file.  We haven't tested this assumption, but it seems the most logically choice  
				trackables.add(new Trackable());
				trackables.get(trackables.size()-1).isVuMark = nodeName.equals("VuMark");
			}
		}
		
		int k = 0;
		for(int j = 0; j < nodeList.getLength(); j++) {
			if(!nodeList.item(j).getNodeName().equals("#text")) {
				trackables.get(k).setName(nodeList.item(j).getAttributes().getNamedItem("name").getNodeValue());
				k++;
			}
			else {continue;}
		}
	}
	
	public static class Parameters {
		public Parameters(HardwareMapSim hwMapSim) {}
		
        public String vuforiaLicenseKey = "<visit https://developer.vuforia.com/license-manager to obtain a license key>";

        public Camera camera = null;
        public CameraName cameraName = null;
        public CameraDirection cameraDirection = CameraDirection.BACK;
        public boolean useExtendedTracking = true;
        
        public enum CameraMonitorFeedback { NONE, AXES , TEAPOT, BUILDINGS };
        public CameraMonitorFeedback cameraMonitorFeedback = CameraMonitorFeedback.AXES;
        
        public int cameraMonitorViewIdParent = 0;
        public ViewGroup cameraMonitorViewParent = null;
        public boolean fillCameraMonitorViewParent = false;

        public Activity activity = null;

        public File[] webcamCalibrationFiles = new File[] {};
        public double maxWebcamAspectRatio = Double.MAX_VALUE;
        public double minWebcamAspectRatio = 0;
        public int secondsUsbPermissionTimeout = 30;

        public void addWebcamCalibrationFile(String name) {
            addWebcamCalibrationFile(new File(WEBCAM_CALIBRATIONS_DIR, name));
        }

        public void addWebcamCalibrationFile(File file){
            webcamCalibrationFiles = Arrays.copyOf(webcamCalibrationFiles, webcamCalibrationFiles.length + 1);
            webcamCalibrationFiles[webcamCalibrationFiles.length - 1] = file;
        }
    }
	
	//TODO: add different argument options for setting the phone and target location
	/**Sets the target location of the {@code i}th element in {@code trackable}.
	The target starts with the picture facing up, with the right side (looking down on the target) pointing towards 
	the positive x-axis, and with the top side pointing towards the positive y-axis.
	The positive z-axis comes straight up out of the target.
	All rotations and translations are relative to the FTC field coordinate system.
	It is best to perform the rotations first while the target is still located at the origin.
	This is accomplished by doing 
	
	<pre>OpenGLMatrix.translation(x,y,z)
	.multiplied(Orientation.getRotationMatrix(axesReference, axesOrder, angleUnit, 
	firstRot, secondRot, thirdRot)</pre>
	
	If there is only one trackable you can use {@link org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.VuforiaTC#setTargetLocation(OpenGLMatrix) setTargetLocation(OpenGLMatrix)}
	@see <a href= "https://github.com/ftctechnh/ftc_app/files/989938/FTC_FieldCoordinateSystemDefinition.pdf">FTC field coordinate system</a>*/
	public void setTargetLocation(OpenGLMatrix targetLocation, int i) {
		trackables.get(i).location = targetLocation;
	}
	/**Sets the target location of the first trackable
	The target starts with the picture facing up, with the right side (looking down on the target) pointing towards 
	the positive x-axis, and with the top side pointing towards the positive y-axis.
	The positive z-axis comes straight up out of the target.
	All rotations and translations are relative to the FTC field coordinate system.
	It is best to perform the rotations first while the target is still located at the origin.
	This is accomplished by doing 
	
	<pre>OpenGLMatrix.translation(x,y,z)
	.multiplied(Orientation.getRotationMatrix(axesReference, axesOrder, angleUnit, 
	firstRot, secondRot, thirdRot)</pre>
	
	@see <a href= "https://github.com/ftctechnh/ftc_app/files/989938/FTC_FieldCoordinateSystemDefinition.pdf">FTC field coordinate system</a>*/
	public void setTargetLocation(OpenGLMatrix targetLocation) {
		setTargetLocation(targetLocation, 0);
	}
	
	/**The phone starts with the screen facing up ( in the direction of the positive z-axis), 
	with the right side (looking down the positive z-axis) pointing towards the positive x-axis, 
	and with the top side pointing towards the positive y-axis.
	All rotations and translations are relative to the robot axes.
	The robot axes are user defined and determined by Vuforia using the matrix you give it here.
	It is best to perform the rotations first while the target is still located at the origin.
	This is accomplished by doing 
	
	<pre>OpenGLMatrix.translation(x,y,z)
	.multiplied(Orientation.getRotationMatrix(axesReference, axesOrder, angleUnit, 
	firstRot, secondRot, thirdRot)</pre>*/
	public void setPhoneLocation(OpenGLMatrix phoneLocation) {
		this.phoneLocation = phoneLocation;
	}
	public OpenGLMatrix getPhoneLocation() {
		return phoneLocation;
	}
	
	public void activate() {}
	
	public Position getPositionOnField(int i) {
		return getLocationOnField(i).position;
	}
	public Orientation getOrientationOnField(int i) {
		return getLocationOnField(i).orientation;
	}
	public Location getLocationOnField(int i) {
		Trackable trackable = trackables.get(i);
		return new Location(trackable.robotPose, trackable.robotOrient);
	}
	
	public Position getPositionOnField() {
		return getPositionOnField(0);
	}
	public Orientation getOrientationOnField() {
		return getOrientationOnField(0);
	}
	public Location getLocationOnField() {
		return getLocationOnField(0);
	}
	
	public boolean isVuMark(int i) {
		return trackables.get(i).isVuMark;
	}
	public boolean isVuMark() {
		return isVuMark(0);
	}
	
	public static class Trackable {
		private String name;
		public void setName(String name) {this.name = name;}
		public String getName() {return name;}
		
		public boolean isVisible;
		public boolean isVuMark;
		public Position robotPose;
		public Orientation robotOrient;
		
		private OpenGLMatrix location;
		public OpenGLMatrix getLocation() {
			return location;
		}
	}
}
