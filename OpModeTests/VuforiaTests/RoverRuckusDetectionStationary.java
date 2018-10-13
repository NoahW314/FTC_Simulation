package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests.VuforiaTests;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.VuforiaSimTC;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.OpModeSim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Vuforia Detection Stationary", group="Vuforia Detection")
public class RoverRuckusDetectionStationary extends OpModeSim {

    private VuforiaSimTC vuforia;
    private boolean targetVisible = false;
    private ElapsedTime timer = new ElapsedTime();
    private double detectTime = -1;

    @Override
    public void init() {
        //Vuforia initialization
    	VuforiaSimTC.Parameters parametersVuforia = new VuforiaSimTC.Parameters(hardwareMap);
        parametersVuforia.useExtendedTracking = false;
		parametersVuforia.cameraDirection = CameraDirection.BACK;

		try {
			vuforia = new VuforiaSimTC(parametersVuforia, "RoverRuckus");
		} catch (Exception e) {
			e.printStackTrace();
		}
        
        vuforia.activate();
    }

    @Override
    public void start(){
        timer.reset();
    }

    @Override
    public void loop() {
    	if(timer.seconds() > 2) {vuforia.trackables.get(0).isVisible = true;}
    	if(timer.seconds() > 5) {vuforia.trackables.get(1).isVisible = true;}
    	if(timer.seconds() > 10) {vuforia.trackables.get(2).isVisible = true;}
    	if(timer.seconds() > 15) {vuforia.trackables.get(3).isVisible = true;}
    	
        targetVisible = false;

        for (VuforiaSimTC.Trackable trackable : vuforia.trackables) {
            if (trackable.isVisible) {
                if(detectTime == -1) {
                    detectTime = timer.seconds();
                }
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
            }
        }

        if(!targetVisible){
            telemetry.addData("Wow", " nice cloaking device.");
        }
        telemetry.addData("Detection Time", detectTime);
        telemetry.update();
    }
}