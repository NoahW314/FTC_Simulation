package org.firstinspires.ftc.teamcode.teamcalamari.Simulation;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.OpModeSim;

public class LogSim {
	//maps of field names to logging tags
	/**Map used for fields that should be logged every time log() is called*/
	public Map<String, String> loggers = new HashMap<>();
	/**Map used for fields that should only be logged when logError() is called.  
	All fields in loggers will also be logged when logError() is called*/
	public Map<String, String> errorLoggers = new HashMap<>();
	
	//maps of field objects to logging tags
	/**Map used for fields that should be logged every time log() is called*/
	private Map<Field, String> loopLoggers = new HashMap<>();
	/**Map used for fields that should only be logged when logError() is called.  
	All fields in loggers will also be logged when logError() is called*/
	private Map<Field, String> errLoggers = new HashMap<>();
	public OpModeSim opMode;
	
	private Writer writer;
    private StringBuffer lineBuffer;

    public LogSim (OpModeSim opMode) {
    	this.opMode = opMode;
    	
        String directoryPath    = "F:\\TC Logs";
        String filePath         = directoryPath + "\\" + opMode.getClass().getSimpleName() + " " + System.currentTimeMillis() +".txt";

        new File(directoryPath).mkdir();        // Make sure that the directory exists

        try {
            writer = new FileWriter(filePath);
            lineBuffer = new StringBuffer(128);
        } catch (IOException e) {
        }
    }

	public void findFields() {
    	for(Entry<String, String> entry : loggers.entrySet()) {
    		try {
	    		Field field = opMode.getClass().getDeclaredField(entry.getKey());
	    		//make sure we can access field we are trying to log
	    		field.setAccessible(true);
	    		loopLoggers.put(field, entry.getValue());
    		}catch(NoSuchFieldException fe) {}
    	}
    	for(Entry<String, String> entry : errorLoggers.entrySet()) {
    		try {
	    		Field field = opMode.getClass().getDeclaredField(entry.getKey());
	    		//make sure we can access field we are trying to log
	    		field.setAccessible(true);
	    		errLoggers.put(field, entry.getKey());
    		}catch(NoSuchFieldException fe) {}
    	}
    }
    
    public void log() {
    	for(Entry<Field, String> entry : loopLoggers.entrySet()) {
    		try {
    			addLine(entry.getValue()+": "+entry.getKey().get(opMode).toString());
			} catch (Exception e) {
				//logging failures should always catch errors
				//we want to continue running the app even if the logging is not working
				addLine(entry.getValue()+": could not access value of field");
		    	addLine(e.getClass().getSimpleName()+": "+e.getLocalizedMessage());
			}
    	}
    	newLine();
    	
    	//paranoia
    	//make sure this data has a chance to get written before 
    	//anything else can throw an error or stop the op mode
    	try {
			writer.flush();
		} catch (IOException e) {
			//this most likely won't do anything since if the writer.flush is
			//throwing an error this data will not get written to the file on the phone
			addLine("Could not flush writer");
		}
    }
    
    /**Used to log OpMode errors*/
    public void logError(RuntimeException e) {
    	//log all normal information first
    	log();
    	
    	//log system information at the time of the error
    	for(Entry<Field, String> entry : errLoggers.entrySet()) {
    		try {
    			addLine(entry.getValue()+": "+entry.getKey().get(opMode).toString());
			} catch (Exception e2) {
				//logging failures should always catch errors
				//we want to continue running the app even if the logging is not working
				addLine(entry.getValue()+": could not access value of field");
		    	addLine(e2.getClass().getSimpleName()+": "+e.getLocalizedMessage());
			}
    	}
    	
    	//separate system info from error messsage
    	newLine();
    	//log information about the error itself
    	addLine(e.getClass().getSimpleName()+": "+e.getLocalizedMessage());
    	StackTraceElement[] stackTrace = e.getStackTrace();
    	for(StackTraceElement el : stackTrace) {
    		addLine(el.toString());
    	}
    	
    	close();
    	throw e;
    }
    /**Used to log LinearOpMode errors*/
    public void logError(InterruptedException e) throws InterruptedException {
    	//log all normal information first
    	log();
    	//log system information at the time of the error
    	for(Entry<Field, String> entry : errLoggers.entrySet()) {
    		try {
    			addLine(entry.getValue()+": "+entry.getKey().get(opMode).toString());
			} catch (Exception e2) {
				//logging failures should always catch errors
				//we want to continue running the app even if the logging is not working
				addLine(entry.getValue()+": could not access value of field");
		    	addLine(e2.getClass().getSimpleName()+": "+e.getLocalizedMessage());
			}
    	}
    	
    	//separate system info from error messsage
    	newLine();
    	//log information about the error itself
    	addLine(e.getClass().getSimpleName()+": "+e.getLocalizedMessage());
    	StackTraceElement[] stackTrace = e.getStackTrace();
    	for(StackTraceElement el : stackTrace) {
    		addLine(el.toString());
    	}
    	
    	close();
    	throw e;
    }
    
    private void newLine(){
        try {
            lineBuffer.append('\n');
            writer.write(lineBuffer.toString());
            lineBuffer.setLength(0);
        }
        catch (IOException e){
        	//not much we can do here
        	//we can't log the error since the newLine method isn't working
        	//we don't want to throw an error since logging failure is not a reason to stop the app
        }
    }
    public void addLine(String s) {
        if (lineBuffer.length()>0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(s);
        newLine();
    }
    
    public void close() {
        try {
        	System.out.println("Closing writer");
            writer.close();
        }
        catch (IOException e) {
        	//not much we can do here
        	//we can't log the error since the close method isn't working
        	//we don't want to throw an error since logging failure is not a reason to stop the app
        }
    }

    @Override
    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }
}
