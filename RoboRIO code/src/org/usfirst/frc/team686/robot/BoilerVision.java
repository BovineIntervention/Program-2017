package org.usfirst.frc.team686.robot;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class BoilerVision{
	
	double centerX = 0.0;
	final Object imgLock = new Object();
		
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		cameraSetUp();
		visionThread();
	}
	
	private static void visionThread() {
		// TODO Auto-generated method stub
		GripPipeline pipeline = new GripPipeline();
		Rect r= new Rect();
		// Get a CvSink. This will capture Mats from the camera
		CvSink cvSink = CameraServer.getInstance().getVideo();
		
		// Mats are very memory expensive. Lets reuse this Mat.
		Mat mat = new Mat();
		// +This cannot be 'true'. The program will never exit if it is. This
		// lets the robot stop this thread when restarting robot code or
		// deploying.
		while (!Thread.interrupted()) 
		{
			// Tell the CvSink to grab a frame from the camera and put it
			// in the source mat.  If there is an error notify the output.
			if (cvSink.grabFrame(mat) == 0) 
			{
				// skip the rest of the current iteration
				continue;
			}
			
			pipeline.process(mat);
			
    		/*System.out.println(pipeline.filterContoursOutput().isEmpty());
			if(pipeline.checkTest().get(0) == "0") {
				System.out.println("Nothing to filter");
				if(pipeline.findContoursOutput().isEmpty() == false) {
					System.out.println("Problem in findContours");
				}
			}
			
			*/
				    		
			if (!pipeline.filterContoursOutput().isEmpty()) 
			{
                r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                
             
            } 
			else 
            {
            	r = new Rect();
            }
			
			
			
			// Give the output stream a new image to display
			
		}
		
    	
	}

	public static void cameraSetUp(){
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setBrightness(-255);
        //camera.setExposureManual(0);
		// Set the resolution
		camera.setResolution(640, 480);
	}
	
	public void test(){
		// Setup a CvSource. This will send images back to the Dashboard
		CvSink cvSink = CameraServer.getInstance().getVideo();
		CvSource outputStream = CameraServer.getInstance().putVideo("Vision Test", 640, 480);
		Mat mat = new Mat();
		Rect r= new Rect();
		
		if (cvSink.grabFrame(mat) == 0) 
		{
			// Send the output the error.
			outputStream.notifyError(cvSink.getError());
		}
		
		Imgproc.rectangle(mat, r.br(), r.tl(), new Scalar(255, 255, 255), 2);
		
		outputStream.putFrame(mat);
	}
	
	public void memeMachine(){
		/*synchronized (imgLock) 
        {
            centerX = r.x + (r.width / 2);
        }*/
	}
}