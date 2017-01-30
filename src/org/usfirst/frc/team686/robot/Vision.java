package org.usfirst.frc.team686.robot;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Servo;

public class Vision {

     double centerX;
	 final Object imgLock = new Object();
	 int counter = 0;
	 Servo cameraServo;
	 GripPipeline pipeline;
	 
	 public void setSettings()
	 	{
		// Get the UsbCamera from CameraServer
	        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	        camera.setBrightness(0);
	        camera.setExposureManual(0);
			  // Set the resolution
			    camera.setResolution(640, 480);
			    camera.setFPS(100);
			    cameraServo.setAngle(1.3);
			
	 	}
	 
	 public void robotVision()
	    {
	    		    Rect r = null;
	    		    Rect r2 = null;
	    		  // Get a CvSink. This will capture Mats from the camera
				      CvSink cvSink = CameraServer.getInstance().getVideo();
				    // Setup a CvSource. This will send images back to the Dashboard
				      CvSource outputStream = CameraServer.getInstance().putVideo("Vision Test", 640, 480);
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
						        // Send the output the error.
						          outputStream.notifyError(cvSink.getError());
						        // skip the rest of the current iteration
						          continue;
					        }
					
					        pipeline.process(mat);
					      // The following prinln's are for debugging purposes and should be removed later
					        //System.out.println(pipeline.filterContoursOutput().size());
					
		    		      /*System.out.println(pipeline.filterContoursOutput().isEmpty());
					        if(pipeline.checkTest().get(0) == "0") 
	                {
						          System.out.println("Nothing to filter");
						          if(pipeline.findContoursOutput().isEmpty() == false) 
	                    {
							              System.out.println("Problem in findContours");
						          }
					        }*/
					
					        /*System.out.println("Brightness: " + camera.getBrightness() 
									  +"/n" + pipeline.checkTest().toString());*/
					
		    		
					        if (!pipeline.filterContoursOutput().isEmpty()) 
					        {
		                  r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
		                 synchronized (imgLock) 
		                 {
		                     centerX = r.x + (r.width / 2);
		                 }
		             
		              }
	                else
	                {
		            	  r = null;
		               	r2 = null;
		              }
					
					        //System.out.println(pipeline.getDualBoxen().size());
					
					        if(pipeline.getDualBoxen().size() !=0) 
	                {
						          r = Imgproc.boundingRect(pipeline.getDualBoxen().get(0));
						          r2 = Imgproc.boundingRect(pipeline.getDualBoxen().get(1));
						          Imgproc.rectangle(mat, r.br(), r2.tl(), new Scalar(255, 255, 255), 2);
					        }
					
					
					        //System.out.println(cameraServo.getAngle());
					
					
					        /*System.out.println("Distance: " + getDistance()
									  +"\nAngle: " + cameraServo.getAngle());*/
									
					      // Give the output stream a new image to display
					        outputStream.putFrame(mat);
	                
					        if(r2 != null && r != null)
	                {
						          adjustCamera((r.y + r2.br().y)/2, cameraServo.getAngle());
					        }
	            }
	         } 
	 
	public void adjustCamera(double yPos, double angle)
    {
    	//System.out.println(yPos);
        if(yPos > 250)
        {
            cameraServo.setAngle(angle-1);
        }
        	
        if(yPos < 230){
          	cameraServo.setAngle(angle+1);
        }
    }

    public double getDistance()
    {
    	  double distance = 0;
    	  double theta = Math.abs(3.3 - cameraServo.getAngle());
    	  double Theight = (42.875) - 16.125;
    	
    	  distance = Theight/Math.tan(Math.PI*(theta)/180);
    	
		    return distance;
    }
	
}
