package org.usfirst.frc.team686.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import frc686.reference.Controller;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Robot extends SampleRobot 
{
    RobotDrive myRobot;
    Joystick stick;
    
    CameraServer server;
    Thread visionThread;
    double centerX;
	  final Object imgLock = new Object();
	  int counter = 0;
	  Servo cameraServo;
	
	  Spark intake;     // non-variable speed
	  Spark conveyor;   // non-variable speed
	  Spark climber;    
	  CANTalon shooter; 
	
	  CANTalon frontLeftDrive;
	  CANTalon frontRightDrive;
	  CANTalon backLeftDrive;
	  CANTalon backRightDrive;
	
    public Robot()
    {
      	// Start robot physical mechanics declarations 
      	frontLeftDrive = new CANTalon(1);		// CAN objects are assigned via an ip connection
      	frontRightDrive = new CANTalon(2);		// 0 is default and should not be used
       	backLeftDrive = new CANTalon(3);
        backRightDrive = new CANTalon(4);
    	  shooter = new CANTalon(5);
      
        myRobot = new RobotDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);  // Drive is based off of CAN speed controllers 1, 2, 3, and 4
        stick = new Joystick(0);
        
        intake = new Spark(0);					// Sparks are based on PWM ports, not CAN assignments
        conveyor = new Spark(1);
        climber = new Spark(2);
        
        cameraServo = new Servo(3);
        centerX = 0.0;
        counter = 0;     
    }
    
    public void robotInit()
    {
    	// Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setBrightness(0);
        camera.setExposureManual(0);
		  // Set the resolution
		    camera.setResolution(640, 480);
		    camera.setFPS(100);
		    cameraServo.setAngle(1.3);
		
    	  GripPipeline pipeline = new GripPipeline();
    	  visionThread = new Thread(() -> 
        { // Consider moving to separate class/mthod
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
        });
		    visionThread.setDaemon(true);
		    visionThread.start();
    }
	
    public void autonomous() 
    {	
    	  myRobot.setSafetyEnabled(false);
    	  double centerX;

      	while(isAutonomous()==true)
        {
      	    synchronized (imgLock) 
            {
    		        centerX = this.centerX;
     	      }
    	      double turn = centerX - (640 / 2);
    	
      	    if(turn >= 150)
            {
                myRobot.tankDrive((turn*.0025), -turn * 0.0025);
    	      }
    	   	
    	      if(turn < 150 && turn > 0)
            {
    		        myRobot.tankDrive((turn*.0035), -turn * 0.0035);
    	      }
    	
    	      if(turn < 0 && turn > -150)
            {
    		        myRobot.tankDrive((turn*.0035), -turn * 0.0035);
    	      }
    	
    	      if(turn <= -150)
            {
    		        myRobot.tankDrive((turn*.0025), -turn * 0.0025);
    	      }    	
    	//System.out.println(turn);
        }    	
    }

    public void operatorControl() 
    { 
        myRobot.setSafetyEnabled(true);
        boolean climbing = false;
        boolean climbHeld = false;			// whether or not the climb button is currently held down
        
        while (isOperatorControl() && isEnabled())			// Do not use loops inside of this while loop 
        {													// Use if statements, or create a new thread if needed
        	  myRobot.mecanumDrive_Cartesian(stick.getRawAxis(Controller.AXIS_LEFT_X), stick.getRawAxis(Controller.AXIS_LEFT_Y), stick.getRawAxis(Controller.AXIS_LEFT_TRIGGER)-stick.getRawAxis(Controller.AXIS_RIGHT_TRIGGER), 0);
        	
        	  // Run the intake while the left bumper is pressed
        	  if (stick.getRawButton(Controller.BUTTON_LEFT_BUMPER))				// Expect to change this into a toggle
        	  {
        		    intake.set(1.0);		// Motors range in value from -1 to 1 
        	  }
        	  else
        	  {
        		    intake.stopMotor();		// Must stop or motor will spin indefinitely 
        	  }
        	
        	  // Run the conveyor while the right bumper is pressed
        	  if (stick.getRawButton(Controller.BUTTON_RIGHT_BUMPER))				// Expect to change this into a toggle
        	  {
        		    conveyor.set(1.0);
        	  }
        	  else
        	  {
        		    conveyor.stopMotor();
        	  }
        	
        	  // Toggle running the climber with the start button
        	  if (stick.getRawButton(Controller.BUTTON_START))
        	  {
        		    if (!climbing && !climbHeld)
        		    {
        			    // start custom climbing algorithm using peg detection
        			      climbing = true;
        			      climbHeld = true;
        		    }
        		    else
        		    {
        			      if (climbing && !climbHeld)
        			      {
        				      // stop detecting peg and attempting to climb
        				        climbing = false;
        				        climbHeld = true;
        			      }
        		    }
        	  }
            else
            {
        	      climbHeld = false;
            }
        	
        	    // Detect the boiler and shoot high goal while the A button is pressed
        	  if (stick.getRawButton(Controller.BUTTON_A))
        	  {
        		  // start custom shooting algorithm using vision detection and angle adjustment
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

    /**
     * Runs during test mode
     */
    public void test() 
    {
    	LiveWindow.run();
    }
}