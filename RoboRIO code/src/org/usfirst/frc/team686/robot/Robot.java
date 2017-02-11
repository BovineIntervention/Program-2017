package org.usfirst.frc.team686.robot;

import edu.wpi.first.wpilibj.CANSpeedController.ControlMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc686.reference.Controller;
import frc686.reference.MotorPorts;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

public class Robot extends SampleRobot 
{
    RobotDrive myRobot;
    Joystick stick;
    SendableChooser<String> autoChoice;
    
    final String defaultAuto = "Drive forward only";
    final String frontPeg = "Put gear on front peg";
    final String rightPeg = "Puts gear on right peg";
    final String leftPeg = "Puts gear on left peg";
    
    CameraServer server;
    Thread visionThread;
	double centerX;
	final Object imgLock = new Object();
	int counter;
	
	Spark intake;     // non-variable speed
	Spark climber;   
	Spark shooterLift;
	Spark shooter;
	
	CANTalon frontLeftDrive;
	CANTalon frontRightDrive;
	CANTalon backLeftDrive;
	CANTalon backRightDrive;
	
    public Robot()
    {
    	// Start robot physical mechanics declarations 
    	frontLeftDrive = new CANTalon(MotorPorts.CANTALON_DRIVE_FRONT_LEFT);		// CAN objects are assigned via an ip connection
    	frontRightDrive = new CANTalon(MotorPorts.CANTALON_DRIVE_FRONT_RIGHT);		// 0 is default and should not be used
    	backLeftDrive = new CANTalon(MotorPorts.CANTALON_DRIVE_BACK_LEFT);
    	backRightDrive = new CANTalon(MotorPorts.CANTALON_DRIVE_BACK_RIGHT);
    	
        myRobot = new RobotDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive);  // Drive is based off of CAN speed controllers 1, 2, 3, and 4
        stick = new Joystick(0);
        
        autoChoice = new SendableChooser<>();
        
        intake = new Spark(MotorPorts.SPARK_INTAKE);					// Sparks are based on PWM ports, not CAN assignments
        climber = new Spark(MotorPorts.SPARK_CLIMBER);
        shooterLift = new Spark(MotorPorts.SPARK_SHOOTER_LIFT);
        shooter = new Spark(MotorPorts.SPARK_SHOOTER);
        
        // Start Jeff B.'s vision code
        centerX = 0.0;
        counter = 0;
        
        // Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setBrightness(-255);
        //camera.setExposureManual(0);
		// Set the resolution
		camera.setResolution(640, 480);
      
    }
    
    public void robotInit()
    {
    	autoChoice.addDefault("Drive Forward", defaultAuto);
    	autoChoice.addObject("Front Peg", frontPeg);
    	autoChoice.addObject("Right Peg", rightPeg);
    	autoChoice.addObject("Left Pin", leftPeg);
    	
    	GripPipeline pipeline = new GripPipeline();
    	visionThread = new Thread(() -> {
    		Rect r= new Rect();
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
				
	    		/*System.out.println(pipeline.filterContoursOutput().isEmpty());
				if(pipeline.checkTest().get(0) == "0") {
					System.out.println("Nothing to filter");
					if(pipeline.findContoursOutput().isEmpty() == false) {
						System.out.println("Problem in findContours");
					}
				}
				
				*/
				System.out.println("Counter: " + counter 
								+"/n" + pipeline.checkTest().toString());
				/*
				
				if(counter%100 == 0) {
					Imgcodecs.imwrite("C:\\Users\\jeffrey\\GRIP.jpg", mat);					
				}*/
	    		
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
	            	r = new Rect();
	            }
				
				Imgproc.rectangle(mat, r.br(), r.tl(), new Scalar(255, 255, 255), 2);
				
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
    }
	
    public void autonomous() 
    {	
    	String autoSelected = autoChoice.getSelected();
    	double centerX;
    	while(isAutonomous())
    	{
    		switch (autoSelected)
    		{
    		case frontPeg:
    			
    			break;
    			
    		case rightPeg:
    			
    			break;
    			
    		case leftPeg:
    			
    			break;
    			
    		case defaultAuto:
    			
    			break;
    		}
    		synchronized (imgLock) 
    		{
    			centerX = this.centerX;
    		}
    		double turn = centerX - (480 / 2);
    		
    		myRobot.arcadeDrive((turn*.0040), -turn * 0.0040);
    		
    		frontLeftDrive.changeControlMode(CANTalon.TalonControlMode.Position);
    		frontLeftDrive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    		frontLeftDrive.setPosition(2);
    	}
    }
    
    public void frontPinAuto()
    {
    	
    }

    public void operatorControl() 
    { 
        myRobot.setSafetyEnabled(true);
        boolean climbing = false;
        boolean climbHeld = false;			// whether or not the climb button is currently held down
        final double STICK_TOLERANCE = .2;  // Dead zone around 0 on the controller so the robot doesn't drift
        final double RAMP_RATE = 96;       // Absolute minimum of 1.173 Volts per second. Controls how fast the 
        
        frontLeftDrive.setVoltageRampRate(RAMP_RATE);
        backLeftDrive.setVoltageRampRate(RAMP_RATE);
        frontRightDrive.setVoltageRampRate(RAMP_RATE);
        backRightDrive.setVoltageRampRate(RAMP_RATE);
        
        frontLeftDrive.enableBrakeMode(true);
        backLeftDrive.enableBrakeMode(true);
        frontRightDrive.enableBrakeMode(true);
        backRightDrive.enableBrakeMode(true);
        while (isOperatorControl() && isEnabled())			// Do not use loops inside of this while loop 
        {													// Use if statements, or create a new thread if needed
        	// dead zone on controller to prevent drift. Attempts to move the robot only when the joystick axis is pushed beyond a certain point
        	if ((stick.getRawAxis(Controller.AXIS_LEFT_X) > STICK_TOLERANCE || stick.getRawAxis(Controller.AXIS_LEFT_X) < -STICK_TOLERANCE) || (stick.getRawAxis(Controller.AXIS_LEFT_Y) > STICK_TOLERANCE || stick.getRawAxis(Controller.AXIS_LEFT_Y) < -STICK_TOLERANCE) || ((stick.getRawAxis(Controller.AXIS_RIGHT_TRIGGER) - stick.getRawAxis(Controller.AXIS_LEFT_TRIGGER)) > STICK_TOLERANCE || (stick.getRawAxis(Controller.AXIS_RIGHT_TRIGGER) - stick.getRawAxis(Controller.AXIS_LEFT_TRIGGER)) < -STICK_TOLERANCE))
        	{
        		myRobot.mecanumDrive_Cartesian(stick.getRawAxis(Controller.AXIS_LEFT_X), stick.getRawAxis(Controller.AXIS_LEFT_Y), stick.getRawAxis(Controller.AXIS_RIGHT_TRIGGER) - stick.getRawAxis(Controller.AXIS_LEFT_TRIGGER), 0);
        	}
        	else
        	{
        		myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
        	}
        	// Run the intake while the left bumper is pressed or reverse it while the right bumper is pressed
        	if (stick.getRawButton(Controller.BUTTON_LEFT_BUMPER))				// Expect to change this into a toggle
        	{
        		intake.set(-1.0);		// Motors range in value from -1 to 1 
        	}
        	else
        	{
        		if (stick.getRawButton(Controller.BUTTON_RIGHT_BUMPER))
        		{
        			intake.set(1.0);
        		}
        		else
        		{
        			intake.stopMotor();		// Must stop or motor will spin indefinitely
        		}
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
    }
        
    /**
     * Runs during test mode
     */
    public void test() 
    {
    	LiveWindow.run();
    }
}
