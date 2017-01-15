package org.usfirst.frc.team686.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import frc686.reference.Controller;

public class Robot extends SampleRobot 
{
    RobotDrive myRobot;
    Joystick stick;
    final int GYRO_TOLERANCE = 1;
    CameraServer server;
    Talon hammer_Alpha;
    Talon hammer_Beta;
    Spark intake;
    boolean toggle, swinging, hammerSet, hammerSetting, switch1, switch2, switch3, power, tick;
    int selection;
	double hammerSpeed, hammerVoltage_Alpha, hammerVoltage_Beta;
    double DESIRED_HAMMER_VELOCITY, HAMMER_VELOCITY_TOLERANCE, HAMMER_POSITION_TOLERANCE, SWUNG_POSITION, HAMMER_SET_POSITION, HAMMER_SET_UPPER_BOUND, HAMMER_SET_LOWER_BOUND;
    DigitalInput dip1, dip2, dip3;
	Ultrasonic ultra1; // left sensor
	Ultrasonic ultra2; // right sensor
	ADXRS450_Gyro gyro;
	AnalogInput ultraSonicSensor1;
    AnalogInput ultraSonicSensor2;
    AnalogInput ultraSonicSensor3;
    AnalogInput ultraSonicSensor4;
    double ultraInches1, ultraInches2, ultraInches3, ultraInches4, frontValue, sideValue;
	double conversionFactor = 0.0394/0.0009977*(1/0.942294);
	Thread visionThread;
	double centerX = 0.0;
	final Object imgLock = new Object();
	int counter = 0;

	
	
    public Robot()
    {
        myRobot = new RobotDrive(0,1);  // Drive is based off of PWM ports 0, 1, 2, and 3
        myRobot.setExpiration(0.1);
        stick = new Joystick(0);
        intake = new Spark(2);
        hammer_Alpha = new Talon(3);
        hammerSpeed = 0;
        DESIRED_HAMMER_VELOCITY = 42;
        HAMMER_VELOCITY_TOLERANCE = 3;
        HAMMER_POSITION_TOLERANCE = 0.5;
        SWUNG_POSITION = 89;
        HAMMER_SET_POSITION = 90;
        HAMMER_SET_UPPER_BOUND = HAMMER_SET_POSITION + HAMMER_POSITION_TOLERANCE;
        HAMMER_SET_LOWER_BOUND = HAMMER_SET_POSITION - HAMMER_POSITION_TOLERANCE;
        switch1 = false;
        switch2 = false;
        switch3 = false;
        selection = 0;
        
        power = false;
        tick = false;
        
        gyro = new ADXRS450_Gyro();
        
        ultraSonicSensor1 = new AnalogInput(0);
        ultraSonicSensor2 = new AnalogInput(1);
        ultraSonicSensor3 = new AnalogInput(2);
        ultraSonicSensor4 = new AnalogInput(3);
        
        dip1 = new DigitalInput(9);
        dip2 = new DigitalInput(8);
        dip3 = new DigitalInput(7);
        
     // Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setBrightness(-255);
        //camera.setExposureManual(0);
		// Set the resolution
		camera.setResolution(640, 480);
      
		
    }
    
    public void robotInit()
    {
    	
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
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
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
	    		
				if (!pipeline.filterContoursOutput().isEmpty()) {
	                r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	                synchronized (imgLock) {
	                    centerX = r.x + (r.width / 2);
	                }
	             
	            } else {
	            	r = new Rect();
	            }
				
				Imgproc.rectangle(mat, r.br(), r.tl(), new Scalar(255, 255, 255), 2);
				
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
				
				incrementCounter();
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
    }
	
    public void autonomous() 
    {	
    	double centerX;
    	while(isAutonomous()==true){
    	
    	synchronized (imgLock) {
    		centerX = this.centerX;
    	}
    	double turn = centerX - (480 / 2);
    	myRobot.arcadeDrive((turn*.0040), -turn * 0.0040);
    	}
    	/*double init;
    	selectAuto();
		switch (selection)
		{
		case(1): // low bar high goal
			gyro.reset();
			myRobot.setSafetyEnabled(false);
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 4.000)
			{
				gyroSteer();
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && gyro.getAngle() < 45)
			{
				convertUltraInches();
				myRobot.tankDrive(0.75, 0);
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && gyro.getAngle() > 0)
			{
				convertUltraInches();
				myRobot.tankDrive(0, 0.75);
			}
			// While the average of the two ultrasonic sensors is greater than 36 inches and they're reasonably close, drive straight.
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			convertUltraInches();
			while(isAutonomous() && (((112.3/194.5)*(sideValue + 26) + frontValue + 0.625) > 112.3)) //CHANGE THIS!!! Below that imaginary line
			{
				gyroSteer();
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			SmartDashboard.putNumber("Front Value at Stop: ", frontValue);
			SmartDashboard.putNumber("Side Value at Stop: ", sideValue);
			Timer.delay(0.125);
			while(isAutonomous() && gyro.getAngle() < 52.50) // While the robot is not pointed 60 degrees to the right, turn right.
			{
				myRobot.tankDrive(0.75, 0.0);
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			//gyro.reset();
			//Timer.delay(0.125);
			//init = Timer.getFPGATimestamp();
			//while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 0.50)
			//{
			//	gyroSteer();
			//	convertUltraInches();
			//}
			//while(isAutonomous() && (frontValue > 54.00))//Fwd until just at edge of batters
			//{
			//	gyroSteer();
			//	convertUltraInches();
			//}
			//myRobot.tankDrive(0.0, 0.0);
		
			if (!isAutonomous())
			{
				break;
			}
			
			autoHighShot();
			
			break;
		case(2): //Position 2 High Goal
			gyro.reset();
			myRobot.setSafetyEnabled(false);
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 4.000)
			{
				gyroSteer();
				convertUltraInches();
			}	
			while(isAutonomous() && gyro.getAngle() > -45)
			{
				convertUltraInches();
				myRobot.tankDrive(0, 0.75);
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && gyro.getAngle() < 0)
			{
				convertUltraInches();
				myRobot.tankDrive(0.75, 0);
			}	
			// While the average of the two ultrasonic sensors is greater than 36 inches and they're reasonably close, drive straight.
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && (((112.3/194.5)*(sideValue + 26) + frontValue + 0.625) > 112.3)) //CHANGE THIS!!! Below that imaginary line
			{
				gyroSteer();
				convertUltraInches();
			}	
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && gyro.getAngle() < 52.50) // While the robot is not pointed 60 degrees to the right, turn right.
			{
				myRobot.tankDrive(0.75, 0.0);
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			gyro.reset();
			Timer.delay(0.125);
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 0.50)
			{
				gyroSteer();
				convertUltraInches();
			}	
			while(isAutonomous() && (frontValue > 54.00))//Fwd until just at edge of batters
			{
				gyroSteer();
				convertUltraInches();
			}	
			myRobot.tankDrive(0.0, 0.0);
			
			if (!isAutonomous())
			{
				break;
			}
			
			autoHighShot();
			
			break;
		case(3): //Position 3 High Goal
			gyro.reset();
			myRobot.setSafetyEnabled(false);
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 4.500)
			{
				gyroSteer();
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && gyro.getAngle() < 110.00) //CALCULATE THIS!!!
			{
				convertUltraInches();
				myRobot.tankDrive(0.75, 0);
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && gyro.getAngle() > 0)
			{
				convertUltraInches();
				myRobot.tankDrive(0, 0.75);
			}
			
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 2.500)
			{
				fastGyroSteer();
				convertUltraInches();
			}
			
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			
			gyro.reset();
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 1.500)
			{
				backwardsGyroSteer();
				convertUltraInches();
			}
			
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			
			if (!isAutonomous())
			{
				break;
			}
			
			autoHighShot();
		
			break;
		case(4): //Position 4 High Shot
			gyro.reset();
			myRobot.setSafetyEnabled(false);
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 4.000)
			{
				gyroSteer();
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && gyro.getAngle() > -52.50) //CALCULATE THIS!!!
			{
				convertUltraInches();
				myRobot.tankDrive(0.0, 0.75);
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && gyro.getAngle() < 0)
			{
				convertUltraInches();
				myRobot.tankDrive(0.75, 0.0);
			}
			
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 2.500)
			{
				fastGyroSteer();
				convertUltraInches();
			}
			
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			
			gyro.reset();
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 1.500)
			{
				backwardsGyroSteer();
				convertUltraInches();
			}
			
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			
			if (!isAutonomous())
			{
				break;
			}
			
			autoHighShot();
		
			break;
			
		case(5): //Position 5 High Shot
			gyro.reset();
			myRobot.setSafetyEnabled(false);
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 4.500)
			{
				gyroSteer();
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && gyro.getAngle() < 90.0) //CALCULATE THIS!!!
			{
				convertUltraInches();
				myRobot.tankDrive(0.0, -0.75);
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			gyro.reset();
			Timer.delay(0.125);
			double startDistance = frontValue;
			
			while(isAutonomous() && frontValue < (startDistance + 64.00))
			{
				backwardsGyroSteer();
				convertUltraInches();
			}
			
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			
			while(isAutonomous() && gyro.getAngle() > 0.0) //CALCULATE THIS!!!
			{
				convertUltraInches();
				myRobot.tankDrive(0.0, 0.75);
			}
			
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 2.500)
			{
				fastGyroSteer();
				convertUltraInches();
			}
			
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			
			gyro.reset();
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && frontValue < 70.00)
			{
				backwardsGyroSteer();
				convertUltraInches();
			}
			
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			
			if (!isAutonomous())
			{
				break;
			}
			
			autoHighShot();
		
			break;
			
		case(6): //Position 5 Low Shot
			gyro.reset();
			myRobot.setSafetyEnabled(false);
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 8.500)
			{
				gyroSteer();
				convertUltraInches();
			}
			while(isAutonomous() && (gyro.getAngle() > -80.00))
			{
				myRobot.tankDrive(-0.75, 0.0);
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 2.000)
			{
				fastGyroSteer();
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			
			myRobot.tankDrive(0.0, -0.75);
			Timer.delay(0.25);
			myRobot.tankDrive(0.0, 0.0);
			
			if (!isAutonomous())
			{
				break;
			}
			
			intake.set(-1);
			hammer_Alpha.set(0.1);
			Timer.delay(2);   // for 2 seconds
			hammer_Alpha.set(0);            // stop hammer
			intake.set(0);
			break;	
		
		case(7): // Low Bar Low Goal
			gyro.reset();
			myRobot.setSafetyEnabled(false);
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 8.500)
			{
				gyroSteer();
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			myRobot.tankDrive(-0.75, -0.75);
			Timer.delay(0.75);
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			while(isAutonomous() && (gyro.getAngle() < 80.00))
			{
				myRobot.tankDrive(0.75, 0.0);
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			Timer.delay(0.125);
			init = Timer.getFPGATimestamp();
			while(isAutonomous() && (Timer.getFPGATimestamp() - init) <= 2.000)
			{
				fastGyroSteer();
				convertUltraInches();
			}
			myRobot.tankDrive(0.0, 0.0);
			
			myRobot.tankDrive(-0.75, 0.0);
			Timer.delay(0.25);
			myRobot.tankDrive(0.0, 0.0);
			
			if (!isAutonomous())
			{
				break;
			}
			
			intake.set(-1);
			hammer_Alpha.set(0.1);
			Timer.delay(2);   // for 2 seconds
			hammer_Alpha.set(0);            // stop hammer
			intake.set(0);
			break;
		
		default: // Default drive forward
			myRobot.setSafetyEnabled(false);
            myRobot.tankDrive(0.75, 0.75);	// drive forwards half speed
            Timer.delay(8);		//    for 2 seconds
            myRobot.tankDrive(0.0, 0.0);
		}*/
			
    }

    public void operatorControl() 
    { 
        myRobot.setSafetyEnabled(true);
        /*while(isOperatorControl() && isEnabled())
        {
        	if(stick.getRawButton(Controller.BUTTON_A))
        	{
        		Timer.delay(.2);
        		hammer_Alpha.set(1);
        		Timer.delay(1);
        	}
        	else
        	{
        		hammer_Alpha.set(0);
        	}
        }*/
        /*
        try {
    		Camera.getImage();
	    } catch (Exception e) {}
         */
        while (isOperatorControl() && isEnabled()) 
        {
            //selectAuto();
            //SmartDashboard.putNumber("Dipswitch value", selection);
        	//convertUltraInches();
        	
    		myRobot.arcadeDrive(-stick.getRawAxis(Controller.AXIS_LEFT_Y), -stick.getRawAxis(Controller.AXIS_LEFT_X));
        	setIntake(stick.getRawButton(Controller.BUTTON_RIGHT_BUMPER), stick.getRawButton(Controller.BUTTON_LEFT_BUMPER));
        	setHammer(stick.getRawAxis(Controller.AXIS_LEFT_TRIGGER) - stick.getRawAxis(Controller.AXIS_RIGHT_TRIGGER), Controller.BUTTON_A, Controller.BUTTON_B, Controller.BUTTON_Y, Controller.BUTTON_X);
        }
        /*
	    try {
        	Camera.stopCapture();
	    } catch (Exception e) {}
	    */
    }
    
    public void overDrive(boolean button)
    {	
    	if (button)
    	{
    		if (!tick && !power)
    		{
    			tick = true;
    			power = true;
    		}
    		else
    		{
    			if (!tick && power)
    			{
    				tick = true;
    				power = true;
    			}
    		}
    	}
    	else
    	{
    		tick = false;
    	}
    }
    
    /**
     * Runs the motors with arcade steering.
     */
        
    // Operates the intake/low output system using two given bumpers
    
    public void convertUltraInches()
    {
    	ultraInches1 = ultraSonicSensor1.getVoltage() * conversionFactor + 0.000;
    	ultraInches2 = ultraSonicSensor2.getVoltage() * conversionFactor + 0.000;
    	ultraInches3 = ultraSonicSensor3.getVoltage() * conversionFactor + 0.000;
    	ultraInches4 = ultraSonicSensor4.getVoltage() * conversionFactor + 0.000;
    	if (ultraInches1 >= 12 && ultraInches2 >= 12)
    	{
    		sideValue = (ultraInches1 + ultraInches2)/2;
    	}
    	else if (ultraInches1 >= 12 && ultraInches2 < 12)
    	{
    		sideValue = ultraInches1;
    	}
    	else if (ultraInches1 < 12 && ultraInches2 >= 12)
    	{
    		sideValue = ultraInches2;
    	}
    	else
    	{
    		
    	}
    	
    	if (ultraInches3 >= 12 && ultraInches4 >= 12)
    	{
    		frontValue = (ultraInches3 + ultraInches4)/2;
    	}
    	else if (ultraInches3 >= 12 && ultraInches4 < 12)
    	{
    		frontValue = ultraInches3;
    	}
    	else if (ultraInches3 < 12 && ultraInches4 >= 12)
    	{
    		frontValue = ultraInches4;
    	}
    	else
    	{
    		
    	}
    	SmartDashboard.putNumber("Side Ultra Rear: ", ultraInches1);
		SmartDashboard.putNumber("Side Ultra Front: ", ultraInches2);
		SmartDashboard.putNumber("Side Value: ", sideValue);
		SmartDashboard.putNumber("Front Ultra Left: ", ultraInches3);
		SmartDashboard.putNumber("Front Ultra Right: ", ultraInches4);
		SmartDashboard.putNumber("Front Value: ", frontValue);
    }
    
    public void towerScan()
    {
    	gyro.reset();
		while(isAutonomous() && gyro.getAngle() > -45)
		{
			convertUltraInches();
			myRobot.tankDrive(-0.75, 0.75);
		}
		myRobot.tankDrive(0.0, 0.0);
		Timer.delay(0.125);
		while(isAutonomous() && gyro.getAngle() < 45)
		{
			myRobot.tankDrive(0.75, -0.75);
		}
    }
    
    public void autoHighShot()
    {
    	intake.set(-1);
		Timer.delay(0.125);
		intake.set(0);
		intake.set(0.75);            // intake to set ball in position
		Timer.delay(0.5);   // for half a second
		intake.set(0);            // stop intake
		Timer.delay(1);
		hammer_Alpha.set(-0.5);          // turn hammer back at half speed to set it
		Timer.delay(0.5);   // for .3 seconds
		hammer_Alpha.set(0);            // stop hammer
		Timer.delay(0.2);   // for .2 seconds
		hammer_Alpha.set(1);            // full swing hammer
		Timer.delay(1);   // for .4 seconds
		hammer_Alpha.set(0);            // stop hammer
    }
    
    public void setIntake(boolean forwardButton, boolean backwardButton)
    {
    	if (forwardButton)
        {
        	intake.set(0.65);
        }
        else if(backwardButton)
        {
        	intake.set(-1);
        }
        else
        {
        	intake.set(0);
        }
    }
    
    // Operates the hammer system based off the given axis
    
    public void setHammer(double speed, int button1, int button2, int button3
    		, int button4)
    {
    	if (stick.getRawButton(button1))
        { 
        	hammer_Alpha.set(1);
        	
        }
        
    	else if (stick.getRawButton(button2))
        { 
        	hammer_Alpha.set(-.5);
        	
        }
        
        else if (stick.getRawButton(button3))
        {
        	hammer_Alpha.set(.1);
        
        }
    	
        else if (stick.getRawButton(button4))
        {
        	hammer_Alpha.set(.45);
        	
        }
    	
        else
        {
        	hammer_Alpha.set(speed);
    		
        }
    }
    
    // Operates only the left motors based off the given axis
    
    public void leftDrive(double speed)
    {
    	myRobot.tankDrive(speed, 0);
    }
    
    // Operates only the left motors based off the given axis
    
    public void rightDrive(double speed)
    {
    	myRobot.tankDrive(0, speed);
    }
    
    public void gyroSteer()
    {
    	if(gyro.getAngle() <= 0 - GYRO_TOLERANCE)
    	{
    		myRobot.tankDrive(0.75, 0.5);
    	}
    	else if(gyro.getAngle() >= 0 + GYRO_TOLERANCE)
    	{
    		myRobot.tankDrive(0.5, 0.75);
    	}
    	else
    	{
    		myRobot.tankDrive(0.75, 0.75);
    	}
    }
    
    public void fastGyroSteer()
    {
    	if(gyro.getAngle() <= 0 - GYRO_TOLERANCE)
    	{
    		myRobot.tankDrive(1.00, 0.66);
    	}
    	else if(gyro.getAngle() >= 0 + GYRO_TOLERANCE)
    	{
    		myRobot.tankDrive(0.66, 1.00);
    	}
    	else
    	{
    		myRobot.tankDrive(1.00, 1.00);
    	}
    }
    
    public void backwardsGyroSteer()
    {
    	if(gyro.getAngle() <= 0 - GYRO_TOLERANCE)
    	{
    		myRobot.tankDrive(-0.50, -0.75);
    	}
    	else if(gyro.getAngle() >= 0 + GYRO_TOLERANCE)
    	{
    		myRobot.tankDrive(-0.75, -0.50);
    	}
    	else
    	{
    		myRobot.tankDrive(-0.75, -0.75);
    	}
    }
    
    public void incrementCounter() {
    	counter++;
    }
    
    
    public void selectAuto()
    {
    	switch1 = !dip3.get();
    	switch2 = !dip2.get();
    	switch3 = !dip1.get();
    	
    	
    	if (switch1 && switch2 && switch3)
		{
			selection = 7;
		}
		else if(switch2 && switch3)
		{
			selection = 6;
		}
		else if(switch1 && switch3)
		{
			selection = 5;
		}
		else if(switch3)
		{
			selection = 4;
		}
		else if (switch1 & switch2)
		{
			selection = 3;
		}
		else if (switch2)
		{
			selection = 2;
		}
		else if (switch1) // position 4 middle shot
		{
			selection = 1;
		}
    	
    	System.out.println(selection);
		
    }
    
    /**
     * Runs during test mode
     */
    public void test() 
    {
    	LiveWindow.run();
    	/*
    	leftDrive(stick.getRawAxis(Controller.AXIS_LEFT_Y));
    	rightDrive(stick.getRawAxis(Controller.AXIS_RIGHT_Y));
    	intake(stick.getRawButton(Controller.BUTTON_LEFT_BUMPER), stick.getRawButton(Controller.BUTTON_RIGHT_BUMPER));
    	hammer(stick.getRawAxis(Controller.AXIS_LEFT_TRIGGER) - stick.getRawAxis(Controller.AXIS_RIGHT_TRIGGER));
    	*/
    }
}
