package org.usfirst.frc.team686.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc686.reference.Controller;
import frc686.reference.MotorPorts;

import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

import org.opencv.core.Mat;
// left in case a drawing is needed on the driver's vision
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class Robot extends SampleRobot 
{
    RobotDrive myRobot;
    Joystick stick;
    SendableChooser<String> autoChoice;
    SendableChooser<String> controlLayout;
    int[] controlChoice;
    double[] moveValues;
    
    double moveDelay;
    double speedDrift;
    double spinUpTime;
    
    boolean timeSet;
	boolean shotStart; 
    
    final String defaultAuto = "Drive forward only";
    final String frontPeg = "Put gear on front peg";
    final String rightPeg = "Puts gear on right peg";
    final String leftPeg = "Puts gear on left peg";
    
    final String JEFF = "President's choice";
    final String KAELA = "Rookie's choice";
    final String DEFAULT = "Programmer's choice";
    
    Thread visionThread;
    
	String key;
	NetworkTable piTalk;
	
	Spark climber;   
	
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
        
        driverVision();
        
        autoChoice = new SendableChooser<>();
        controlLayout = new SendableChooser<>();
        /*  integer array used to change the button scheme
         *  0: Button number for intake
         *  1: Button number for reversing intake
         *  2: Button number for climber
         *  3: Button number for reversing climber
         *  4: Button number for shooter
         *  5: Button number for reversing shooter
         */
        controlChoice = new int[8];
        /*  double array used to store axis values
         *  0: Value of x direction movement
         *  1: Value of y direction movement
         *  2: Value of rotational movement
         */
        moveValues = new double[3];
        
        moveDelay = 0;
        speedDrift = 0;
        
        timeSet = false;
        
        key = "SmartDashboard";
        
        climber = new Spark(MotorPorts.SPARK_CLIMBER);
        
    }
    public void robotInit()
    {
    	autoChoice.addDefault("Drive Forward", defaultAuto);
    	autoChoice.addObject("Front Peg", frontPeg);
    	autoChoice.addObject("Right Peg", rightPeg);
    	autoChoice.addObject("Left Pin", leftPeg);
    	SmartDashboard.putData("Auto Choice", autoChoice);
    	
    	controlLayout.addDefault("Testing", DEFAULT);
    	controlLayout.addObject("Jeff", JEFF);
    	controlLayout.addObject("Kaela", KAELA);
    	SmartDashboard.putData("Control Layout", controlLayout);
    	
    	NetworkTable.setServerMode();
    	
    	piTalk = NetworkTable.getTable(key);
    }
    public void autonomous() 
    {	
    	myRobot.setSafetyEnabled(false);
    	String autoSelected = autoChoice.getSelected();
		
    	talonAutoSetup();
    	
    	double turnTime = 0;
    	int part = 1;
    	while(isAutonomous() && isEnabled())
    	{
    		switch (autoSelected)
    		{
    		case frontPeg:
    			// guides the gear onto the peg
    			gearAid();
    			break;
    			
    		case rightPeg:
    			// Moves the robot backwards from the starting wall to the baseline
    			if (part == 1)
    			{
    				frontLeftDrive.set(-6.434); 
    				frontRightDrive.set(-6.434);
    				backLeftDrive.set(-6.434);
    				backRightDrive.set(-6.434);
    				
    				if ((frontLeftDrive.getPosition() + frontRightDrive.getPosition() + backLeftDrive.getPosition() + backRightDrive.getPosition()) / 4 > -6.5 && frontLeftDrive.getPosition() + frontRightDrive.getPosition() + backLeftDrive.getPosition() + backRightDrive.getPosition() / 4 < -6.3)
    				{
    					part = 2;
    				}
    			}
    			else
    			{
	    			// turns 60 degrees to the right (because the robot is orientated backwards)
	    			if (part == 2)
	    			{
	    				frontLeftDrive.changeControlMode(CANTalon.TalonControlMode.Speed);
	    				frontRightDrive.changeControlMode(CANTalon.TalonControlMode.Speed);
	    				backLeftDrive.changeControlMode(CANTalon.TalonControlMode.Speed);
	    				backRightDrive.changeControlMode(CANTalon.TalonControlMode.Speed);
	    			
	    				frontLeftDrive.set(-.3611);
	    				frontRightDrive.set(.1806);
	    				backLeftDrive.set(-.4583);
	    				backRightDrive.set(.3241);
	    				
	    				if(turnTime == 0)
	    				{
	    					turnTime = Timer.getFPGATimestamp();
	    				}
	    				else
	    				{
	    					if (turnTime + 3 < Timer.getFPGATimestamp())
	    					{
	    						part = 3;
	    					}
	    				}
	    			}
	    			else
	    			{
	    				if (part == 3)
	    				{
	    					gearAid();  // guides the gear onto the peg
	    				}
	    			}
    			}
    			break;
    			
    		case leftPeg:
    			// Moves the robot backwards from the starting wall to the baseline
    			if (part == 1)
    			{
    				frontLeftDrive.set(-6.434); 
    				frontRightDrive.set(-6.434);
    				backLeftDrive.set(-6.434);
    				backRightDrive.set(-6.434);
    				
    				if ((frontLeftDrive.getPosition() + frontRightDrive.getPosition() + backLeftDrive.getPosition() + backRightDrive.getPosition()) / 4 > -6.5 && frontLeftDrive.getPosition() + frontRightDrive.getPosition() + backLeftDrive.getPosition() + backRightDrive.getPosition() / 4 < -6.3)
    				{
    					part = 2;
    				}
    			}
    			else
    			{
	    			// turns 60 degrees to the left (because the robot is orientated backwards)
	    			if (part == 2)
	    			{
	    				frontLeftDrive.changeControlMode(CANTalon.TalonControlMode.Speed);
	    				frontRightDrive.changeControlMode(CANTalon.TalonControlMode.Speed);
	    				backLeftDrive.changeControlMode(CANTalon.TalonControlMode.Speed);
	    				backRightDrive.changeControlMode(CANTalon.TalonControlMode.Speed);
	    			
	    				frontLeftDrive.set(.3611);
	    				frontRightDrive.set(-.1806);
	    				backLeftDrive.set(.4583);
	    				backRightDrive.set(-.3241);
	    				
	    				if(turnTime == 0)
	    				{
	    					turnTime = Timer.getFPGATimestamp();
	    				}
	    				else
	    				{
	    					if (turnTime + 3 < Timer.getFPGATimestamp())
	    					{
	    						part = 3;
	    					}
	    				}
	    			}
	    			else
	    			{
	    				if (part == 3)
	    				{
	    					gearAid();  // guides the gear onto the peg
	    				}
	    			}
    			}
    			break;
    			
    		case defaultAuto:
    			// Moves the robot backwards over the baseline
    			frontLeftDrive.set(-7); 
    			frontRightDrive.set(-7);
    			backLeftDrive.set(-7);
    			backRightDrive.set(-7);
    			break;
    		}
    		SmartDashboard.putString("Auto Selected", autoSelected);
    			
    		SmartDashboard.putNumber("Front Left Wheel Encoder Position", frontLeftDrive.getPosition());
    		SmartDashboard.putNumber("Front Right Wheel Encoder Position", frontRightDrive.getPosition());
    		SmartDashboard.putNumber("Back Left Wheel Encoder Position", backLeftDrive.getPosition());
    		SmartDashboard.putNumber("Back Right Wheel Encoder Position", backRightDrive.getPosition());
    		
    		SmartDashboard.putNumber("pegAngle", piTalk.getNumber("pegAngle", -11));
    	}
    }
    public void talonAutoSetup()
    {
    	frontLeftDrive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		frontRightDrive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		frontRightDrive.reverseSensor(true);
		backLeftDrive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		backRightDrive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		backRightDrive.reverseSensor(true);
    		
		frontLeftDrive.setPID(2, .02, .3, 0, 20, 36, 0);
	    frontRightDrive.setPID(2.5, .02, .25, 0, 50, 48, 0);
	 	backLeftDrive.setPID(2.55, .0255, .26, 0, 20, 36, 0);
    	backRightDrive.setPID(2.35, .0235, .235, 0, 20, 36, 0);
    	
    	frontLeftDrive.changeControlMode(CANTalon.TalonControlMode.Position);
		frontRightDrive.changeControlMode(CANTalon.TalonControlMode.Position);
		backLeftDrive.changeControlMode(CANTalon.TalonControlMode.Position);
		backRightDrive.changeControlMode(CANTalon.TalonControlMode.Position);
		
		frontLeftDrive.setPosition(0);
		frontRightDrive.setPosition(0);
		backLeftDrive.setPosition(0);
		backRightDrive.setPosition(0);
		
		frontLeftDrive.configEncoderCodesPerRev(255/4);
		frontRightDrive.configEncoderCodesPerRev(255/4);
		backLeftDrive.configEncoderCodesPerRev(255/4);
		backRightDrive.configEncoderCodesPerRev(255/4);
		
		frontLeftDrive.configMaxOutputVoltage(14);
		frontRightDrive.configMaxOutputVoltage(14);
		backLeftDrive.configMaxOutputVoltage(14);
		backRightDrive.configMaxOutputVoltage(14);
    }
    public void operatorControl() 
    { 
        myRobot.setSafetyEnabled(true);
        boolean climbing = false;
        boolean climbHeld = false;			// whether or not the climb button is currently held down
        boolean climbingReverse = false;
        boolean climbHeldReverse = false;
        boolean override = false;           // removes the ability to manually drive the robot
       
        final double STICK_TOLERANCE = .2;  // Dead zone around 0 on the controller so the robot doesn't drift
        final double RAMP_RATE = 96;       // Absolute minimum of 1.173 Volts per second. Controls how fast the voltage increases or decreases to the motor
        
        customButtons();
        
        frontLeftDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		frontRightDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		backLeftDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		backRightDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        
        frontLeftDrive.enableBrakeMode(true);
        backLeftDrive.enableBrakeMode(true);
        frontRightDrive.enableBrakeMode(true);
        backRightDrive.enableBrakeMode(true);
        
        while (isOperatorControl() && isEnabled())			// Do not use loops inside of this while loop 
        {													// Use if statements, or create a new thread if needed
        	customAxis();
        	SmartDashboard.putNumber("pegAngle", piTalk.getNumber("pegAngle", -11));
    		
        	// dead zone on controller to prevent drift. Attempts to move the robot only when the joystick axis is pushed beyond a certain point
		    if ((!override && (moveValues[0] > STICK_TOLERANCE || moveValues[0] < -STICK_TOLERANCE) || (moveValues[1] > STICK_TOLERANCE || moveValues[1] < -STICK_TOLERANCE) || (moveValues[2] > STICK_TOLERANCE || moveValues[2] < -STICK_TOLERANCE)))
    	    {
                frontLeftDrive.setVoltageRampRate(RAMP_RATE);
                backLeftDrive.setVoltageRampRate(RAMP_RATE);
                frontRightDrive.setVoltageRampRate(RAMP_RATE);
                backRightDrive.setVoltageRampRate(RAMP_RATE);
                
        		frontLeftDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    			frontRightDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    			backLeftDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    			backRightDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
                
    	    	myRobot.mecanumDrive_Cartesian(moveValues[0], moveValues[1], moveValues[2], 0);
    	    }
    	    else
    	    {
                frontLeftDrive.setVoltageRampRate(1200);
                backLeftDrive.setVoltageRampRate(1200);
                frontRightDrive.setVoltageRampRate(1200);
                backRightDrive.setVoltageRampRate(1200);
                
        		frontLeftDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    			frontRightDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    			backLeftDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    			backRightDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	    	
    	    	myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
    	    }
    	    
        	// Toggle running the climber with the start button
        	if (stick.getRawButton(controlChoice[2]))
        	{
        		if (!climbing && !climbHeld)
        		{
        			// start custom climbing algorithm using peg detection
        			climbing = true;
        			climbingReverse = false;
        			climbHeld = true;
        			
        			climber.set(1);
        		}
        		else
        		{
        			if (climbing && !climbHeld)
        			{
        				// stop detecting peg and attempting to climb
        				climbing = false;
        				climbingReverse = false;
        				climbHeld = true;
        				
        				climber.set(0);
        			}
        		}
        	}
        	else
        	{
        		climbHeld = false;
        	}
        	
        	// Toggle running the climber in reverse with the select button
        	if (stick.getRawButton(controlChoice[3]))
        	{
        		if (!climbingReverse && !climbHeldReverse)
        		{
        			// start custom climbing algorithm using peg detection
        			climbing = false;
        			climbingReverse = true;
        			climbHeldReverse = true;
        			
        			climber.set(-1);
        		}
        		else
        		{
        			if (climbingReverse && !climbHeldReverse)
        			{
        				// stop detecting peg and attempting to climb
        				climbing = false;
        				climbingReverse = false;
        				climbHeldReverse = true;
        				
        				climber.set(0);
        			}
        		}
        	}
        	else
        	{
        		climbHeldReverse = false;
        	}
        	
        	// for assistance in placing a gear on the peg
        	if (stick.getRawButton(controlChoice[6]))
        	{
        		frontLeftDrive.setVoltageRampRate(RAMP_RATE);
                backLeftDrive.setVoltageRampRate(RAMP_RATE);
                frontRightDrive.setVoltageRampRate(RAMP_RATE);
                backRightDrive.setVoltageRampRate(RAMP_RATE);
        		
                override = true;
                
                gearAid();
        	}
        	else
        	{
    			override = false;
        	}
        }
    }
    public void customButtons()
    {
    	String layoutSelection = controlLayout.getSelected();
    	
    	switch (layoutSelection)
    	{
    	case JEFF:
    		controlChoice[0] = Controller.BUTTON_RIGHT_BUMPER;
    		controlChoice[1] = Controller.BUTTON_LEFT_BUMPER;
    		controlChoice[2] = Controller.BUTTON_START;
    		controlChoice[3] = Controller.BUTTON_BACK;
    		controlChoice[4] = Controller.BUTTON_A;
    		controlChoice[5] = Controller.BUTTON_B;
    		controlChoice[6] = Controller.BUTTON_Y;
    		controlChoice[7] = Controller.BUTTON_X;
    		break;
    		
    	case KAELA:
    		controlChoice[0] = Controller.BUTTON_RIGHT_BUMPER;
    		controlChoice[1] = Controller.BUTTON_LEFT_BUMPER;
    		controlChoice[2] = Controller.BUTTON_START;
    		controlChoice[3] = Controller.BUTTON_BACK;
    		controlChoice[4] = Controller.BUTTON_A;
    		controlChoice[5] = Controller.BUTTON_B;
    		controlChoice[6] = Controller.BUTTON_Y;
    		controlChoice[7] = Controller.BUTTON_X;
    		break;
    		
    	case DEFAULT:
    		controlChoice[0] = Controller.BUTTON_RIGHT_BUMPER;
    		controlChoice[1] = Controller.BUTTON_LEFT_BUMPER;
    		controlChoice[2] = Controller.BUTTON_START;
    		controlChoice[3] = Controller.BUTTON_BACK;
    		controlChoice[4] = Controller.BUTTON_A;
    		controlChoice[5] = Controller.BUTTON_B;
    		controlChoice[6] = Controller.BUTTON_Y;
    		controlChoice[7] = Controller.BUTTON_X;
    		break;
    	}
    	
    	SmartDashboard.putString("Layout Selected", layoutSelection);
    }
    public void customAxis()

    {
    	String layoutSelection = controlLayout.getSelected();
    	
    	switch (layoutSelection)
    	{
    	case JEFF:
    		moveValues[0] = stick.getRawAxis(Controller.AXIS_LEFT_X);
    		moveValues[1] = stick.getRawAxis(Controller.AXIS_LEFT_Y);
    		moveValues[2] = stick.getRawAxis(Controller.AXIS_RIGHT_TRIGGER) - stick.getRawAxis(Controller.AXIS_LEFT_TRIGGER);
    		break;
    		
    	case KAELA:
    		moveValues[0] = stick.getRawAxis(Controller.AXIS_LEFT_X);
    		moveValues[1] = stick.getRawAxis(Controller.AXIS_LEFT_Y);
    		moveValues[2] = stick.getRawAxis(Controller.AXIS_RIGHT_TRIGGER) - stick.getRawAxis(Controller.AXIS_LEFT_TRIGGER);
    		break;
    		
    	case DEFAULT:
    		moveValues[0] = stick.getRawAxis(Controller.AXIS_LEFT_X);
    		moveValues[1] = stick.getRawAxis(Controller.AXIS_LEFT_Y);
    		moveValues[2] = stick.getRawAxis(Controller.AXIS_RIGHT_TRIGGER) - stick.getRawAxis(Controller.AXIS_LEFT_TRIGGER);
    		break;
    	}
    }
    public void gearAid()
    {
     	 double angle = piTalk.getNumber("pegAngle", 0);
    	 //double angle = SmartDashboard.getNumber("angle", 0);          // gets the angle from the SmartDashboard (for testing)
     	double turn = 0;
 		SmartDashboard.putNumber("angle", angle);
 		
 		frontLeftDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		frontRightDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		backLeftDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		backRightDrive.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		
		if (angle > 120)
		{
			turn = angle/240;
		}
		else
		{
			turn = Math.max(-1.0, Math.min(1.0, (piTalk.getNumber("pegAngle", 0)))/60);
		}
		
 		if (angle < 10 && angle > -10 )
		{
 			myRobot.mecanumDrive_Cartesian(0, .2, 0, 0);
		}
 		else
		{
 			myRobot.mecanumDrive_Cartesian(0, .2, turn, 0);
		}
    }
    public void driverVision()  // This is taken directly from the intermediate vision sample code
    {
    	visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setResolution(320, 240);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 320, 240);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
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
				// Put a rectangle on the image
				//Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
				
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
    }
    /**
     * Runs during test mode
     */
    public void test() 
    {
    	boolean wheelPIDTest = false;
    	LiveWindow.run();
    	
    	if (wheelPIDTest)
    	{
    		frontLeftDrive.setPID(2.5, .019, 95, 0, 20, 36, 0);
    	    frontRightDrive.setPID(2.1, .017, 80, 0, 20, 36, 0);
    		backLeftDrive.setPID(2.5, .02, 45, 0, 20, 36, 0);
        	backRightDrive.setPID(2.5, .02, 60, 0, 20, 36, 0);
    	}
    }
}