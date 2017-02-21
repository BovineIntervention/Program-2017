package org.usfirst.frc.team686.robot;

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
    
    final String defaultAuto = "Drive forward only";
    final String frontPeg = "Put gear on front peg";
    final String rightPeg = "Puts gear on right peg";
    final String leftPeg = "Puts gear on left peg";
    
    final String JEFF = "President's choice";
    final String KAELA = "Rookie's choice";
    final String DEFAULT = "Programmer's choice";
    
    CameraServer server;
    Thread visionThread;
    
	String key;
	NetworkTable piTalk;
	
	Spark intake;     // non-variable speed
	Spark climber;   
	Spark shooterLift;
	Spark shooter;
	
	CANTalon frontLeftDrive;
	CANTalon frontRightDrive;
	CANTalon backLeftDrive;
	CANTalon backRightDrive;
	
	Servo shotGate;
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
        controlLayout = new SendableChooser<>();
        /*  integer array used to change the button scheme
         *  0: Button number for intake
         *  1: Button number for reversing intake
         *  2: Button number for climber
         *  3: Button number for reversing climber
         *  4: Button number for shooter
         *  5: Button number for reversing shooter
         */
        controlChoice = new int[6];
        /*  double array used to store axis values
         *  0: Value of x direction movement
         *  1: Value of y direction movement
         *  2: Value of rotational movement
         */
        moveValues = new double[3];
        
        key = "SmartDashboard";
        
        intake = new Spark(MotorPorts.SPARK_INTAKE);					// Sparks are based on PWM ports, not CAN assignments
        climber = new Spark(MotorPorts.SPARK_CLIMBER);
        shooter = new Spark(MotorPorts.SPARK_SHOOTER);
        
        shotGate = new Servo(MotorPorts.SERVO_SHOT_GATE);
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
    	
    	double id = 255;
    	while(isAutonomous() && isEnabled())
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
    		SmartDashboard.putString("Auto Selected", autoSelected);
    	/*	synchronized (imgLock) 
    		{
    			centerX = this.centerX;
    		}
    		turn = centerX - (480 / 2);
    		
    		myRobot.arcadeDrive((turn*.0040), -turn * 0.0040);
    		*/
    			
    		frontLeftDrive.set(1);
        	frontRightDrive.set(1);
        	backLeftDrive.set(1);
        	backRightDrive.set(1);
    		
    		SmartDashboard.putNumber("Front Left Wheel Encoder Position", frontLeftDrive.getPosition());
    		SmartDashboard.putNumber("Front Right Wheel Encoder Position", frontRightDrive.getPosition());
    		SmartDashboard.putNumber("Back Left Wheel Encoder Position", backLeftDrive.getPosition());
    		SmartDashboard.putNumber("Back Right Wheel Encoder Position", backRightDrive.getPosition());
    		
    		SmartDashboard.putNumber("angle", piTalk.getNumber("angle", 360));
    		SmartDashboard.putNumber("distance", piTalk.getNumber("distance", 0));
    		piTalk.putNumber("id", id);
    		SmartDashboard.putNumber("idTest", piTalk.getNumber("id", 0));
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
    		
		frontLeftDrive.setPID(6, 0, 0, 0, 20, 36, 0);
	   frontRightDrive.setPID(6, 0, 0, 0, 20, 36, 0);
	 	 backLeftDrive.setPID(6, 0, 0, 0, 20, 36, 0);
    	backRightDrive.setPID(6, 0, 0, 0, 20, 36, 0);
    	
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
		
		frontLeftDrive.setCloseLoopRampRate(96);
		frontRightDrive.setCloseLoopRampRate(96);
		backLeftDrive.setCloseLoopRampRate(96);
		backRightDrive.setCloseLoopRampRate(96);
    }

    public void operatorControl() 
    { 
        myRobot.setSafetyEnabled(true);
        boolean climbing = false;
        boolean climbHeld = false;			// whether or not the climb button is currently held down
        boolean climbingReverse = false;
        boolean climbHeldReverse = false;
        boolean shotStart = false;
        
        double spinUpTime = 0;
        final double STICK_TOLERANCE = .2;  // Dead zone around 0 on the controller so the robot doesn't drift
        final double RAMP_RATE = 96;       // Absolute minimum of 1.173 Volts per second. Controls how fast the 
        
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
        	// dead zone on controller to prevent drift. Attempts to move the robot only when the joystick axis is pushed beyond a certain point
        	if ((moveValues[0] > STICK_TOLERANCE || moveValues[0] < -STICK_TOLERANCE) || (moveValues[1] > STICK_TOLERANCE || moveValues[1] < -STICK_TOLERANCE) || (moveValues[2] > STICK_TOLERANCE || moveValues[2] < -STICK_TOLERANCE))
        	{
                frontLeftDrive.setVoltageRampRate(RAMP_RATE);
                backLeftDrive.setVoltageRampRate(RAMP_RATE);
                frontRightDrive.setVoltageRampRate(RAMP_RATE);
                backRightDrive.setVoltageRampRate(RAMP_RATE);
                
        		myRobot.mecanumDrive_Cartesian(moveValues[0], moveValues[1], moveValues[2], 0);
        	}
        	else
        	{
                frontLeftDrive.setVoltageRampRate(1200);
                backLeftDrive.setVoltageRampRate(1200);
                frontRightDrive.setVoltageRampRate(1200);
                backRightDrive.setVoltageRampRate(1200);
        		
        		myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
        	}
        	// Run the intake while the left bumper is pressed or reverse it while the right bumper is pressed
        	if (stick.getRawButton(controlChoice[0]))				// Expect to change this into a toggle
        	{
        		intake.set(-1.0);		// Motors range in value from -1 to 1 
        	}
        	else
        	{
        		if (stick.getRawButton(controlChoice[1]))
        		{
        			intake.set(1.0);
        		}
        		else
        		{
        			intake.stopMotor();		// Must stop or motor will spin indefinitely
        		}
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
        	
        	// Detect the boiler and shoot high goal while the A button is pressed
        	if (stick.getRawButton(controlChoice[4]))
        	{
        		// start custom shooting algorithm using vision detection and angle adjustment
        		shooter.set(0.9);
        		if (!shotStart)
        		{
        			spinUpTime = Timer.getFPGATimestamp();
        		}
        		
        		if (spinUpTime + 500 <= Timer.getFPGATimestamp())
        		{
        			shotGate.set(1);
        		}
        		shotStart = true;
        	}
        	else
        	{
        		if (stick.getRawButton(controlChoice[5]))
        		{
        			shotGate.set(1);
        		    shooter.set(-.5);
        		}
        		else
        		{
        			shotGate.set(0);
        		    shooter.set(0.0);
        		}
        	}
        }
    }
    public void customButtons()
    {
    	String layoutSelection = controlLayout.getSelected();
    	
    	switch (layoutSelection)
    	{
    	case JEFF:
    		controlChoice[0] = Controller.BUTTON_LEFT_BUMPER;
    		controlChoice[1] = Controller.BUTTON_RIGHT_BUMPER;
    		controlChoice[2] = Controller.BUTTON_START;
    		controlChoice[3] = Controller.BUTTON_BACK;
    		controlChoice[4] = Controller.BUTTON_A;
    		controlChoice[5] = Controller.BUTTON_B;
    		break;
    		
    	case KAELA:
    		controlChoice[0] = Controller.BUTTON_LEFT_BUMPER;
    		controlChoice[1] = Controller.BUTTON_RIGHT_BUMPER;
    		controlChoice[2] = Controller.BUTTON_START;
    		controlChoice[3] = Controller.BUTTON_BACK;
    		controlChoice[4] = Controller.BUTTON_A;
    		controlChoice[5] = Controller.BUTTON_B;
    		break;
    		
    	case DEFAULT:
    		controlChoice[0] = Controller.BUTTON_LEFT_BUMPER;
    		controlChoice[1] = Controller.BUTTON_RIGHT_BUMPER;
    		controlChoice[2] = Controller.BUTTON_START;
    		controlChoice[3] = Controller.BUTTON_BACK;
    		controlChoice[4] = Controller.BUTTON_A;
    		controlChoice[5] = Controller.BUTTON_B;
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
