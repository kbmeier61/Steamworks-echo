package org.usfirst.frc.team1626.robot;

import java.lang.reflect.InvocationTargetException;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Talon VII :: Robot
 * 
 * @author Rohan Mishra, Nikhil Sathi, Jonathan Heitz, Mr. Daniel (iDubbbzTv, Jr.) & Team 1626 Falcon Robotics
 * @version 1.0.0
 * 
 */

public class Robot extends IterativeRobot {
	
	private final double pickupSpeed = .90;
	
	private CANTalon upperLeft;
	private CANTalon upperRight;
	private CANTalon lowerLeft;
	private CANTalon lowerRight;
	
	private RobotDrive drive;
	
	private XboxController xbox;
	private Joystick driverLeft;
	private Joystick driverRight;
	
	private Talon pickUpOneTalon;
	
	private CANTalon shooterOneTopMotor;
	private CANTalon shooterOneBottomMotor;
	private CANTalon shooterTwoTopMotor;
	private CANTalon shooterTwoBottomMotor;
	
	private Talon winchTalon;
	
	private Talon agitatorLeft;
	private Talon agitatorRight;
	
	private AnalogInput pneumaticPressure;
	
	private DoubleSolenoid gearShifter;
	Toggle  highGear;
	
	int autoLoopCounter;
	ActionRecorder actions;

	private Thread autoThread;

	private boolean autoStarted;
		
	@Override
	public void robotInit() {
		
		boolean robotHasTalonSRX = false;

		try {
			lowerRight = new CANTalon(1);
			long version = lowerRight.GetFirmwareVersion();
			System.out.println("lowerRight firmware version is: " + version);
			
			if (version > 0) {
				robotHasTalonSRX = true;
			}
		} catch (Exception e) {

		}
		
		if (robotHasTalonSRX) {
			upperLeft          = new CANTalon(3);
			upperRight         = new CANTalon(10);
			lowerLeft           = new CANTalon(11);
			lowerRight          = new CANTalon(1);

			System.out.println("Running with CANTalons");
			
			drive              = new RobotDrive(upperLeft, lowerLeft, upperRight, lowerRight);
			
			winchTalon         = new Talon(0);
			pickUpOneTalon     = new Talon(1);
			
			agitatorLeft = new Talon(2);
			agitatorRight = new Talon(3);
			
		} else {
			System.out.println("Running with old Talons");
			Talon leftFront		= new Talon(0);
			Talon rightFront	= new Talon(1);
			Talon leftRear		= new Talon(2);
			Talon rightRear		= new Talon(3);
			// Reverses Joysticks front to back
			leftFront.setInverted(true);
			rightFront.setInverted(true);
			leftRear.setInverted(true);
			rightRear.setInverted(true);
			drive				= new RobotDrive(leftFront, leftRear, rightFront, rightRear);

			winchTalon         = new Talon(4);
			pickUpOneTalon     = new Talon(5);
			
			agitatorLeft = new Talon(6);
			agitatorRight = new Talon(7);
			
		}
		
		
		driverLeft 		   = new Joystick(0);
		driverRight 	   = new Joystick(1);

		xbox               = new XboxController(2);
		
		highGear 			= new Toggle();
		
		pneumaticPressure  = new AnalogInput(0);
		
		shooterOneTopMotor = new CANTalon(4);
		shooterOneBottomMotor = new CANTalon(6);
		shooterTwoTopMotor = new CANTalon(2);
		shooterTwoBottomMotor = new CANTalon(5);
		
		shooterOneBottomMotor.setInverted(true);
		
		gearShifter        = new DoubleSolenoid(4, 5);

		actions 		   = new ActionRecorder();
		actions.setMethod(this, "robotOperation", DriverInput.class).
			setUpButton(xbox, 1).
			setDownButton(xbox, 2).
			setRecordButton(xbox, 3);
		DriverInput.nameInput("Operator-X-Button");
		DriverInput.nameInput("Operator-Y-Button");
		DriverInput.nameInput("Operator-A-Button");
		DriverInput.nameInput("Operator-B-Button");
		DriverInput.nameInput("Driver-Left");
		DriverInput.nameInput("Driver-Right");
		DriverInput.nameInput("Shift-Input");
		DriverInput.nameInput("Winch-Button");
		DriverInput.nameInput("Winch-Reverse");
		
		
//		DriverInput.nameInput("Shooter-Forward");
//		DriverInput.nameInput("Shooter-Backward");
//		DriverInput.nameInput("Driver-Forward");
//		DriverInput.nameInput("Driver-Backward");
		
		
//        new Thread(() -> {
//            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
//            camera.setResolution(640, 480);
//            
//            CvSink cvSink = CameraServer.getInstance().getVideo();
//            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
//            
//            Mat source = new Mat();
//            Mat output = new Mat();
//            
//            while(!Thread.interrupted()) {
//                cvSink.grabFrame(source);
//                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
//                outputStream.putFrame(output);
//            }
//        }).start();

	}
	
	@Override
	public void autonomousInit() {
		autoLoopCounter = 0;
		actions.autonomousInit();
		autoStarted=false;	
	}
	
	@Override
	public void disabledInit() {
		actions.disabledInit();
		if (autoThread != null) {
			System.out.println("Checking autonomous thread");
			if (autoThread.isAlive()) {
				System.out.println("Interrupting autonomous thread");
				autoThread.interrupt();
			}
			try {
				System.out.println("Joining autonomous thread");
				autoThread.join(100);
			} catch (InterruptedException e) {
				System.out.println("Too long to join autonomous thread");
			}
			if (!autoThread.isAlive()) {
				autoThread = null;
				System.out.println("Autonomous thread terminated");
			}
		}
	}

	@Override
	public void disabledPeriodic() {
		actions.disabledPeriodic();
	}
	
	@Override
	public void autonomousPeriodic() {
		try
		{
			if (actions != null)
			{
//				actions.playback();
//				actions.longPlayback(this, -1);
//				if (autoThread == null) {
//					autoThread = new Thread(actions);
//					autoThread.run();
//				}
				if (!autoStarted) {
					actions.notifierAuto();
					autoStarted = true;
				}
				
			} else
			{
				Timer.delay(0.010);
			}
		} catch (Exception e)
		{
			System.out.println("AP: " + e.toString());
		}
		
		
	}
	
	@Override
	public void teleopInit() {
		DriverInput.setRecordTime();
		actions.teleopInit();
	}

	@Override
	public void teleopPeriodic() {
//		double leftAxisValue = xbox.getRawAxis(2);
//		double rightAxisValue = xbox.getRawAxis(5);
//		drive.tankDrive(leftAxisValue, rightAxisValue);
		
//		// TODO - make sure solenoid values are correct (rn kFoward shifts to low gear)
//		if (xbox.getStickButton() == true && highGear == true) {
//			gearShifter.set(DoubleSolenoid.Value.kForward);
//			highGear = false;
//		} else if (xbox.getStickButton() == true && highGear == false) {
//			gearShifter.set(DoubleSolenoid.Value.kReverse);
//			highGear = true;
//		}
//		
//		if (xbox.getXButton() == true) {
//			shooterOneTalon.set(.99);
//			shooterMotorTwo.set(.99);
//		} else if (xbox.getYButton() == true) {
//			shooterOneTalon.set(-.99);
//			shooterMotorTwo.set(-.99);
//		} else {
//			shooterOneTalon.set(0);
//			shooterMotorTwo.set(0);
//		}
		
		try {
			actions.input(new DriverInput()
					.withInput("Operator-X-Button", xbox.getXButton())
					.withInput("Operator-Y-Button", xbox.getYButton())
					.withInput("Operator-A-Button", xbox.getAButton())
					.withInput("Operator-B-Button", xbox.getBButton())
					.withInput("Driver-Left", driverLeft.getRawAxis(1))
					.withInput("Driver-Right", driverRight.getRawAxis(1))
					.withInput("Shift-Input", driverRight.getRawButton(1))
					.withInput("Winch-Button",  xbox.getBackButton())
					.withInput("Winch-Reverse", xbox.getStartButton()));
				
		} catch (IllegalAccessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IllegalArgumentException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InvocationTargetException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void robotOperation(DriverInput input) {
//		System.out.println("Operating with: <" + input.toString() + ">");
		
		if (input.getButton("Operator-X-Button") == true) {
			shooterOneTopMotor.set(.99);
			shooterTwoTopMotor.set(.99);
			shooterOneBottomMotor.set(.99);
			shooterTwoBottomMotor.set(.99);
			agitatorLeft.set(.99);
			agitatorRight.set(.99);
		} else if (input.getButton("Operator-Y-Button") == true) {
			shooterOneTopMotor.set(-.99);
			shooterTwoTopMotor.set(-.99);
			shooterOneBottomMotor.set(-.99);
			shooterTwoBottomMotor.set(-.99);
			agitatorLeft.set(-.99);
			agitatorRight.set(-.99);
		} else {
			shooterOneTopMotor.set(0);
			shooterTwoTopMotor.set(0);
			shooterOneBottomMotor.set(0);
			shooterTwoBottomMotor.set(0);
			agitatorLeft.set(0);
			agitatorRight.set(0);
		}

		if (input.getButton("Operator-A-Button") == true) {
			pickUpOneTalon.set(pickupSpeed);
		} else if (input.getButton("Operator-B-Button") == true) {
			pickUpOneTalon.set(-pickupSpeed);
		} else {
			pickUpOneTalon.set(0);
		}
		
		double leftAxis = input.getAxis("Driver-Left");
		double rightAxis = input.getAxis("Driver-Right");
		
		boolean shift = input.getButton("Shift-Input");
		highGear.input(shift);
		
//		System.out.println("Gear State: " + highGear.getState() + "Button: " + shift);
		if (highGear.getState()) {
			gearShifter.set(DoubleSolenoid.Value.kForward);
		} else {
			gearShifter.set(DoubleSolenoid.Value.kReverse);
		}
		
		drive.tankDrive(leftAxis, rightAxis);
		
		boolean winch = input.getButton("Winch-Button");
		
//		System.out.println("WinchButton: " + winch);
		
		if (winch) {
			winchTalon.set(-.99);
		} else if (input.getButton("Winch-Reverse")) {
			winchTalon.set(.99);
		} else {
			winchTalon.set(0);
		}
		
		
		if (input.getButton("Winch-Reverse")) {
			
		}
}
	
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}

	@Override
	public void robotPeriodic() {
		double inputAnalogPress = pneumaticPressure.getVoltage();
		SmartDashboard.putString("DB/String 4", String.format("%.1f",(250.0 * (inputAnalogPress / 5.0)) - 25));
	}
}

