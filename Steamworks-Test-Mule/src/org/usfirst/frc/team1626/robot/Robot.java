package org.usfirst.frc.team1626.robot;

import java.lang.reflect.InvocationTargetException;

import org.opencv.core.Mat;

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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
	private final double shooterSpeed = 1.0;
	private final double agitatorSpeed = 1.0;
	private final double winchSpeed = 1.0;
	
	private final boolean invertedDrive = false;
	private final boolean debugControls=true;

	private PowerDistributionPanel pdp;

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

	private DoubleSolenoid gearHandler;
	private DoubleSolenoid driveTrainShifter;

	Toggle  highGear;
	private AnalogInput pressureSensor;

	//	Pipeline visionProcessing;
	Thread visionThread;

	int autoLoopCounter;
	ActionRecorder actions;

	private Thread autoThread;

	private boolean autoStarted;

	@Override
	public void robotInit() {
		pdp               		 = new PowerDistributionPanel(0);

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

			shooterOneTopMotor = new CANTalon(4);
			shooterOneBottomMotor = new CANTalon(6);
			shooterTwoTopMotor = new CANTalon(2);
			shooterTwoBottomMotor = new CANTalon(5);

			drive              = new RobotDrive(upperLeft, lowerLeft, upperRight, lowerRight);

			winchTalon         = new Talon(0);
			pickUpOneTalon     = new Talon(1);

			agitatorLeft = new Talon(2);
			agitatorRight = new Talon(3);

			upperLeft.setInverted(invertedDrive);
			lowerLeft.setInverted(invertedDrive);
			upperRight.setInverted(invertedDrive);
			lowerRight.setInverted(invertedDrive);

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
			shooterOneTopMotor = new CANTalon(8);
			shooterOneBottomMotor = shooterOneTopMotor;
			shooterTwoTopMotor = shooterOneTopMotor;
			shooterTwoBottomMotor = shooterOneTopMotor;

		}


		driverLeft 		   = new Joystick(0);
		driverRight 	   = new Joystick(1);

		xbox               = new XboxController(2);

		highGear 			= new Toggle();
		pressureSensor = new AnalogInput(0);

		shooterOneTopMotor.setInverted(true);

		gearHandler			     = new DoubleSolenoid(6, 7);
		driveTrainShifter        = new DoubleSolenoid(4, 5);

		// Robot initially in low gear, this sets it into high gear
		driveTrainShifter.set(DoubleSolenoid.Value.kReverse);

		visionThread = new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(640, 480);

			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("Contours", 640, 480);

			Mat mat = new Mat();

			while (!Thread.interrupted()) {
				if (cvSink.grabFrame(mat) == 0) {
					outputStream.notifyError(cvSink.getError());
					continue;
				}

				//				visionProcessing.process(mat);
			}
			visionThread.setDaemon(true);
			visionThread.start();
		});

		actions 		   = new ActionRecorder();
		actions.setMethod(this, "robotOperation", DriverInput.class).
		setUpButton(xbox, 1).
		setDownButton(xbox, 2).
		setRecordButton(xbox, 3);
		DriverInput.nameInput("Driver-Left");
		DriverInput.nameInput("Driver-Right");
		DriverInput.nameInput("Driver-Left-Trigger");
		DriverInput.nameInput("Driver-Right-Trigger");
		DriverInput.nameInput("Operator-Left-Stick");
		DriverInput.nameInput("Operator-Left-Bumper");
		DriverInput.nameInput("Operator-Left-Trigger");
		DriverInput.nameInput("Operator-Right-Stick");
		DriverInput.nameInput("Operator-Right-Bumper");
		DriverInput.nameInput("Operator-Right-Trigger");
		DriverInput.nameInput("Operator-X-Button");
		DriverInput.nameInput("Operator-Y-Button");
		DriverInput.nameInput("Operator-A-Button");
		DriverInput.nameInput("Operator-B-Button");
		DriverInput.nameInput("Operator-Start-Button");
		DriverInput.nameInput("Operator-Back-Button");

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
	public void robotPeriodic() {		
		// given vout, pressure = 250(vout/vcc) - 25
		// vcc is assumed to be 5.0
		double pressure = (250.0 * (pressureSensor.getVoltage() / 5.0)) - 25;
		SmartDashboard.putString("DB/String 4", String.format("%.1f", pressure));
//		SmartDashboard.putNumber("PDP Voltage", pdp.getVoltage());
//
//		// RoboRIO Brownout triggers @ 6.8V		
//		if (Timer.getMatchTime() >= 15.0) {
//			while (pdp.getVoltage() <= 7.2) {
//				xbox.setRumble(RumbleType.kLeftRumble, 1.0);
//				xbox.setRumble(RumbleType.kRightRumble, 1.0);
//
//				if (pdp.getVoltage() > 7.2) {
//					// TODO - Reset certain components
//
//					break;
//				}
//			}
//		}
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

		try {
			actions.input(new DriverInput()
					.withInput("Driver-Left", driverLeft.getRawAxis(1))
					.withInput("Driver-Right", driverRight.getRawAxis(1))
					.withInput("Driver-Left-Trigger", driverLeft.getRawButton(1))
					.withInput("Driver-Right-Trigger", driverRight.getRawButton(1))
					.withInput("Operator-Left-Trigger", xbox.getTrigger(Hand.kLeft))
					.withInput("Operator-Left-Stick", xbox.getRawAxis(1))
					.withInput("Operator-Left-Bumper", xbox.getBumper(Hand.kLeft))
					.withInput("Operator-Right-Trigger", xbox.getTrigger(Hand.kRight))
					.withInput("Operator-Right-Stick", xbox.getRawAxis(5))
					.withInput("Operator-Right-Bumper", xbox.getBumper(Hand.kRight))
					.withInput("Operator-A-Button", xbox.getAButton())
					.withInput("Operator-B-Button", xbox.getBButton())
					.withInput("Operator-X-Button", xbox.getXButton())
					.withInput("Operator-Y-Button", xbox.getYButton())
					.withInput("Operator-Start-Button", xbox.getStartButton())
					.withInput("Operator-Back-Button", xbox.getBackButton())
					);

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

		double leftAxis = input.getAxis("Driver-Left");
		double rightAxis = input.getAxis("Driver-Right");
		drive.tankDrive(leftAxis, rightAxis);

		boolean shift = (input.getButton("Driver-Right-Trigger") || input.getButton("Driver-Left-Trigger"));
		highGear.setState
		(shift);
//		System.out.println("Gear State: " + highGear.getState() + "Button: " + shift);
		if (highGear.getState()) {
			driveTrainShifter.set(DoubleSolenoid.Value.kForward);
//			controlDebug("Shifter forward(high)");
		} else {
			driveTrainShifter.set(DoubleSolenoid.Value.kReverse);
//			controlDebug("Shifter reverse(low)");
}

		if (input.getButton("Operator-X-Button") == true) {
			shooterOneTopMotor.set(shooterSpeed);
			shooterTwoTopMotor.set(shooterSpeed);
			shooterOneBottomMotor.set(shooterSpeed);
			shooterTwoBottomMotor.set(shooterSpeed);
			agitatorLeft.set(agitatorSpeed);
			agitatorRight.set(agitatorSpeed);
			controlDebug("Shooters Forward (" + shooterSpeed + ")");
		} else if (input.getButton("Operator-Y-Button") == true) {
			shooterOneTopMotor.set(-shooterSpeed);
			shooterTwoTopMotor.set(-shooterSpeed);
			shooterOneBottomMotor.set(-shooterSpeed);
			shooterTwoBottomMotor.set(-shooterSpeed);
			agitatorLeft.set(-agitatorSpeed);
			agitatorRight.set(-agitatorSpeed);
			controlDebug("Shooters reverse");
		} else {
			shooterOneTopMotor.set(0);
			shooterTwoTopMotor.set(0);
			shooterOneBottomMotor.set(0);
			shooterTwoBottomMotor.set(0);
			agitatorLeft.set(0);
			agitatorRight.set(0);
		}

		if (input.getButton("Operator-Right-Bumper")) {
			gearHandler.set(DoubleSolenoid.Value.kForward);
//			controlDebug("Gear handler forward");
		} else if (input.getButton("Operator-Left-Bumper")) {
			gearHandler.set(DoubleSolenoid.Value.kReverse);
//			controlDebug("Gear handler reverse");
		}

		if (input.getButton("Operator-A-Button") == true) {
			pickUpOneTalon.set(pickupSpeed);
			controlDebug("Pickup forward (" + pickupSpeed + ")");
		} else if (input.getButton("Operator-B-Button") == true) {
			pickUpOneTalon.set(-pickupSpeed);
		} else {
			pickUpOneTalon.set(0);
		}


		if (input.getButton("Operator-Start-Button")) {
			winchTalon.set(winchSpeed);
//			controlDebug("Winch foreward");
		} else if (input.getButton("Operator-Back-Button")) {
			winchTalon.set(-winchSpeed);
//			controlDebug("Winch reverse");
		} else {
			winchTalon.set(0);
		}
	}

	private void controlDebug(String string) {
		if (debugControls) {
			System.out.println(string);
		}
		
	}

	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}

