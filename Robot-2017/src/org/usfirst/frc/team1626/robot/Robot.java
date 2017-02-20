package org.usfirst.frc.team1626.robot;

import java.lang.reflect.InvocationTargetException;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;
import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotDrive;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * Talon VII :: Falcon Robotics Team 1626
 *  - Tankdrive
 *  - Xbox Controller
 *  - Shooter
 *  - Using the winch to climb
 *  - Ball pick up
 *  - Autonomous Echo code
 *  - Brownout protection
 *  
 * @author Rohan Mishra & Team 1626 Falcon Robotics
 * @version 1.0.0
 * 
 */

public class Robot extends IterativeRobot {
	private PowerDistributionPanel pdp;
	
	public CANTalon frontLeft;
	public CANTalon frontRight;
	public CANTalon backLeft;
	public CANTalon backRight;
	
	private RobotDrive drive;

	private XboxController xbox;
	private Joystick driveLeft;
	private Joystick driveRight;
	
	private Talon winchTalon;
	private Talon pickUpTalon;
	private Talon agitatorLeft;
//	private Talon agitatorRight;
	
	private CANTalon shooterTalonOneTop;
	private CANTalon shooterTalonOneBottom;
	private CANTalon shooterTalonTwoTop;
	private CANTalon shooterTalonTwoBottom;
	
	private DoubleSolenoid gearShifter;
	private boolean highGear = true;
	
	private AnalogInput pressureSensor;
	
	int autoLoopCounter;
	ActionRecorder actions;
		
	@Override
	public void robotInit() {
		pdp               		 = new PowerDistributionPanel(0);
		
		frontLeft         		 = new CANTalon(3);
		frontRight         	  	 = new CANTalon(10);
		backLeft          		 = new CANTalon(11);
		backRight         		 = new CANTalon(1);
		
		drive            		 = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
		driveLeft         		 = new Joystick(0);
		driveRight		  		 = new Joystick(1);
		
		xbox               		 = new XboxController(2);
		
		winchTalon               = new Talon(0);
		pickUpTalon    		 	 = new Talon(1);
		agitatorLeft			 = new Talon(2);
//		agitatorRight			 = new Talon(2);
		// TODO - Change ids & implement velocity closed loop control
		shooterTalonOneTop       = new CANTalon(4);
		shooterTalonOneBottom    = new CANTalon(6);
		shooterTalonTwoTop       = new CANTalon(2);
		shooterTalonTwoBottom    = new CANTalon(5);
		
		gearShifter       		 = new DoubleSolenoid(4, 5);
		
		pressureSensor    		 = new AnalogInput(0);
		
		actions 		   		 = new ActionRecorder();
		
		shooterTalonOneBottom.setInverted(true);
		// TODO - Robot disabled = low gear, this sets it into high gear
		gearShifter.set(DoubleSolenoid.Value.kReverse);
		actions.setMethod(this, "robotOperation", DriverInput.class).
			setUpButton(xbox, 1).
			setDownButton(xbox, 2).
			setRecordButton(xbox, 3);
		// TODO - Add Gear-Shifter input
		DriverInput.nameInput("Driver-Left");
		DriverInput.nameInput("Driver-Right");
		DriverInput.nameInput("Driver-Left-Trigger");
		DriverInput.nameInput("Driver-Right-Trigger");
		DriverInput.nameInput("Operator-Left-Trigger");
		DriverInput.nameInput("Operator-Right-Trigger");
		DriverInput.nameInput("Operator-X-Button");
		DriverInput.nameInput("Operator-A-Button");
		DriverInput.nameInput("Operator-B-Button");
		DriverInput.nameInput("Operator-Y-Button");
		DriverInput.nameInput("Operator-Start-Button");
		DriverInput.nameInput("Operator-Back-Button");
	} 
	
	@Override
	public void robotPeriodic() {
		drive.tankDrive(driveLeft.getRawAxis(1), driveRight.getRawAxis(1));
		
		// given vout, pressure = 250(vout/vcc) - 25
		// vcc is assumed to be 5.0
		double pressure = (250.0 * (pressureSensor.getVoltage() / 5.0)) - 25;
		SmartDashboard.putString("DB/String 4", String.format("%.1f", pressure));
		SmartDashboard.putNumber("PDP Voltage", pdp.getVoltage());
	}
	
	@Override
	public void autonomousInit() {
		autoLoopCounter = 0;
		actions.autonomousInit();
	}

	@Override
	public void autonomousPeriodic() {
		try {
			if (actions != null) {
//				actions.playback();
				actions.longPlayback(this, -1);
			} else {
				Timer.delay(0.010);
			}
		} catch (Exception e) {
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
		if (xbox.getTrigger(Hand.kRight)) {
			shooterTalonOneTop.set(.99);
			shooterTalonOneBottom.set(.99);
			shooterTalonTwoTop.set(.99);
			shooterTalonTwoBottom.set(.99);
		} else if (xbox.getTrigger(Hand.kLeft)) {
			shooterTalonOneTop.set(-.99);
			shooterTalonOneBottom.set(-.99);
			shooterTalonTwoTop.set(-.99);
			shooterTalonTwoBottom.set(-.99);
		} else {
			shooterTalonOneTop.set(0);
			shooterTalonOneBottom.set(0);
			shooterTalonTwoTop.set(0);
			shooterTalonTwoBottom.set(0);
		}
		
		if (xbox.getXButton()) {
			pickUpTalon.set(.99);
		} else if (xbox.getYButton()) {
			pickUpTalon.set(-.99);
		} else {
			pickUpTalon.set(0);
		}
		
		if (xbox.getAButton()) {
			winchTalon.set(.99);
		} else if (xbox.getBButton()) {
			winchTalon.set(-.99);
		} else {
			winchTalon.set(0);
		}
		
		// TODO - make sure solenoid values are correct (rn kFoward shifts to low gear)		
		if ((driveLeft.getRawButton(1) == true || driveRight.getRawButton(1) == true) && highGear == true) {
			gearShifter.set(DoubleSolenoid.Value.kForward);
			highGear = false;
		} else if ((driveLeft.getRawButton(1) == true || driveRight.getRawButton(1) == true) && highGear == false) {
			gearShifter.set(DoubleSolenoid.Value.kReverse);
			highGear = true;
		}
		
		// RoboRIO Brownout triggers @ 6.8V		
		if (Timer.getMatchTime() >= 7.0) {
			while (pdp.getVoltage() <= 7.2) {
				xbox.setRumble(RumbleType.kLeftRumble, 1.0);
				xbox.setRumble(RumbleType.kRightRumble, 1.0);
				
				if (pdp.getVoltage() > 7.2) {
					// TODO - Reset certain components
					
					break;
				}
			}
		}
		
		try {
			actions.input(new DriverInput()
				.withInput("Driver-Left", driveLeft.getRawAxis(1))
				.withInput("Driver-Right", driveRight.getRawAxis(1))
				.withInput("Driver-Left-Trigger", driveLeft.getRawButton(1))
				.withInput("Driver-Right-Trigger", driveRight.getRawButton(1))
				.withInput("Operator-Left-Trigger", xbox.getTrigger(Hand.kLeft))
				.withInput("Operator-Right-Trigger", xbox.getTrigger(Hand.kRight))
				.withInput("Operator-X-Button", xbox.getXButton())
				.withInput("Operator-Y-Button", xbox.getYButton())
				.withInput("Operator-A-Button", xbox.getAButton())
				.withInput("Operator-B-Button", xbox.getBButton())
				.withInput("Operator-Start-Button", xbox.getStartButton())
				.withInput("Operator-Back-Button", xbox.getBackButton()));
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
	
	@Override
	public void disabledInit() {
		actions.disabledInit();
	}

	@Override
	public void disabledPeriodic() {
		actions.disabledPeriodic();
	}
	
	public void robotOperation(DriverInput input) {
		System.out.println("Operating with: <" + input.toString() + ">");
		
		input.getAxis("Drive-Left");
		input.getAxis("Driver-Right");
		
		if (input.getTrigger("Operator-Right-Trigger")) {
			shooterTalonOneTop.set(.99);
			shooterTalonOneBottom.set(.99);
			shooterTalonTwoTop.set(.99);
			shooterTalonTwoBottom.set(.99);
		} else if (input.getTrigger("Operator-Left-Trigger")) {
			shooterTalonOneTop.set(-.99);
			shooterTalonOneBottom.set(-.99);
			shooterTalonTwoTop.set(-.99);
			shooterTalonTwoBottom.set(-.99);
		} else {
			shooterTalonOneTop.set(0);
			shooterTalonOneBottom.set(0);
			shooterTalonTwoTop.set(0);
			shooterTalonTwoBottom.set(0);
		}
		
		if (input.getButton("Operator-X-Button")) {
			pickUpTalon.set(.99);
		} else if (input.getButton("Operator-Y-Button")) {
			pickUpTalon.set(-.99);
		} else {
			pickUpTalon.set(0);
		}
		
		if (input.getButton("Operator-A-Button")) {
			winchTalon.set(.99);
		} else if (input.getButton("Operator-B-Button")) {
			winchTalon.set(-.99);
		} else {
			winchTalon.set(0);
		}
	}
	
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
}
