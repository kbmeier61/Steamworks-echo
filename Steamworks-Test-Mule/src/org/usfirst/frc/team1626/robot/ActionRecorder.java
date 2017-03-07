package org.usfirst.frc.team1626.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**	The {@link ActionRecorder} class provides a record/playback engine for establishing autonomous routines
 *  for an {@link IterativeRobot} by recording the actions of the robot during teleop (normally on a practice
 *  field).  To achieve this, driver inputs are bundled in into a {@link DriverInput}.  The teleoperatedPeriodic
 *  method calls the {@link input} method with a DriverInput.  The input method records the DriverInputs, if 
 *  recording is enabled, and invokes a method that encapsulates all of the robot operations.  This method will
 *  be invoked again with the same inputs during autonomous.  
 */

public class ActionRecorder implements Runnable
{
	private static final String autoDirName = "/home/lvuser/auto";
	private static double ticsPerSecond=1000000.0;
	
	private boolean recording=false;
	private boolean recordingReady=false;
	private long playbackStart;
	private List<DriverInput> driverInputs;
	private Iterator<DriverInput> playbackIterator;
	private Object playbackObject;
	private Method playbackMethod;
	private StateButton upButton;
	private StateButton downButton;
	private StateButton recordButton;
	private List<File> autoFileList;
	private int autoFileIndex;
	private File fileToRecord=new File(autoDirName + "/" + SmartDashboard.getString("DB/String 0", "new_auto.csv"));
	private List<String> details;
	private Notifier nextTask;
	private int tasksDone;
	
	// For timing accuracy measurements
	
	private DateTimeFormatter nameFmt = DateTimeFormatter.ofPattern("yyyyMMddHHmmss");

	private long Sx=0;
	private long Sx2=0;
	private long Sxy=0;
	private long Sy=0;
	private long Sy2=0;
	private long n=0;
	
	public class StateButton
	{
		private int buttonNumber;
		private XboxController controller;
		private boolean prevState;
		
		public StateButton(XboxController stick, int button)
		{
			controller = stick;
			buttonNumber = button;
			prevState = false;
		}
		
		public boolean getState()
		{
			boolean rv=false;
			boolean currState=controller.getRawButton(buttonNumber);
			rv=currState && !prevState;
			prevState=currState;
			return rv;			
		}
	}
	
	private class AutoOperation implements Runnable {
		
		DriverInput input;
		
		public AutoOperation(DriverInput inp) {
			input = inp;
		}

		@Override
		public void run() {
			invokeMethod(input);
			tasksDone++;

			if (playbackIterator.hasNext() &&
					((RobotBase)playbackObject).isAutonomous() && ((RobotBase)playbackObject).isEnabled()) {
//				nextTask();
				input=playbackIterator.next();
				
				double delay = timeToNext(input);
				System.out.println("After task " + tasksDone + " delay is " + delay);
				nextTask.startSingle(delay);
			}
		}
	}
	
	public ActionRecorder()
	{
		recording=false;
		recordingReady=false;
	}

	@SuppressWarnings("rawtypes")
	private Method lookUpMethod(Object obj, String methodName, Class... args)
	{
		Method method=null;
		try
		{
			method = obj.getClass().getMethod(methodName,  args);
		} catch (NoSuchMethodException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (SecurityException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return method;
	}

	@SuppressWarnings("rawtypes")
	public ActionRecorder setMethod(Object obj, String methodName,  Class... args)
	{
		Method method;
		if ((method=lookUpMethod(obj, methodName, args)) != null)
		{
			playbackObject=obj;
			playbackMethod=method;
		}
		return this;
	}

	public ActionRecorder setController(XboxController stick)
	{
		return this;
	}

	public ActionRecorder setUpButton(XboxController stick, int up)
	{
		upButton=new StateButton(stick, up);
		return this;
	}

	public ActionRecorder setDownButton(XboxController stick, int down)
	{
		downButton=new StateButton(stick, down);
		return this;
	}
	public ActionRecorder setRecordButton(XboxController stick, int rec)
	{
		recordButton=new StateButton(stick, rec);
		return this;
	}

	public void startRecording()
	{
		recording=true;
		recordingReady=false;
		SmartDashboard.putBoolean("Auto/Recording", true);
		SmartDashboard.putBoolean("DB/LED 0", true);
	}
	
	public boolean isRecording()
	{
		return recording;
	}

	public void stopRecording()
	{
		recording=false;
		SmartDashboard.putBoolean("Auto/Recording", false);
		SmartDashboard.putBoolean("DB/LED 0", false);
	}

	public void toggleRecording()
	{
		if (isRecording())
		{
			stopRecording();
		} else
		{
			startRecording();
		}
	}
	
	public void displayName()
	{
		String name = "";
		String tmp = "";

		if ((autoFileIndex >= 0) && (autoFileIndex < autoFileList.size())) {

			File file = autoFileList.get(autoFileIndex);
			if ((file != null) && ((tmp=file.getName()) != null)) {
				name = tmp;
			}
		}
		
		SmartDashboard.putString("Auto/FileName", name);
		SmartDashboard.putString("DB/String 0", name);
	}
	
	private int getAutoFileList()
	{
		autoFileList=new ArrayList<File>();
		File autoDir = new File(autoDirName);
		System.out.println("Auto Root is: " + autoDir.getAbsolutePath());
		File[] autoLs = autoDir.listFiles();
		System.out.println("Containing " + autoLs.length + " files");

		int newIdx=-1;

		if (autoLs != null)
		{
			for (File f : autoLs)
			{
				if (f.isFile())
				{
					System.out.println("File<" + f.getAbsolutePath() + ">");
					String fName=f.getName();
					if (fName.matches("new[0-9]+\\.csv"))
					{
						int dotPos = fName.indexOf('.', 3);
						System.out.println("dot pos is " + dotPos);
						if (dotPos > 3)
						{
							String idx = fName.substring(3, dotPos);
							int fNum=Integer.parseInt(idx);
							System.out.println("num<" + idx + ">=" + fNum);

							if (fNum >= newIdx)
							{
								newIdx=fNum+1;
								System.out.println("new index is " + newIdx);
							}
						}
					}
				}
				autoFileList.add(f);
			}
		}
		return newIdx;
	}
	
	private void writeDriverInputs()
	{
		try
		{
			System.out.println("WDI: <" + fileToRecord.getAbsolutePath() + ">");
			BufferedWriter outFile= new BufferedWriter(new FileWriter(fileToRecord));
			
			for (DriverInput input: driverInputs)
			{
				outFile.write(input.toString());
				outFile.write("\n");
			}
			
			outFile.close();
		} catch (IOException e) {
			System.out.println(fileToRecord.getAbsolutePath() + ": " + e.toString());
			e.printStackTrace(System.out);
		}
	}

	/*
	 * m=(n*Sxy - Sx*Sy/(n*Sx2-(Sx)**2)
	 * b=(Sy-b*Sx)/n
	 * s.d. = sqrt((Sx2/n)-(Sx/n)**2)
	 */

	
	public void disabledInit()
	{
		System.out.println("Entering disabledInit");
		
		if (nextTask != null) {
			nextTask.stop();
			nextTask=null;
		}
		
		System.out.println("n=" + n +
				" Sx=" + Sx + " Sx2=" + Sx2 + " Sxy=" + Sxy + " Sy=" + Sy + " Sy2=" + Sy2);
				
		double m=((double)(n*Sxy - Sx*Sy))/(((double)(n*Sx2))-Math.pow((double)Sx,2));
		double b=((double)(Sy-m*Sx))/((double)n);
		double mean = ((double)Sy)/((double)n);
		double sd = Math.sqrt(((double)Sy2/(double)n)-Math.pow(mean,2));

		SmartDashboard.putNumber("Auto/Timing/Slope", m);
		SmartDashboard.putNumber("Auto/Timing/Intercept", b);
		SmartDashboard.putNumber("Auto/Timing/Standard Deviation",  sd);
		SmartDashboard.putNumber("Auto/Timing/Mean",  mean);
		
		if (fileToRecord != null) {
			System.out.println("FileToRecord: <" + fileToRecord.getAbsolutePath() + ">");
		} else {
			System.out.println("FileToRecord is null");
		}
		
		if (isRecording())
		{
			if ((driverInputs != null) && (driverInputs.size() > 0))
			{
				writeDriverInputs();
			}
		}
		
		stopRecording();
		
		getAutoFileList();

		autoFileIndex=0;
		
		String dashboardFileName = SmartDashboard.getString("DB/String 0", "nothing.csv");
		
		for (int i=0; i<autoFileList.size(); i++) {
			String tmp=autoFileList.get(i).getName();
			if (dashboardFileName.equals(tmp)) {
				autoFileIndex=i;
				break;
			}
		}
		
//		autoFileList.add(new File("/home/lvuser/auto", "new" + String.format("%03d.csv", newIdx)));
		displayName();
		
		if (details != null) {
			writeDetails();
			details=null;
		}
	}

	private void writeDetails() {

		try {

			String name = "/home/lvuser/log/" + LocalDateTime.now().format(nameFmt) + ".log";

			BufferedWriter logFile = new BufferedWriter(new FileWriter(name));

			for (String det : details) {
				logFile.write(det + "\n");
			}
			
			logFile.close();
			
		} catch (Exception e) {
			System.err.println("Error writing to logfile" + e.toString());
		}

	}

	public void disabledPeriodic()
	{
		if ((recordButton != null) && recordButton.getState())
		{
			toggleRecording();
		}
		
		if ((upButton != null) && upButton.getState())
		{
			if (autoFileIndex < (autoFileList.size()-1))
			{
				autoFileIndex++;
				displayName();
			}
		}
		
		if ((downButton != null) && downButton.getState())
		{
			if (autoFileIndex > 0)
			{
				autoFileIndex--;
				displayName();
			}
		}

		playbackIterator=null;
	}

	public void input(DriverInput drIn) throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
	{
		if (isRecording())
		{
			if (!recordingReady)
			{
				driverInputs = new ArrayList<DriverInput>();
				recordingReady=true;
			}
			driverInputs.add(drIn);
		}
		playbackMethod.invoke(playbackObject,  drIn);
	}
	
	public void longPlayback(RobotBase robot, int nCycles)
	{
		while (((nCycles < 0) || nCycles > 0) && (robot.isAutonomous() && robot.isEnabled()))
		{
			playback();
			nCycles--;
		}

	}
	
	public void playback()
	{
		if ((driverInputs==null) || (driverInputs.size() == 0))
		{
			System.out.println("No driver inputs to playback");
			Timer.delay(0.050);
			return;
		}
		
		if (playbackIterator == null)
		{
			System.out.println("Creating Iterator for " + driverInputs.size() + " inputs");
			playbackIterator=driverInputs.iterator();
			playbackStart=Utility.getFPGATime();
		}

		if (playbackIterator.hasNext())
		{
			DriverInput input=playbackIterator.next();
			
//			System.out.println("input time offset is " + input.getTimeOffset());

			double delayForPlayback=((double)(playbackStart+input.getTimeOffset() - Utility.getFPGATime()))/1000000.0;
//			System.out.println("Delay before input is " + delayForPlayback);

			if (delayForPlayback > 0)
			{
				Timer.delay(delayForPlayback);
			}
			
			long expectedTime=playbackStart+input.getTimeOffset();
			long timeError=Utility.getFPGATime() - expectedTime;
			
			Sx += expectedTime;
			Sx2 += (expectedTime*expectedTime);
			Sxy += (expectedTime*timeError);
			Sy2 += (timeError*timeError);
			Sy += timeError;
			
			n++;
			

			try {
				String playDetails = String.format("%.6f", (double)expectedTime/1000000.0) + "," + timeError + "," + input.toString();
				details.add(playDetails);
				
				playbackMethod.invoke(playbackObject,  input);
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
		} else
		{
			Timer.delay(0.010);
		}
	}
	
	private void readAutoFile(File autoFile)
	{
		BufferedReader inFile = null;
		try
		{
			inFile = new BufferedReader(new FileReader(autoFile));
			driverInputs=new ArrayList<DriverInput>();
			
			String line;
			while ((line=inFile.readLine()) != null)
			{
//				System.out.println("Line was <" + line + ">");
				String[] tokens = line.split(";");
				int timeOffset = Integer.parseInt(tokens[0]);
				Object[] drIn = new Object[tokens.length-1];
				for (int i=1; i < tokens.length; i++)
				{
					if (tokens[i].equalsIgnoreCase("true") || tokens[i].equalsIgnoreCase("false")) {
						drIn[i-1] = new Boolean(tokens[i]);
					} else if (tokens[i].equalsIgnoreCase("null")) {
						drIn[i-1]=null;
					} else {
						drIn[i-1]=new Double(tokens[i]);
					}
				}
				DriverInput input=new DriverInput(drIn);
				input.setTimeOffset(timeOffset);
				driverInputs.add(input);
			}
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (NumberFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} finally
		{
			if (inFile != null)
			{
				try {
					inFile.close();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}
	
	public void teleopInit()
	{
		if (isRecording())
		{
			String recordFileName = SmartDashboard.getString("DB/String 0", "new_auto.csv");
			File recordFile=new File(autoDirName + "/" + recordFileName);
			
			
//			int fileIndex = 0;
//			for (File file : autoFileList) {
//				if (file.getName().equals(recordFileName)) {
//					recordFile=file;
//					break;
//				}
//			}
			System.out.println("Recording to " + recordFile.getAbsolutePath());
			fileToRecord=recordFile;
		}
	}

	public void autonomousInit()
	{
//		System.out.println("Entering autonomous init with " + autoFileList.get(autoFileIndex).getAbsoluteFile());
		details = new ArrayList<String>();
		File autoFile = autoFileList.get(autoFileIndex);
		if (autoFile.canRead())
		{
//			System.out.println("Reading <" + autoFile.getName() + ">");
			readAutoFile(autoFile);
		}
		if (driverInputs==null)
		{
			System.out.println("No Auto File");
		} else
		{
			System.out.println("Auto File has " + driverInputs.size() + " elements");
			Sx=0;
			Sx2=0;
			Sxy=0;
			Sy=0;
			Sy2=0;
			n=0;

		}
	}
	
	public void notifierAuto() {
		if ((driverInputs==null) || (driverInputs.size() == 0))
		{
			System.out.println("No driver inputs to playback");
			Timer.delay(0.050);
			return;
		}
		
		makeIterator();
		
		if (playbackIterator.hasNext() &&
				((RobotBase)playbackObject).isAutonomous() && ((RobotBase)playbackObject).isEnabled()) {

			tasksDone=0;
			DriverInput input = playbackIterator.next();
			nextTask = new Notifier(new AutoOperation(input));
			nextTask.startSingle(timeToNext(input));
		}
	}

	@Override
	public void run() {
		if ((driverInputs==null) || (driverInputs.size() == 0))
		{
			System.out.println("No driver inputs to playback");
			Timer.delay(0.050);
			return;
		}
		
		if (playbackIterator == null)
		{
			System.out.println("Creating Iterator for " + driverInputs.size() + " inputs");
			playbackIterator=driverInputs.iterator();
			playbackStart=Utility.getFPGATime();
		}

		while (playbackIterator.hasNext() &&
				((RobotBase)playbackObject).isAutonomous() && ((RobotBase)playbackObject).isEnabled() &&
				(!Thread.interrupted())) {
			DriverInput input=playbackIterator.next();
			
//			System.out.println("input time offset is " + input.getTimeOffset());

			double delayForPlayback=((double)(playbackStart+input.getTimeOffset() - Utility.getFPGATime()))/1000000.0;
//			System.out.println("Delay before input is " + delayForPlayback);

			if (delayForPlayback > 0)
			{
				Timer.delay(delayForPlayback);
			}
			
			long expectedTime=playbackStart+input.getTimeOffset();
			long timeError=Utility.getFPGATime() - expectedTime;
			
			Sx += expectedTime;
			Sx2 += (expectedTime*expectedTime);
			Sxy += (expectedTime*timeError);
			Sy2 += (timeError*timeError);
			Sy += timeError;
			
			n++;
			

			try {
				String playDetails = String.format("%.6f", (double)expectedTime/1000000.0) + "," + timeError + "," + input.toString();
				details.add(playDetails);
				
				playbackMethod.invoke(playbackObject,  input);
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
		System.out.println("Autonomous thread ending");
	}
	
	protected void invokeMethod(DriverInput input) {
		long expectedTime=timeOfEvent(input);
		long timeError=Utility.getFPGATime() - expectedTime;
		
		Sx += expectedTime;
		Sx2 += (expectedTime*expectedTime);
		Sxy += (expectedTime*timeError);
		Sy2 += (timeError*timeError);
		Sy += timeError;
		
		n++;
		

		try {
			String playDetails = String.format("%.6f", ((double)expectedTime)/ticsPerSecond) + "," +
					String.format("%.6f", ((double)timeError)/ticsPerSecond) + "," + input.toString();
			details.add(playDetails);
			
			playbackMethod.invoke(playbackObject,  input);
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
	
	public double realTime(long t) {
		return ((double)t) / ticsPerSecond;
	}
	
	private void makeIterator() {
		if (playbackIterator == null)
		{
			System.out.println("Creating Iterator for " + driverInputs.size() + " inputs");
			playbackIterator=driverInputs.iterator();
			playbackStart=Utility.getFPGATime();
		}
	}
	
	public long timeOfEvent(DriverInput input) {
		long expectedTime=playbackStart+input.getTimeOffset();
		return expectedTime;
 
	}
	
	double timeToNext(DriverInput nextInput) {
		double nextTime = realTime(timeOfEvent(nextInput));
		double delay = nextTime - realTime(Utility.getFPGATime());
		return delay;
	}
	
	private void nextTask() {
		DriverInput nextInput = playbackIterator.next();
		
		double nextTime = realTime(timeOfEvent(nextInput));
		double delay = nextTime - realTime(Utility.getFPGATime());
		
		nextTask = new Notifier(new AutoOperation(nextInput));
		nextTask.startSingle(delay);
		
//		System.out.println("Notifier created to start at " + nextTime + " (now " +
//				((double)Utility.getFPGATime())/ticsPerSecond + ")" +
//				" delay " + delay +
//				" playbackStart was " + realTime(playbackStart) +
//				" Event time " + realTime(nextInput.getTimeOffset()));
	}
	
	public boolean notifyActive() {
		return nextTask != null;
	}
}