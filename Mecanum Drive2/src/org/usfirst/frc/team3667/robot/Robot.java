package org.usfirst.frc.team3667.robot;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;

import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team3667.robot.Robot.startingPosition;
import org.usfirst.frc.team3667.robot.Robot.target;

import com.analog.adis16448.frc.ADIS16448_IMU;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	ADIS16448_IMU imu;
	SendableChooser<startingPosition> startingPositionRadio;
	SendableChooser<target> primaryTargetRadio;
	// SendableChooser<target> secondTargetRadio;

	String gameData = "";

	public static final double kDistancePerRevolution = 18.84;
	Command autonomousCommand;
	// for an AS5145B Magnetic Encoder - Bravo
	public static final double kPulsesPerRevolution = 405;
	// US Digital E4T Optical Encoder - Alpha
	// public static final double kPulsesPerRevolution = 256;

	public enum Direction {
		FORWARD, REVERSE, LEFT, RIGHT, CUBEACTION
	};

	public enum AutonPlays {
		driveForwardOnly,
		// Start Left Scale Left
		startLeft_ScaleLeft,
		// Start Left Scale Right
		startLeft_ScaleRight,
		// Start Left Switch Left
		startLeft_SwitchLeft,
		// Start Left Switch Right
		startLeft_SwitchRight,
		// Start Center Switch Left
		startCenter_SwitchLeft,
		// Start Center Switch Right
		startCenter_SwitchRight,
		// Start Right Scale Left
		startRight_ScaleLeft,
		// Start Right Scale Right
		startRight_ScaleRight,
		// Start Right Switch Left
		startRight_SwitchLeft,
		// Start Right Switch Right
		startRight_SwitchRight,
	};

	public enum Action {
		UP, DOWN, OUTTAKE, INTAKE
	};

	public enum startingPosition {
		Left, Center, Right
	}

	public enum target {
		Switch, Scale, OnSideAny, None
	}

	public static final double kDistancePerPulse = kDistancePerRevolution / kPulsesPerRevolution;
	// Encoder Values for 18.
	private Encoder leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
	private Encoder rightEncoder = new Encoder(2, 3, true, EncodingType.k4X);
	private Encoder liftEncoder = new Encoder(4, 5, false, EncodingType.k4X);
	private DigitalInput limitSwitchHigh = new DigitalInput(6);
	private DigitalInput limitSwitchLow = new DigitalInput(7);
	DoubleSolenoid climbShifter = new DoubleSolenoid(0, 1);
	VictorSP _pickupTilt = new VictorSP(1);

	// Encoder Values for 17.
	// private Encoder leftEncoder = new Encoder(0, 1, true, EncodingType.k4X);
	// private Encoder rightEncoder = new Encoder(2, 3, false,
	// EncodingType.k4X);
	// private Encoder liftEncoder = new Encoder(4, 5, false, EncodingType.k4X);
	// Search US Digital Encoder FRC Java E4P Encoder

	Timer autoTime = new Timer();
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(13);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(12);
	WPI_TalonSRX _rearRightMotor = new WPI_TalonSRX(11);
	WPI_TalonSRX _rearLeftMotor = new WPI_TalonSRX(10);
	WPI_TalonSRX _lift = new WPI_TalonSRX(14);
	WPI_TalonSRX _lift2 = new WPI_TalonSRX(15);
	WPI_TalonSRX _pickupLeft = new WPI_TalonSRX(16);
	WPI_TalonSRX _pickupRight = new WPI_TalonSRX(17);
	// WPI_TalonSRX _pickupTilt = new WPI_TalonSRX(9);
	SpeedControllerGroup leftMotors = new SpeedControllerGroup(_frontLeftMotor, _rearLeftMotor);
	SpeedControllerGroup rightMotors = new SpeedControllerGroup(_frontRightMotor, _rearRightMotor);
	DifferentialDrive _drive = new DifferentialDrive(leftMotors, rightMotors);
	Joystick _driveController = new Joystick(0);
	Joystick _cubeController = new Joystick(1);
	int counter = 0;
	long quitinTime = System.currentTimeMillis();

	double desiredCubeHeight = 0;

	int autonStep = 1;
	AutonPlays curPlay = AutonPlays.driveForwardOnly;

	double lastValidDirection = 0;

	AnalogInput auto1 = new AnalogInput(0);
	AnalogInput auto2 = new AnalogInput(1);
	AnalogInput auto3 = new AnalogInput(2);
	AnalogInput auto4 = new AnalogInput(3);

	// This function is run when the robot is first started up and should be
	// used for any initialization code.
	public void robotInit() {
		imu = new ADIS16448_IMU();
		imu.calibrate();
		imu.reset();

		// _drive.setInvertedMotor(MotorType.kFrontRight, true);
		// _drive.setInvertedMotor(MotorType.kRearRight, true);
		// autonomousCommand = new move() ;

		// start of encoders
		// leftEncoder.setDistancePerPulse(kDistancePerPulse);
		// rightEncoder.setDistancePerPulse(kDistancePerPulse);
		// liftEncoder.setDistancePerPulse(kDistancePerPulse);
		leftEncoder.reset();
		rightEncoder.reset();
		liftEncoder.reset();
		leftEncoder.setDistancePerPulse(kDistancePerPulse);
		rightEncoder.setDistancePerPulse(kDistancePerPulse);
		liftEncoder.setDistancePerPulse(kDistancePerPulse);
		// Setup the menu for position selection.
		startingPositionRadio = new SendableChooser<startingPosition>();
		startingPositionRadio.addObject("Left", startingPosition.Left);
		startingPositionRadio.addDefault("Center", startingPosition.Center);
		startingPositionRadio.addObject("Right", startingPosition.Right);
		SmartDashboard.putData("Starting Position", startingPositionRadio);
		// First Target Selector.
		primaryTargetRadio = new SendableChooser<target>();
		primaryTargetRadio.addObject("On Side Any", target.OnSideAny);
		primaryTargetRadio.addObject("Scale", target.Scale);
		primaryTargetRadio.addDefault("Switch", target.Switch);
		// primaryTargetRadio.addDefault("Drive Forward", target.DriveForward);
		SmartDashboard.putData("First Target", primaryTargetRadio);
		// Second Target Selector.
		// secondTargetRadio = new SendableChooser<target>();
		// secondTargetRadio.addObject("On Side Any", target.OnSideAny);
		// secondTargetRadio.addObject("Scale", target.Scale);
		// secondTargetRadio.addObject("Switch", target.Switch);
		// secondTargetRadio.addDefault("None", target.None);
		// SmartDashboard.putData("Second Target", secondTargetRadio);

		/*
		 * UsbCamera camera =
		 * CameraServer.getInstance().startAutomaticCapture();
		 * camera.setResolution(640, 480); CvSink cvsink =
		 * CameraServer.getInstance().getVideo(); CvSource outputStream =
		 * CameraServer.getInstance().putVideo("Blur", 640, 480); Mat source =
		 * new Mat(); Mat output = new Mat(); while(!Thread.interrupted()) {
		 * cvSink.grabFrame(source); Imgproc.cvtColor(source, output,
		 * Imgproc.COLOR_BayerBG2BGR); outputStream.putFrame(output); }
		 */

		// CameraServer.getInstance().startAutomaticCapture();

	}

	public void disabledInit() {

	}

	public void telopInit() {

	}

	// This function is called periodically during operator control
	public void teleopPeriodic() {

		// Update the Smart Dashboard Data
		updateSmartDashboardData();

		// Logic to control the Shifting Solenoid to Low Gear
		if (_cubeController.getRawButton(1)) {
			// climbShifter.set(DoubleSolenoid.Value.kOff);
			climbShifter.set(DoubleSolenoid.Value.kForward);
		}

		// Logic to control the Shifting Solenoid to High Gear
		if (_cubeController.getRawButton(2)) {
			climbShifter.set(DoubleSolenoid.Value.kReverse);
		}

		// Logic for Cube Pickup and Release
		if (_cubeController.getRawButton(5)) {
			_pickupLeft.set(1);
			_pickupRight.set(-1);
		} else if (_cubeController.getRawButton(6)) {
			_pickupLeft.set(-1);
			_pickupRight.set(1);
		} else if (_cubeController.getRawAxis(2) != 0) {
			_pickupLeft.set(_cubeController.getRawAxis(7));
			_pickupRight.set(_cubeController.getRawAxis(7) * -1.0);
		} else {
			_pickupLeft.set(_cubeController.getRawAxis(8) * -1.0);
			_pickupRight.set(_cubeController.getRawAxis(8));
		}

		// Logic to Lift and Lower
		double curLiftVal = _cubeController.getRawAxis(1) * -1.0;
		if (curLiftVal > .2 && !limitSwitchHigh.get()) {
			_lift.set(curLiftVal * 0.9);
			_lift2.set(curLiftVal * 0.9);
		} else if (curLiftVal < .2 && !limitSwitchLow.get()) {
			_lift.set(curLiftVal);
			_lift2.set(curLiftVal);
		} else {
			_lift.set(0);
			_lift2.set(0);
		}
		SmartDashboard.putNumber("Cube Controller", _cubeController.getRawAxis(1));

		// Logic for Cube Tilt
		_pickupTilt.set(_cubeController.getRawAxis(5)); // was 5
		SmartDashboard.putNumber("Pickup Tilt", _cubeController.getRawAxis(5));

		// Logic to drive or "Step Test" Auton
		if (_driveController.getRawButton(4)) {
			testAutonomousPeriodic(); // When the yellow "Y" button is pressed
		} else {
			// Basic logic to drive the robot
			if (!limitSwitchLow.get()) {
				// Slow the robot down when not at low position on lift
				_drive.arcadeDrive(_driveController.getRawAxis(1) * -0.6, _driveController.getRawAxis(4) * 0.6);
			} else {
				// FULL SPEED!!! robot drive (not quite hyper speed though)
				_drive.arcadeDrive(_driveController.getRawAxis(1) * -1, _driveController.getRawAxis(4));
			}
		}

		// Logic to reset sensor for auton testing
		if (_driveController.getRawButton(3)) {
			// When the blue "X" button is pressed on Drive Controller
			initAndResetAll();
		}
	}

	// No code to be added in autonomousInit
	public void autonomousInit() {
		initAndResetAll();
	}

	private void initAndResetAll() {
		imu.reset();
		leftEncoder.reset();
		rightEncoder.reset();
		liftEncoder.reset();
		autonStep = 1;
		lastValidDirection = 0;
		// Get FMS Data to determine ownership sides.
		int retries = 50;
		while (gameData.length() < 3 && retries > 0) {
			SmartDashboard.putString("Current Play:", gameData);
			try {
				Thread.sleep(5);
				gameData = DriverStation.getInstance().getGameSpecificMessage();
				if (gameData == null) {
					gameData = "";
				}
			} catch (Exception e) {
			}
			retries--;
		}
		curPlay = determinePlay();
		updateSmartDashboardData();
	}

	public void autonomousPeriodic() {
		// Update the Smart Dashboard Data
		updateSmartDashboardData();
		// Nothing else should go here
		executeAutonomousCommandCompendium();
	}

	public void testAutonomousPeriodic() {
		executeAutonomousCommandCompendium();
	}

	private Boolean adjustHeightPeriodic(double height) {
		Boolean heightAttained = false;
		if (height > 0 && !limitSwitchHigh.get()) {
			_lift.set(1.0);
			_lift2.set(1.0);
		} else if (height <= 0 && !limitSwitchLow.get()) {
			_lift.set(-1.0);
			_lift2.set(-1.0);
		} else {
			_lift.set(0);
			_lift2.set(0);
			heightAttained = true;
		}
		return heightAttained;
	}

	private void robotAction(Direction driveDirection, double distance, double powerPercent, double height,
			double cubeActionPercent, double minDuration) {
		// Where are we starting from
		double startingLeftEncoder = leftEncoder.getDistance();
		double startingRightEncoder = rightEncoder.getDistance();
		double turnDegree = 0;
		double targetDegree = 0;
		if (minDuration != 0) {
			quitinTime = System.currentTimeMillis() + (long) (minDuration * 1000);
		} else {
			quitinTime = System.currentTimeMillis() + 7000;
		}
		boolean heightAttained = true;
		switch (driveDirection) {
		case CUBEACTION:
			while (System.currentTimeMillis() < quitinTime) {
				adjustHeightPeriodic(height);
				cubeActionPeriodic(cubeActionPercent);
			}
			break;
		case FORWARD:
			while (leftEncoder.getDistance() <= (startingLeftEncoder + distance)
					&& rightEncoder.getDistance() <= (startingRightEncoder + distance) && distance > 0
					&& System.currentTimeMillis() < quitinTime) {
				double correctionRotation = 0;
				if (lastValidDirection - imu.getAngleZ() > 1) {
					correctionRotation = 0.4;
				} else if (imu.getAngleZ() - lastValidDirection > 1) {
					correctionRotation = -0.4;
				}
				if (leftEncoder.getDistance() <= startingLeftEncoder + distance
						&& rightEncoder.getDistance() <= startingRightEncoder + distance) {
					_drive.arcadeDrive(powerPercent * .01, correctionRotation);
				} else if (leftEncoder.getDistance() >= startingLeftEncoder + distance
						&& rightEncoder.getDistance() >= startingRightEncoder + distance) {
					_drive.arcadeDrive(powerPercent * -.01, correctionRotation);
				}
				heightAttained = adjustHeightPeriodic(height);
				cubeActionPeriodic(cubeActionPercent);
			}
			while (!heightAttained && System.currentTimeMillis() < quitinTime) {
				heightAttained = adjustHeightPeriodic(height);
			}
			break;
		case REVERSE:
			double correctionRotation = 0;
			if (lastValidDirection - imu.getAngleZ() > 1) {
				correctionRotation = -0.4;
			} else if (imu.getAngleZ() - lastValidDirection > 1) {
				correctionRotation = 0.4;
			}
			while (leftEncoder.getDistance() >= startingLeftEncoder - distance
					&& rightEncoder.getDistance() >= startingRightEncoder - distance && distance > 0
					&& System.currentTimeMillis() < quitinTime) {
				_drive.arcadeDrive(powerPercent * -.01, correctionRotation);
				heightAttained = adjustHeightPeriodic(height);
				cubeActionPeriodic(cubeActionPercent);
			}
			while (!heightAttained && System.currentTimeMillis() < quitinTime) {
				heightAttained = adjustHeightPeriodic(height);
			}
			break;
		case LEFT:
			if (powerPercent > 85)
				powerPercent = 85;
			targetDegree = lastValidDirection - distance;
			while (targetDegree < imu.getAngleZ() && heightAttained && System.currentTimeMillis() < quitinTime) {
				// check if within 10 degrees and if so slow turn
				if (Math.abs(targetDegree - imu.getAngleZ()) > 10) {
					SmartDashboard.putNumber("speed", powerPercent * -.01);
					_drive.arcadeDrive(0, powerPercent * -.01);
				} else {
					_drive.arcadeDrive(0, -.5);
				}
				heightAttained = adjustHeightPeriodic(height);
				cubeActionPeriodic(cubeActionPercent);
			}
			lastValidDirection -= distance;
			// If we overshot on the turn, than correct
			while (lastValidDirection > imu.getAngleZ()) {
				_drive.arcadeDrive(0, .5);
			}
			while (!heightAttained && System.currentTimeMillis() < quitinTime) {
				heightAttained = adjustHeightPeriodic(height);
			}
			break;
		case RIGHT:
			if (powerPercent > 85)
				powerPercent = 85;
			turnDegree = lastValidDirection + distance;
			while (turnDegree > imu.getAngleZ() && heightAttained && System.currentTimeMillis() < quitinTime) {
				// check if within 10 degrees and if so slow turn
				if (Math.abs(turnDegree - imu.getAngleZ()) > 10) {
					_drive.arcadeDrive(0, powerPercent * .01);
				} else {
					_drive.arcadeDrive(0, .5);
				}
				heightAttained = adjustHeightPeriodic(height);
				cubeActionPeriodic(cubeActionPercent);
			}
			lastValidDirection += distance;
			// If we overshot on the turn, then correct
			while (lastValidDirection < imu.getAngleZ()) {
				_drive.arcadeDrive(0, -.5);
			}
			while (!heightAttained && System.currentTimeMillis() < quitinTime) {
				heightAttained = adjustHeightPeriodic(height);
			}
			break;
		default:
			break;
		}
		_lift.set(0);
		_lift2.set(0);
	}

	private void cubeActionPeriodic(double cubeActionPercent) {
		_pickupLeft.set(cubeActionPercent / 100 * 1);
		_pickupRight.set(cubeActionPercent / 100 * -1);
	}

	private void updateSmartDashboardData() {
		try {
			SmartDashboard.putNumber("encoderL", leftEncoder.getDistance());
			SmartDashboard.putNumber("encoderR", rightEncoder.getDistance());
			SmartDashboard.putNumber("Gyro Angle Z", imu.getAngleZ());
			SmartDashboard.putNumber("Last Valid", lastValidDirection);
			SmartDashboard.putNumber("Auton Step", autonStep);
			SmartDashboard.putNumber("Timer", Timer.getMatchTime());
			SmartDashboard.putNumber("Lift Encoder", liftEncoder.getDistance());
			SmartDashboard.putString("FMS Data", gameData);
			SmartDashboard.putString("Current Play:", curPlay.toString());

			// Analog inputs, for testing and eventual autonomous selection
			SmartDashboard.putNumber("Analog Input I:", auto1.getValue());
			SmartDashboard.putNumber("Analog Input II:", auto2.getValue());
			SmartDashboard.putNumber("Analog Input III:", auto3.getValue());
			SmartDashboard.putNumber("Analog Input IV:", auto4.getValue());
		} catch (Exception exc) {

		}
	}

	private void executeAutonomousCommandCompendium() {
		if (curPlay == AutonPlays.driveForwardOnly) {
			curPlay = determinePlay();
			// This should be done during init, but check again
		}
		// This is the 3667 play book for Autonomous options
		switch (curPlay) {
		case driveForwardOnly:
			driveForwardOnly();
			break;
		case startCenter_SwitchLeft:
			startCenterSwitchLeftNone();
			break;
		case startCenter_SwitchRight:
			startCenterSwitchRightNone();
			break;
		case startLeft_ScaleLeft:
			startLeftScaleLeftNone();
			break;
		case startLeft_ScaleRight:
			startLeftScaleRightNone();
			break;
		case startLeft_SwitchLeft:
			startLeftSwitchLeftNone();
			break;
		case startLeft_SwitchRight:
			startLeftSwitchRightNone();
			break;
		case startRight_ScaleLeft:
			startRightScaleLeftNone();
			break;
		case startRight_ScaleRight:
			startRightScaleRightNone();
			break;
		case startRight_SwitchLeft:
			startRightSwitchLeftNone();
			break;
		case startRight_SwitchRight:
			startRightSwitchRightNone();
			break;
		default:
			break;
		}
	}

	private AutonPlays determinePlay() {
		startingPosition startPosition = startingPosition.Center;
		target primaryTarget = target.Switch;
		// target secondTarget = target.None;
		try {
			startPosition = (startingPosition) startingPositionRadio.getSelected();
			primaryTarget = (target) primaryTargetRadio.getSelected();
			// secondTarget = (target) secondTargetRadio.getSelected();
		} catch (Exception exc) {
		}
		char switchPosition = ' ';
		char scalePosition = ' ';
		try {
			switchPosition = gameData.charAt(0);
		} catch (Exception e) {
		}
		try {
			scalePosition = gameData.charAt(1);
		} catch (Exception e) {
		}
		// Change to Uppercase just in case
		if (switchPosition == 'l') {
			switchPosition = 'L';
		}
		if (switchPosition == 'r') {
			switchPosition = 'R';
		}
		if (scalePosition == 'l') {
			scalePosition = 'L';
		}
		if (scalePosition == 'r') {
			scalePosition = 'R';
		}
		// Setup a default play just in case logic fails
		AutonPlays curPlay = AutonPlays.driveForwardOnly;
		if (gameData.length() > 0) {
			if (startPosition == startingPosition.Left) {
				if (primaryTarget == target.OnSideAny) {
					if (switchPosition == 'L') {
						curPlay = AutonPlays.startLeft_SwitchLeft;
					}
					if (scalePosition == 'L') {
						curPlay = AutonPlays.startLeft_ScaleLeft;
					}
					if (switchPosition == 'R') {
						curPlay = AutonPlays.startLeft_SwitchRight;
					}
				}
				if (primaryTarget == target.Scale) {
					if (scalePosition == 'L') {
						curPlay = AutonPlays.startLeft_ScaleLeft;
					}
					if (scalePosition == 'R') {
						curPlay = AutonPlays.startLeft_ScaleRight;
					}
				}
				if (primaryTarget == target.Switch) {
					if (switchPosition == 'L') {
						curPlay = AutonPlays.startLeft_SwitchLeft;
					}
					if (switchPosition == 'R') {
						curPlay = AutonPlays.startLeft_SwitchRight;
					}
				}
			}
			if (startPosition == startingPosition.Center) {
				if (switchPosition == 'L') {
					curPlay = AutonPlays.startCenter_SwitchLeft;
				}
				if (switchPosition == 'R') {
					curPlay = AutonPlays.startCenter_SwitchRight;
				}
			}
			if (startPosition == startingPosition.Right) {
				if (primaryTarget == target.OnSideAny) {
					if (switchPosition == 'L') {
						curPlay = AutonPlays.startRight_SwitchLeft;
					}
					if (scalePosition == 'L') {
						curPlay = AutonPlays.startRight_ScaleLeft;
					}
					if (switchPosition == 'R') {
						curPlay = AutonPlays.startRight_SwitchRight;
					}
				}
				if (primaryTarget == target.Scale) {
					if (scalePosition == 'L') {
						curPlay = AutonPlays.startRight_ScaleLeft;
					}
					if (scalePosition == 'R') {
						curPlay = AutonPlays.startRight_ScaleRight;
					}
				}
				if (primaryTarget == target.Switch) {
					if (switchPosition == 'L') {
						curPlay = AutonPlays.startRight_SwitchLeft;
					}
					if (switchPosition == 'R') {
						curPlay = AutonPlays.startRight_SwitchRight;
					}
				}
			}
		}
		return curPlay;
	}

	// Compendium pages (plays) have now been subjected to the power of CODE
	// CLEANUP!
	// Notice how much we reduced our plays... so clean!
	
	private void startLeftScaleRightNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 5, 80, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 5, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 145, 80, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.FORWARD, 60, 60, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.RIGHT, 95, 60, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.FORWARD, 185, 75, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			// robotAction(Direction.FORWARD, 5, 60, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			// robotAction(Direction.REVERSE, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.LEFT, 95, 60, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			// robotAction(Direction.FORWARD, 20, 75, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.CUBEACTION, 0, 0, 85, 0, 1);
			autonStep++;
			break;
		case 12:
			robotAction(Direction.FORWARD, 20, 55, 85, 0, 0);
			autonStep++;
			break;
		case 13:
			robotAction(Direction.CUBEACTION, 0, 0, 85, 100, 1);
			autonStep++;
			break;
		case 14:
			robotAction(Direction.REVERSE, 10, 55, 85, 0, 0);
			autonStep++;
			break;
		case 15:
			robotAction(Direction.CUBEACTION, 0, 0, 0, 0, 2);
			autonStep++;
			break;
		}
	}

	private void startLeftSwitchRightNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 5, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 140, 80, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.FORWARD, 60, 60, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.FORWARD, 175, 75, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.RIGHT, 105, 60, 0, 0, 0);
			autonStep++; // no more eleven skipping :(
			break;
		case 8:
			robotAction(Direction.FORWARD, 5, 55, 48, 0, 1.5);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startRightScaleLeftNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 5, 80, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 5, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 145, 80, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.FORWARD, 60, 60, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.LEFT, 95, 60, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.FORWARD, 185, 75, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			// robotAction(Direction.FORWARD, 5, 60, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			// robotAction(Direction.REVERSE, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			// robotAction(Direction.FORWARD, 20, 75, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.CUBEACTION, 0, 0, 85, 0, 1);
			autonStep++;
			break;
		case 12:
			robotAction(Direction.FORWARD, 20, 55, 85, 0, 0);
			autonStep++;
			break;
		case 13:
			robotAction(Direction.CUBEACTION, 0, 0, 85, 100, 1);
			autonStep++;
			break;
		case 14:
			robotAction(Direction.REVERSE, 10, 55, 85, 0, 0);
			autonStep++;
			break;
		case 15:
			robotAction(Direction.CUBEACTION, 0, 0, 0, 0, 2);
			autonStep++;
			break;
		}
	}

	private void startRightSwitchLeftNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 5, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 140, 80, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.FORWARD, 60, 60, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.FORWARD, 175, 75, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.LEFT, 105, 60, 0, 0, 0);
			autonStep++; // elevenSkipping = null;
			break;
		case 8:
			robotAction(Direction.FORWARD, 5, 55, 48, 0, 1.5);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}
	}

	private void driveForwardOnly() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 140, 50, 0, 0, 0);
			autonStep++;
			break;
		}
	}

	private void startCenterSwitchLeftNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 12, 75, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 50, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.RIGHT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 16, 60, 48, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.REVERSE, 48, 60, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.RIGHT, 45, 60, 0, 0, 0);
			autonStep++;
			break;
		}
	}

	private void startCenterSwitchRightNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 12, 70, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 50, 70, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.LEFT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 16, 60, 48, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.REVERSE, 48, 60, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.LEFT, 45, 60, 0, 0, 0);
			autonStep++;
			break;
		}
	}

	private void startLeftScaleLeftNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 198, 80, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 30, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 10, 60, 84, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 0, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 15, 50, 84, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 100, 1);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.REVERSE, 15, 50, 84, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.CUBEACTION, 0, 0, 0, 0, 1);
			autonStep++;
			break;
		}
	}

	private void startLeftSwitchLeftNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 132, 75, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 0, 1);
			autonStep++;
			break;
		case 4:
			// robotAction(Direction.FORWARD, 8, 50, 48, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		case 6:
			// robotAction(Direction.REVERSE, 12, 75, 48, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.CUBEACTION, 0, 0, 0, 0, 2);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.FORWARD, 60, 60, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.RIGHT, 135, 60, 0, 0, 0);
			autonStep++;
			break;
		}
	}

	private void startRightScaleRightNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 198, 80, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 30, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 10, 60, 84, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 0, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 15, 50, 84, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 100, 1);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.REVERSE, 15, 50, 84, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.CUBEACTION, 0, 0, 0, 0, 1);
			autonStep++;
			break;
		}
	}

	private void startRightSwitchRightNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 132, 75, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 0, 1);
			autonStep++;
			break;
		case 4:
			// robotAction(Direction.FORWARD, 8, 50, 48, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		case 6:
			// robotAction(Direction.REVERSE, 12, 75, 48, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.CUBEACTION, 0, 0, 0, 0, 2);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.FORWARD, 60, 60, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.LEFT, 135, 60, 0, 0, 0);
			autonStep++;
			break;
		}
	}

}