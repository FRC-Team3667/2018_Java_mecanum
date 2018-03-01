package org.usfirst.frc.team3667.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.SampleRobot;
//import edu.wpi.first.wpilibj.Sendable;
//import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.ctre.CANTalon.TalonControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;

import org.usfirst.frc.team3667.robot.Robot.cubePickup;
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
	SendableChooser<target> firstTargetRadio;
	SendableChooser<cubePickup> cubePickupRadio;
	SendableChooser<target> secondTargetRadio;

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
		startLeft_ScaleLeft_ScaleLeft, startLeft_ScaleLeft_SwitchLeft, startLeft_ScaleLeft_SwitchRight, startLeft_ScaleLeft_None,
		// Start Left Scale Right
		startLeft_ScaleRight_ScaleRight, startLeft_ScaleRight_SwitchLeft, startLeft_ScaleRight_SwitchRight, startLeft_ScaleRight_None,
		// Start Left Switch Left
		startLeft_SwitchLeft_SwitchLeft, startLeft_SwitchLeft_ScaleRight, startLeft_SwitchLeft_ScaleLeft, startLeft_SwitchLeft_None,
		// Start Left Switch Right
		startLeft_SwitchRight_ScaleLeft, startLeft_SwitchRight_ScaleRight, startLeft_SwitchRight_SwitchRight, startLeft_SwitchRight_None,
		// Start Center Scale Left
		startCenter_ScaleLeft_ScaleLeft, startCenter_ScaleLeft_SwitchLeft, startCenter_ScaleLeft_SwitchRight, startCenter_ScaleLeft_None,
		// Start Center Scale Right
		startCenter_ScaleRight_ScaleRight, startCenter_ScaleRight_SwitchLeft, startCenter_ScaleRight_SwitchRight, startCenter_ScaleRight_None,
		// Start Center Switch Left
		startCenter_SwitchLeft_SwitchLeft, startCenter_SwitchLeft_ScaleRight, startCenter_SwitchLeft_ScaleLeft, startCenter_SwitchLeft_None,
		// Start Center Switch Right
		startCenter_SwitchRight_ScaleLeft, startCenter_SwitchRight_ScaleRight, startCenter_SwitchRight_SwitchRight, startCenter_SwitchRight_None,
		// Start Right Scale Left
		startRight_ScaleLeft_ScaleLeft, startRight_ScaleLeft_SwitchLeft, startRight_ScaleLeft_SwitchRight, startRight_ScaleLeft_None,
		// Start Right Scale Right
		startRight_ScaleRight_ScaleRight, startRight_ScaleRight_SwitchLeft, startRight_ScaleRight_SwitchRight, startRight_ScaleRight_None,
		// Start Right Switch Left
		startRight_SwitchLeft_SwitchLeft, startRight_SwitchLeft_ScaleRight, startRight_SwitchLeft_ScaleLeft, startRight_SwitchLeft_None,
		// Start Right Switch Right
		startRight_SwitchRight_ScaleLeft, startRight_SwitchRight_ScaleRight, startRight_SwitchRight_SwitchRight, startRight_SwitchRight_None,
	};

	public enum Action {
		UP, DOWN, OUTTAKE, INTAKE
	};

	public enum startingPosition {
		Left, Center, Right
	}

	public enum target {
		Switch, Scale, None
	}

	public enum cubePickup {
		One, Two, Three, Four, Five, Six
	}

	public static final double kDistancePerPulse = kDistancePerRevolution / kPulsesPerRevolution;
	// Encoder Values for 18.
	private Encoder leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
	private Encoder rightEncoder = new Encoder(2, 3, true, EncodingType.k4X);
	private Encoder liftEncoder = new Encoder(4, 5, false, EncodingType.k4X);
	private DigitalInput limitSwitchHigh = new DigitalInput(6);
	private DigitalInput limitSwitchLow = new DigitalInput(7);

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
	SpeedControllerGroup leftMotors = new SpeedControllerGroup(_frontLeftMotor, _rearLeftMotor);
	SpeedControllerGroup rightMotors = new SpeedControllerGroup(_frontRightMotor, _rearRightMotor);
	DifferentialDrive _drive = new DifferentialDrive(leftMotors, rightMotors);
	Joystick _joy = new Joystick(0);
	Joystick _cubeJoy = new Joystick(1);

	double desiredCubeHeight = 0;

	int autonStep = 1;
	AutonPlays curPlay = AutonPlays.driveForwardOnly;

	double lastValidDirection = 0;

	// Pneumatics variables
	Compressor compressor = new Compressor(6);
	DoubleSolenoid solenoid = new DoubleSolenoid(1, 2); // replace with actual
														// device ids once
														// configured in
														// Silverlight please -M

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		imu = new ADIS16448_IMU();
		imu.calibrate();
		imu.reset();

		// _drive.setInvertedMotor(MotorType.kFrontRight, true);
		// _drive.setInvertedMotor(MotorType.kRearRight, true);
		// autonomousCommand = new move() ;

		// start of encoders
		leftEncoder.setDistancePerPulse(kDistancePerPulse);
		rightEncoder.setDistancePerPulse(kDistancePerPulse);
		liftEncoder.setDistancePerPulse(kDistancePerPulse);
		leftEncoder.reset();
		rightEncoder.reset();
		liftEncoder.reset();
		// Setup the menu for position selection.
		startingPositionRadio = new SendableChooser<startingPosition>();
		startingPositionRadio.addObject("Left", startingPosition.Left);
		startingPositionRadio.addDefault("Center", startingPosition.Center);
		startingPositionRadio.addObject("Right", startingPosition.Right);
		SmartDashboard.putData("Starting Position", startingPositionRadio);
		// First Target Selector.
		firstTargetRadio = new SendableChooser<target>();
		firstTargetRadio.addDefault("Scale", target.Scale);
		firstTargetRadio.addObject("Switch", target.Switch);
		SmartDashboard.putData("First Targer", firstTargetRadio);
		// Cube Pickup Selector.
		// cubePickupRadio = new SendableChooser();
		// cubePickupRadio.addDefault("One", cubePickup.One);
		// cubePickupRadio.addObject("Two", cubePickup.Two);
		// cubePickupRadio.addObject("Three", cubePickup.Three);
		// cubePickupRadio.addObject("Four", cubePickup.Four);
		// cubePickupRadio.addObject("Five", cubePickup.Five);
		// cubePickupRadio.addObject("Six", cubePickup.Six);
		// SmartDashboard.putData("CUBE PICKUP", cubePickupRadio);
		// Second Target Selector.
		secondTargetRadio = new SendableChooser<target>();
		secondTargetRadio.addDefault("Scale", target.Scale);
		secondTargetRadio.addObject("Switch", target.Switch);
		secondTargetRadio.addObject("None", target.None);
		SmartDashboard.putData("Second Target", secondTargetRadio);

		compressor.setClosedLoopControl(true);

		// solenoid code, move these bits to their own methods before deploying!
		solenoid.set(DoubleSolenoid.Value.kOff); // turns it off
		solenoid.set(DoubleSolenoid.Value.kForward); // activates its "forward"
														// channel
		solenoid.set(DoubleSolenoid.Value.kReverse); // activates its "reverse"
														// channel
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		// Update the Smart Dashboard Data
		updateSmartDashboardData();
		if (_joy.getRawAxis(2) != 0) {
			_pickupLeft.set(_joy.getRawAxis(2));
			_pickupRight.set(_joy.getRawAxis(2));
		} else {
			_pickupLeft.set(_joy.getRawAxis(3) * -1.0);
			_pickupRight.set(_joy.getRawAxis(3) * -1.0);
		}
		
		if (_cubeJoy.getRawAxis(2) != 0) {
			_lift.set(_cubeJoy.getRawAxis(2));
			_lift2.set(_cubeJoy.getRawAxis(2));
		} else {
			_lift.set(_cubeJoy.getRawAxis(3) * -1.0);
			_lift2.set(_cubeJoy.getRawAxis(3) * -1.0);
		}

		if (_joy.getRawButton(4)) {
			testAutonomousPeriodic(); // When the yellow "Y" button is pressed
		} else {
			_drive.arcadeDrive(_joy.getRawAxis(1) * -1, _joy.getRawAxis(4));
		}
		if (_joy.getRawButton(3)) {
			initAndResetAll(); // When the blue "X" button is pressed
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
		while (gameData.length() != 3 && retries > 0) {
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

	private void executeAutonomousCommandCompendium() {
		if (curPlay == AutonPlays.driveForwardOnly) {
			curPlay = determinePlay();
			// This should be done during init, but check again in case we are
			// just driving forward
		}
		// This is the 3667 play book for Autonomous options
		switch (curPlay) {
		case driveForwardOnly:
			driveForwardOnly();
			break;
		case startCenter_ScaleLeft_ScaleLeft:
			startCenterScaleLeftScaleLeft();
			break;
		case startCenter_ScaleLeft_SwitchLeft:
			startCenterScaleLeftSwitchLeft();
			break;
		case startCenter_ScaleLeft_SwitchRight:
			startCenterScaleLeftSwitchRight();
			break;
		case startCenter_ScaleLeft_None:
			startCenterScaleLeftNone();
			break;
		case startCenter_ScaleRight_ScaleRight:
			startCenterScaleRightScaleRight();
			break;
		case startCenter_ScaleRight_SwitchLeft:
			startCenterScaleRightSwitchLeft();
			break;
		case startCenter_ScaleRight_SwitchRight:
			startCenterScaleRightSwitchRight();
			break;
		case startCenter_ScaleRight_None:
			startCenterScaleRightNone();
			break;
		case startCenter_SwitchLeft_ScaleLeft:
			startCenterSwitchLeftScaleLeft();
			break;
		case startCenter_SwitchLeft_ScaleRight:
			startCenterSwitchLeftScaleRight();
			break;
		case startCenter_SwitchLeft_SwitchLeft:
			startCenterSwitchLeftSwitchLeft();
			break;
		case startCenter_SwitchLeft_None:
			startCenterSwitchLeftNone();
			break;
		case startCenter_SwitchRight_ScaleLeft:
			startCenterSwitchRightScaleLeft();
			break;
		case startCenter_SwitchRight_ScaleRight:
			startCenterSwitchRightScaleRight();
			break;
		case startCenter_SwitchRight_SwitchRight:
			startCenterSwitchRightSwitchRight();
			break;
		case startCenter_SwitchRight_None:
			startCenterSwitchRightNone();
			break;
		case startLeft_ScaleLeft_ScaleLeft:
			startLeftScaleLeftScaleLeft();
			break;
		case startLeft_ScaleLeft_SwitchLeft:
			startLeftScaleLeftSwitchLeft();
			break;
		case startLeft_ScaleLeft_SwitchRight:
			startLeftScaleLeftSwitchRight();
			break;
		case startLeft_ScaleLeft_None:
			startLeftScaleLeftNone();
			break;
		case startLeft_ScaleRight_ScaleRight:
			startLeftScaleRightScaleRight();
			break;
		case startLeft_ScaleRight_SwitchLeft:
			startLeftScaleRightSwitchLeft();
			break;
		case startLeft_ScaleRight_SwitchRight:
			startLeftScaleRightSwitchRight();
			break;
		case startLeft_ScaleRight_None:
			startLeft_ScaleRight_None();
			break;
		case startLeft_SwitchLeft_ScaleLeft:
			startLeftSwitchLeftScaleLeft();
			break;
		case startLeft_SwitchLeft_ScaleRight:
			startLeftSwitchLeftScaleRight();
			break;
		case startLeft_SwitchLeft_SwitchLeft:
			startLeftSwitchLeftSwitchLeft();
			break;
		case startLeft_SwitchLeft_None:
			startLeftSwitchLeftNone();
			break;
		case startLeft_SwitchRight_ScaleLeft:
			startLeftSwitchRightScaleLeft();
			break;
		case startLeft_SwitchRight_ScaleRight:
			startLeftSwitchRightScaleRight();
			break;
		case startLeft_SwitchRight_SwitchRight:
			startLeftSwitchRightSwitchRight();
			break;
		case startLeft_SwitchRight_None:
			startLeftSwitchRightNone();
			break;
		case startRight_ScaleLeft_ScaleLeft:
			startRightScaleLeftScaleLeft();
			break;
		case startRight_ScaleLeft_SwitchLeft:
			startRightScaleLeftSwitchLeft();
			break;
		case startRight_ScaleLeft_SwitchRight:
			startRightScaleLeftSwitchRight();
			break;
		case startRight_ScaleLeft_None:
			startRightScaleLeftNone();
			break;
		case startRight_ScaleRight_ScaleRight:
			startRightScaleRightScaleRight();
			break;
		case startRight_ScaleRight_SwitchLeft:
			startRightScaleRightSwitchLeft();
			break;
		case startRight_ScaleRight_SwitchRight:
			startRightScaleRightSwitchRight();
			break;
		case startRight_ScaleRight_None:
			startRightScaleRightNone();
			break;
		case startRight_SwitchLeft_ScaleLeft:
			startRightSwitchLeftScaleLeft();
			break;
		case startRight_SwitchLeft_ScaleRight:
			startRightSwitchLeftScaleRight();
			break;
		case startRight_SwitchLeft_SwitchLeft:
			startRightSwitchLeftSwitchLeft();
			break;
		case startRight_SwitchLeft_None:
			startRightSwitchLeftNone();
			break;
		case startRight_SwitchRight_ScaleLeft:
			startRightSwitchRightScaleLeft();
			break;
		case startRight_SwitchRight_ScaleRight:
			startRightSwitchRightScaleRight();
			break;
		case startRight_SwitchRight_SwitchRight:
			startRightSwitchRightSwitchRight();
			break;
		case startRight_SwitchRight_None:
			startRightSwitchRightNone();
			break;
		default:
			break;
		}
	}

	private void driveForwardOnly() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 150, 50, 0, 0, 0);
			autonStep++;
			break;
		}
	}

	private void startCenterScaleLeftScaleLeft() {
		// Not done
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 55, 60, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 105, 60, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 261, 70, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			// Needs to do with cube grabber
			robotAction(Direction.FORWARD, 0, 60, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.REVERSE, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.FORWARD, 100, 60, 0, 0, 0);
			autonStep++;
			break;
		}
	}

	private void startCenterScaleLeftSwitchLeft() {
		switch (autonStep) {
		case 1:

		}
	}

	private void startCenterScaleLeftSwitchRight() {
		// TODO Auto-generated method stub

	}

	private void startCenterScaleLeftNone() {
		// TODO Auto-generated method stub

	}

	private void startCenterScaleRightScaleRight() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 55, 60, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 105, 60, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 261, 70, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			// Needs to do with cube grabber
			robotAction(Direction.FORWARD, 0, 60, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.REVERSE, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.FORWARD, 100, 60, 0, 0, 0);
			autonStep++;
			break;
		case 12:
			// robotAction(Direction., 0, 60, 0, 0, 0);
			break;
		}
	}

	private void startCenterScaleRightSwitchLeft() {
		// TODO Auto-generated method stub

	}

	private void startCenterScaleRightSwitchRight() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 12, 75, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 70, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.LEFT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 27, 75, 48, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.REVERSE, 27, 75, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.LEFT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.FORWARD, 36, 75, 0, -100, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.CUBEACTION, 0, 0, 0, -100, 1);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.REVERSE, 36, 75, 0, 0, 0);
			autonStep++;
			break;
		case 12:
			robotAction(Direction.RIGHT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 13:
			robotAction(Direction.FORWARD, 27, 75, 48, 0, 0);
			autonStep++;
			break;
		case 14:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}

	}

	private void startCenterScaleRightNone() {
		// TODO Auto-generated method stub

	}

	private void startCenterSwitchLeftScaleLeft() {
		// TODO Auto-generated method stub

	}

	private void startCenterSwitchLeftScaleRight() {
		// TODO Auto-generated method stub

	}

	private void startCenterSwitchLeftSwitchLeft() {
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
			robotAction(Direction.FORWARD, 70, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.RIGHT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 27, 75, 48, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.REVERSE, 27, 75, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.RIGHT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.FORWARD, 36, 75, 0, -100, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.CUBEACTION, 0, 0, 0, -100, 1);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.REVERSE, 36, 75, 0, 0, 0);
			autonStep++;
			break;
		case 12:
			robotAction(Direction.LEFT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 13:
			robotAction(Direction.FORWARD, 27, 75, 48, 0, 0);
			autonStep++;
			break;
		case 14:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
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
			robotAction(Direction.FORWARD, 70, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.RIGHT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 27, 75, 48, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startCenterSwitchRightScaleLeft() {
		// TODO Auto-generated method stub

	}

	private void startCenterSwitchRightScaleRight() {
		// TODO Auto-generated method stub

	}

	private void startCenterSwitchRightSwitchRight() {
		// TODO Auto-generated method stub

	}

	private void startCenterSwitchRightNone() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 12, 75, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 70, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.LEFT, 45, 75, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 27, 75, 48, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startCenterNoneScaleLeft() {
		// TODO Auto-generated method stub

	}

	private void startCenterNoneScaleRight() {
		// TODO Auto-generated method stub

	}

	private void startCenterNoneSwitchLeft() {
		// TODO Auto-generated method stub

	}

	private void startCenterNoneSwitchRight() {
		// TODO Auto-generated method stub

	}

	private void startCenterNoneNone() {
		// TODO Auto-generated method stub

	}

	private void startLeftScaleLeftScaleLeft() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 285, 90, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 20, 50, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 100, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.REVERSE, 24, 50, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.RIGHT, 70, 50, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 92, 75, 0, -100, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.CUBEACTION, 0, 0, 0, -100, 0.5);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.LEFT, 160, 60, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.FORWARD, 38, 90, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startLeftScaleLeftSwitchLeft() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 285, 90, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 20, 50, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 100, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.REVERSE, 24, 50, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.RIGHT, 70, 50, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 92, 75, 0, -100, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.CUBEACTION, 0, 0, 0, -100, 0.5);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startLeftScaleLeftSwitchRight() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 285, 90, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 20, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 100, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.REVERSE, 24, 60, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 55, 75, 0, -100, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.FORWARD, 200, 75, 0, -100, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.RIGHT, 135, 60, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.FORWARD, 55, 75, 0, -100, 0);
			autonStep++;
			break;
		case 12:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startLeftScaleLeftNone() {
		// TODO Auto-generated method stub

	}

	private void startLeftScaleRightScaleRight() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 215, 90, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 200, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 79, 75, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.LEFT, 90, 60, 85, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 15, 55, 85, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.CUBEACTION, 0, 0, 85, 100, 1);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.REVERSE, 15, 55, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.LEFT, 75, 60, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.FORWARD, 112, 75, 0, -100, 0);
			autonStep++;
			break;
		case 12:
			robotAction(Direction.CUBEACTION, 0, 0, 0, -100, 1);
			autonStep++;
			break;
		case 13:
			robotAction(Direction.RIGHT, 180, 60, 0, 0, 0);
			autonStep++;
			break;
		case 14:
			robotAction(Direction.FORWARD, 112, 75, 0, 0, 0);
			autonStep++;
			break;
		case 15:
			robotAction(Direction.LEFT, 115, 60, 85, 0, 0);
			autonStep++;
			break;
		case 16:
			robotAction(Direction.FORWARD, 15, 55, 85, 0, 0);
			autonStep++;
			break;
		case 17:
			robotAction(Direction.CUBEACTION, 0, 0, 85, 100, 1);
			autonStep++;
			break;
		case 18:
			robotAction(Direction.REVERSE, 15, 55, 0, 0, 0);
			autonStep++;
			break;
		}
	}

	private void startLeftScaleRightSwitchRight() {
		// TODO Auto-generated method stub

	}

	private void startLeftScaleRightSwitchLeft() {
		// TODO Auto-generated method stub

	}

	private void startLeftScaleRightNone() {
		// TODO Auto-generated method stub

	}

	private void startLeft_ScaleRight_None() {
		// TODO Auto-generated method stub

	}

	private void startLeftSwitchLeftScaleLeft() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 140, 60, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 90, 40, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 45, 100, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.REVERSE, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.LEFT, 90, 40, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 13, 60, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.RIGHT, 90, 40, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.FORWARD, 13, 60, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.CUBEACTION, 0, 0, 0, -100, 1);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.REVERSE, 13, 60, 0, 0, 0);
			autonStep++;
			break;
		case 12:
			robotAction(Direction.LEFT, 90, 40, 0, 0, 0);
			autonStep++;
			break;
		case 13:
			robotAction(Direction.FORWARD, 40, 60, 0, 0, 0);
			autonStep++;
			break;
		case 14:
			robotAction(Direction.RIGHT, 90, 40, 0, 0, 0);
			autonStep++;
			break;
		case 15:
			robotAction(Direction.FORWARD, 10, 60, 0, 0, 0);
			autonStep++;
			break;
		case 16:
			robotAction(Direction.CUBEACTION, 0, 0, 85, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startLeftSwitchLeftScaleRight() {
		// TODO Auto-generated method stub

	}

	private void startLeftSwitchLeftSwitchLeft() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 164, 75, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 12, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 48, -100, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.REVERSE, 12, 75, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 48, 75, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.RIGHT, 135, 60, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.FORWARD, 38, 75, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.RIGHT, 25, 60, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startLeftSwitchLeftNone() {
		// TODO Auto-generated method stub

	}

	private void startLeftSwitchRightScaleLeft() {
		// TODO Auto-generated method stub

	}

	private void startLeftSwitchRightScaleRight() {
		// TODO Auto-generated method stub

	}

	private void startLeftSwitchRightSwitchRight() {
		// TODO Auto-generated method stub

	}

	private void startLeftSwitchRightNone() {
		// TODO Auto-generated method stub

	}

	private void startLeftNoneScaleLeft() {
		// TODO Auto-generated method stub

	}

	private void startLeftNoneScaleRight() {
		// TODO Auto-generated method stub

	}

	private void startLeftNoneSwitchLeft() {
		// TODO Auto-generated method stub

	}

	private void startLeftNoneSwitchRight() {
		// TODO Auto-generated method stub

	}

	private void startLeftNoneNone() {
		// TODO Auto-generated method stub

	}

	private void startRightScaleLeftScaleLeft() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 215, 90, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 200, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.FORWARD, 79, 75, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.RIGHT, 90, 60, 85, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 15, 55, 85, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.CUBEACTION, 0, 0, 85, 100, 1);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.REVERSE, 15, 55, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.RIGHT, 75, 60, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.FORWARD, 112, 75, 0, -100, 0);
			autonStep++;
			break;
		case 12:
			robotAction(Direction.CUBEACTION, 0, 0, 0, -100, 1);
			autonStep++;
			break;
		case 13:
			robotAction(Direction.LEFT, 180, 60, 0, 0, 0);
			autonStep++;
			break;
		case 14:
			robotAction(Direction.FORWARD, 112, 75, 0, 0, 0);
			autonStep++;
			break;
		case 15:
			robotAction(Direction.RIGHT, 115, 60, 85, 0, 0);
			autonStep++;
			break;
		case 16:
			robotAction(Direction.FORWARD, 15, 55, 85, 0, 0);
			autonStep++;
			break;
		case 17:
			robotAction(Direction.CUBEACTION, 0, 0, 85, 100, 1);
			autonStep++;
			break;
		case 18:
			robotAction(Direction.REVERSE, 15, 55, 0, 0, 0);
			autonStep++;
			break;
		}
	}

	private void startRightScaleLeftSwitchLeft() {
		// TODO Auto-generated method stub

	}

	private void startRightScaleLeftSwitchRight() {
	}

	private void startRightScaleLeftNone() {
		// TODO Auto-generated method stub

	}

	private void startRightScaleRightScaleRight() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 285, 90, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 20, 50, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 100, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.REVERSE, 24, 50, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.LEFT, 70, 50, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 92, 75, 0, -100, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.CUBEACTION, 0, 0, 0, -100, 0.5);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.RIGHT, 160, 60, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.FORWARD, 38, 90, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 100, 1);
			autonStep++;
			break;
		}

	}

	private void startRightScaleRightSwitchLeft() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 285, 90, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 20, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 100, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.REVERSE, 24, 60, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 55, 75, 0, -100, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.FORWARD, 200, 75, 0, -100, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.LEFT, 135, 60, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.FORWARD, 55, 75, 0, -100, 0);
			autonStep++;
			break;
		case 12:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startRightScaleRightSwitchRight() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 285, 90, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 20, 50, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 84, 100, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.REVERSE, 24, 50, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.LEFT, 70, 50, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 92, 75, 0, -100, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.CUBEACTION, 0, 0, 0, -100, 0.5);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startRightScaleRightNone() {
		// TODO Auto-generated method stub

	}

	private void startRightSwitchLeftScaleLeft() {
		// TODO Auto-generated method stub

	}

	private void startRightSwitchLeftScaleRight() {
		// TODO Auto-generated method stub

	}

	private void startRightSwitchLeftSwitchLeft() {
		// TODO Auto-generated method stub

	}

	private void startRightSwitchLeftNone() {
		// TODO Auto-generated method stub

	}

	private void startRightSwitchRightScaleLeft() {
		// TODO Auto-generated method stub

	}

	private void startRightSwitchRightScaleRight() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 55, 100, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 90, 100, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 10, 100, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 45, 100, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.REVERSE, 10, 100, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.RIGHT, 90, 100, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 13, 100, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.LEFT, 90, 100, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.FORWARD, 13, 60, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.CUBEACTION, 0, 0, 0, -100, 1);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.REVERSE, 13, 60, 0, 0, 0);
			autonStep++;
			break;
		case 12:
			robotAction(Direction.RIGHT, 90, 100, 0, 0, 0);
			autonStep++;
			break;
		case 13:
			robotAction(Direction.FORWARD, 40, 60, 0, 0, 0);
			autonStep++;
			break;
		case 14:
			robotAction(Direction.LEFT, 90, 100, 0, 0, 0);
			autonStep++;
			break;
		case 15:
			robotAction(Direction.FORWARD, 10, 60, 0, 0, 0);
			autonStep++;
			break;

		}

	}

	private void startRightSwitchRightSwitchRight() {
		switch (autonStep) {
		case 1:
			robotAction(Direction.FORWARD, 164, 75, 0, 0, 0);
			autonStep++;
			break;
		case 2:
			robotAction(Direction.LEFT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 3:
			robotAction(Direction.FORWARD, 12, 75, 0, 0, 0);
			autonStep++;
			break;
		case 4:
			robotAction(Direction.CUBEACTION, 0, 0, 48, -100, 1);
			autonStep++;
			break;
		case 5:
			robotAction(Direction.REVERSE, 12, 75, 0, 0, 0);
			autonStep++;
			break;
		case 6:
			robotAction(Direction.RIGHT, 90, 60, 0, 0, 0);
			autonStep++;
			break;
		case 7:
			robotAction(Direction.FORWARD, 48, 75, 0, 0, 0);
			autonStep++;
			break;
		case 8:
			robotAction(Direction.LEFT, 135, 60, 0, 0, 0);
			autonStep++;
			break;
		case 9:
			robotAction(Direction.FORWARD, 38, 75, 0, 0, 0);
			autonStep++;
			break;
		case 10:
			robotAction(Direction.LEFT, 25, 60, 0, 0, 0);
			autonStep++;
			break;
		case 11:
			robotAction(Direction.CUBEACTION, 0, 0, 48, 100, 1);
			autonStep++;
			break;
		}
	}

	private void startRightSwitchRightNone() {
		// TODO Auto-generated method stub

	}

	private void startRightNoneScaleLeft() {
		// TODO Auto-generated method stub

	}

	private void startRightNoneScaleRight() {
		// TODO Auto-generated method stub

	}

	private void startRightNoneSwitchLeft() {
		// TODO Auto-generated method stub

	}

	private void startRightNoneSwitchRight() {
		// TODO Auto-generated method stub

	}

	private void startRightNoneNone() {
		// TODO Auto-generated method stub

	}

	private void robotAction(Direction driveDirection, double distance, double powerPercent, double height,
			double cubeAction, double minDuration) {
		double startingLeftEncoder = leftEncoder.getDistance();
		double startingRightEncoder = rightEncoder.getDistance();
		double turnDegree = 0;
		double targetDegree = 0;
		double currentCubeHeight = liftEncoder.getDistance();
		desiredCubeHeight += height;
		switch (driveDirection) {
		case CUBEACTION:
			// TODO
			long sDuration = (long) (minDuration * 1000);
			try {
				Thread.sleep(sDuration);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			break;
		case FORWARD:
			while (leftEncoder.getDistance() <= startingLeftEncoder + distance
					&& rightEncoder.getDistance() <= startingRightEncoder + distance && distance > 0) {
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
				// obtain the height encoder value here to determine if the
				// height should change while driving
				currentCubeHeight = liftEncoder.getDistance();
				// if (currentCubeHeight != desiredCubeHeight) {
				// if (currentCubeHeight < desiredCubeHeight) {
				// _lift.set(.5);
				// } else {
				// _lift.set(-.5);
				// }
				// }
				// while (limitSwitchHigh.get()) {
				// 	Timer.delay(13);
				//}
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
					&& rightEncoder.getDistance() >= startingRightEncoder - distance && distance > 0) {
				_drive.arcadeDrive(powerPercent * -.01, correctionRotation);
			}
			break;
		case LEFT:
			if (powerPercent > 85)
				powerPercent = 85;
			targetDegree = lastValidDirection - distance;
			while (targetDegree < imu.getAngleZ()) {
				// check if within 10 degrees and if so slow turn
				if (Math.abs(targetDegree - imu.getAngleZ()) > 10) {
					SmartDashboard.putNumber("speed", powerPercent * -.01);
					_drive.arcadeDrive(0, powerPercent * -.01);
				} else {
					_drive.arcadeDrive(0, -.5);
				}
			}
			lastValidDirection -= distance;
			// If we overshot on the turn, than correct
			while (lastValidDirection > imu.getAngleZ()) {
				_drive.arcadeDrive(0, .4);
			}
			break;
		case RIGHT:
			if (powerPercent > 85)
				powerPercent = 85;
			turnDegree = lastValidDirection + distance;
			while (turnDegree > imu.getAngleZ()) {
				// check if within 10 degrees and if so slow turn
				if (Math.abs(turnDegree - imu.getAngleZ()) > 10) {
					_drive.arcadeDrive(0, powerPercent * .01);
				} else {
					_drive.arcadeDrive(0, .5);
				}
			}
			lastValidDirection += distance;
			// If we overshot on the turn, then correct
			while (lastValidDirection < imu.getAngleZ()) {
				_drive.arcadeDrive(0, -.4);
			}
			break;
		default:
			break;
		}
		SmartDashboard.putNumber("target Degree", targetDegree);
		SmartDashboard.putNumber("speed", powerPercent * .01);

	}

	private double getDegreeZ() {
		double someDegree;
		someDegree = imu.getAngleZ();
		return normalizeDegree(someDegree);
	}

	// very nice method that we definitely *did* create
	private double normalizeDegree(double someDegree) {
		while (someDegree < 0 || someDegree >= 360) {
			if (someDegree >= 360) {
				someDegree -= 360;

			} else if (someDegree < 0) {
				someDegree += 360;
			}
		}
		return someDegree;
	}

	private void adjustedDrive(double xAxis, double yAxis, double rotation, double gyroAngle) {
		double curHeight = 0;
		// Round to the nearest Hundredth to remove game pad inaccuracy
		double adjXAxis = Math.round(xAxis * 10.0) / 10.0;
		double adjYAxis = Math.round(yAxis * 10.0) / 10.0;
		double adjRotation = Math.round(rotation * 10.0) / 10.0;
		double adjGyroAngle = Math.round(gyroAngle * 10.0) / 10.0;
		double minSpeedNum = .1;
		// double maxSpeedNum = 1;
		if (curHeight < 10) {
			adjXAxis = Math.round(xAxis * 10.0) / 10.0;
			adjYAxis = Math.round(yAxis * 10.0) / 10.0;
			adjRotation = Math.round(rotation * 10.0) / 10.0;
			adjGyroAngle = Math.round(gyroAngle * 10.0) / 10.0;
		} else if (curHeight < 20) {
			adjXAxis = xAxis * .75;
			adjYAxis = yAxis * .75;
			adjRotation = rotation * .75;
			adjGyroAngle = gyroAngle * .75;
		} else {
			adjXAxis = xAxis * .5;
			adjYAxis = yAxis * .5;
			adjRotation = rotation * .5;
			adjGyroAngle = gyroAngle * .5;
		}
		if (Math.abs(adjXAxis) < minSpeedNum && adjXAxis != 0) {
			if (adjXAxis > 0) {
				adjXAxis = minSpeedNum;

			} else {
				adjXAxis = minSpeedNum * -1;
			}
		}
		if (Math.abs(adjYAxis) < minSpeedNum && adjYAxis != 0) {
			if (adjYAxis > 0) {
				adjYAxis = minSpeedNum;

			} else {
				adjYAxis = minSpeedNum * -1;
			}
		}
		if (Math.abs(adjRotation) < minSpeedNum && adjRotation != 0) {
			if (adjRotation > 0) {
				adjRotation = minSpeedNum;

			} else {
				adjRotation = minSpeedNum * -1;
			}
		}
		if (Math.abs(adjGyroAngle) < minSpeedNum && adjGyroAngle != 0) {
			if (adjGyroAngle > 0) {
				adjGyroAngle = minSpeedNum;

			} else {
				adjGyroAngle = minSpeedNum * -1;
			}
		}
	}

	private void updateSmartDashboardData() {
		SmartDashboard.putNumber("encoderL", leftEncoder.getDistance());
		SmartDashboard.putNumber("encoderR", rightEncoder.getDistance());
		SmartDashboard.putBoolean("button1", _joy.getRawButton(1));
		SmartDashboard.putBoolean("button2", _joy.getRawButton(2));
		SmartDashboard.putNumber("IMU Angle Z", getDegreeZ());
		SmartDashboard.putNumber("Gyro Angle Z", imu.getAngleZ());
		SmartDashboard.putNumber("Last Valid", lastValidDirection);
		SmartDashboard.putNumber("Auton Step", autonStep);
		SmartDashboard.putNumber("Timer", Timer.getMatchTime());
		SmartDashboard.putNumber("Test Encoder Distance Value", liftEncoder.getDistance());
		SmartDashboard.putNumber("Test Encoder Rate Value", liftEncoder.getRate());
		SmartDashboard.putNumber("Test Encoder Count", liftEncoder.get());
		SmartDashboard.putNumber("Test Encoder Raw", liftEncoder.getRaw());
		SmartDashboard.putString("FMS Data", gameData);
		SmartDashboard.putString("Current Play:", curPlay.toString());
	}

	private AutonPlays determinePlay() {
		startingPosition startPosition = (startingPosition) startingPositionRadio.getSelected();
		target firstTarget = (target) firstTargetRadio.getSelected();
		// cubePickup cubePickup = (cubePickup) cubePickupRadio.getSelected();
		target secondTarget = (target) secondTargetRadio.getSelected();
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
		AutonPlays curPlay = AutonPlays.driveForwardOnly;
		if (gameData.length() > 0) {
			// All Start Left Logic
			if (startPosition == startingPosition.Left) {
				if (firstTarget == target.Scale) {
					if (scalePosition == 'L') {
						switch (secondTarget) {
						case Scale:
							curPlay = AutonPlays.startLeft_ScaleLeft_ScaleLeft;
							break;
						case Switch:
							if (switchPosition == 'L') {
								curPlay = AutonPlays.startLeft_ScaleLeft_SwitchLeft;
							} else {
								curPlay = AutonPlays.startLeft_ScaleLeft_SwitchRight;
							}
							break;
						case None:
							curPlay = AutonPlays.startLeft_ScaleLeft_None;
							break;
						}
					}
					// Start Left Scale Right
					else {
						switch (secondTarget) {
						case Scale:
							curPlay = AutonPlays.startLeft_ScaleRight_ScaleRight;
							break;
						case Switch:
							if (switchPosition == 'L') {
								curPlay = AutonPlays.startLeft_ScaleRight_SwitchLeft;
							} else {
								curPlay = AutonPlays.startLeft_ScaleRight_SwitchRight;
							}
							break;
						case None:
							curPlay = AutonPlays.startLeft_ScaleRight_None;
							break;
						}
					}
				}
				// Start Left Switch Left
				else if (firstTarget == target.Switch) {
					if (switchPosition == 'L') {
						switch (secondTarget) {
						case Scale:
							if (scalePosition == 'L') {
								curPlay = AutonPlays.startLeft_SwitchLeft_ScaleLeft;
							} else {
								curPlay = AutonPlays.startLeft_SwitchLeft_ScaleRight;
							}
							break;
						case Switch:
							curPlay = AutonPlays.startLeft_SwitchLeft_SwitchLeft;
							break;
						case None:
							curPlay = AutonPlays.startLeft_SwitchLeft_None;
							break;
						}
					}
					// Start Left Switch Right
					else {
						switch (secondTarget) {
						case Scale:
							if (scalePosition == 'L') {
								curPlay = AutonPlays.startLeft_SwitchRight_ScaleLeft;
							} else {
								curPlay = AutonPlays.startLeft_SwitchRight_ScaleRight;
							}
							break;
						case Switch:
							curPlay = AutonPlays.startLeft_SwitchRight_SwitchRight;
							break;
						case None:
							curPlay = AutonPlays.startLeft_SwitchRight_None;
							break;
						}
					}
				}
			}
			// All Start Center Logic
			else if (startPosition == startingPosition.Center) {
				if (firstTarget == target.Scale) {
					if (scalePosition == 'L') {
						switch (secondTarget) {
						case Scale:
							curPlay = AutonPlays.startCenter_ScaleLeft_ScaleLeft;
							break;
						case Switch:
							if (switchPosition == 'L') {
								curPlay = AutonPlays.startCenter_ScaleLeft_SwitchLeft;
							} else {
								curPlay = AutonPlays.startCenter_ScaleLeft_SwitchRight;
							}
							break;
						case None:
							curPlay = AutonPlays.startCenter_ScaleLeft_None;
							break;
						}
					}
					// Start Center Scale Right
					else {
						switch (secondTarget) {
						case Scale:
							curPlay = AutonPlays.startCenter_ScaleRight_ScaleRight;
							break;
						case Switch:
							if (switchPosition == 'L') {
								curPlay = AutonPlays.startCenter_ScaleRight_SwitchLeft;
							} else {
								curPlay = AutonPlays.startCenter_ScaleRight_SwitchRight;
							}
							break;
						case None:
							curPlay = AutonPlays.startCenter_ScaleRight_None;
							break;
						}
					}
				}
				// Start Center Switch Left
				else if (firstTarget == target.Switch) {
					if (switchPosition == 'L') {
						switch (secondTarget) {
						case Scale:
							if (scalePosition == 'L') {
								curPlay = AutonPlays.startCenter_SwitchLeft_ScaleLeft;
							} else {
								curPlay = AutonPlays.startCenter_SwitchLeft_ScaleRight;
							}
							break;
						case Switch:
							curPlay = AutonPlays.startCenter_SwitchLeft_SwitchLeft;
							break;
						case None:
							curPlay = AutonPlays.startCenter_SwitchLeft_None;
							break;
						}
					}
					// Start Center Switch Right
					else {
						switch (secondTarget) {
						case Scale:
							if (scalePosition == 'L') {
								curPlay = AutonPlays.startCenter_SwitchRight_ScaleLeft;
							} else {
								curPlay = AutonPlays.startCenter_SwitchRight_ScaleRight;
							}
							break;
						case Switch:
							curPlay = AutonPlays.startCenter_SwitchRight_SwitchRight;
							break;
						case None:
							curPlay = AutonPlays.startCenter_SwitchRight_None;
							break;
						}
					}
				}
			}
			// All Start Right Logic
			else if (startPosition == startingPosition.Right) {
				if (firstTarget == target.Scale) {
					if (scalePosition == 'L') {
						switch (secondTarget) {
						case Scale:
							curPlay = AutonPlays.startRight_ScaleLeft_ScaleLeft;
							break;
						case Switch:
							if (switchPosition == 'L') {
								curPlay = AutonPlays.startRight_ScaleLeft_SwitchLeft;
							} else {
								curPlay = AutonPlays.startRight_ScaleLeft_SwitchRight;
							}
							break;
						case None:
							curPlay = AutonPlays.startRight_ScaleLeft_None;
							break;
						}
					}
					// Start Right Scale Right
					else {
						switch (secondTarget) {
						case Scale:
							curPlay = AutonPlays.startRight_ScaleRight_ScaleRight;
							break;
						case Switch:
							if (switchPosition == 'L') {
								curPlay = AutonPlays.startRight_ScaleRight_SwitchLeft;
							} else {
								curPlay = AutonPlays.startRight_ScaleRight_SwitchRight;
							}
							break;
						case None:
							curPlay = AutonPlays.startRight_ScaleRight_None;
							break;
						}
					}
				}
				// Start Right Switch Left
				else if (firstTarget == target.Switch) {
					if (switchPosition == 'L') {
						switch (secondTarget) {
						case Scale:
							if (scalePosition == 'L') {
								curPlay = AutonPlays.startRight_SwitchLeft_ScaleLeft;
							} else {
								curPlay = AutonPlays.startRight_SwitchLeft_ScaleRight;
							}
							break;
						case Switch:
							curPlay = AutonPlays.startRight_SwitchLeft_SwitchLeft;
							break;
						case None:
							curPlay = AutonPlays.startRight_SwitchLeft_None;
							break;
						}
					}
					// Start Right Switch Right
					else {
						switch (secondTarget) {
						case Scale:
							if (scalePosition == 'L') {
								curPlay = AutonPlays.startRight_SwitchRight_ScaleLeft;
							} else {
								curPlay = AutonPlays.startRight_SwitchRight_ScaleRight;
							}
							break;
						case Switch:
							curPlay = AutonPlays.startRight_SwitchRight_SwitchRight;
							break;
						case None:
							curPlay = AutonPlays.startRight_SwitchRight_None;
							break;
						}
					}
				}
			}
		}
		return curPlay;
	}
}