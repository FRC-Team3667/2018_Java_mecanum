package org.usfirst.frc.team3667.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
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
	SendableChooser startingPositionRadio;
	SendableChooser firstTargetRadio;
	SendableChooser cubePickupRadio;
	SendableChooser secondTargetRadio;

	String gameData;

	// Command autonomousCommand;
	public static final double kDistancePerRevolution = 18.84; // guestimate
	Command autonomousCommand;
	// SendableChooser autoChooser;
	// for an AS5145B Magnetic Encoder
	public static final double kPulsesPerRevolution = 1024;

	public enum Direction {
		FORWARD, REVERSE, LEFT, RIGHT
	};

	public enum AutonPlays {
		// Start Left Scale Left
		startLeft_ScaleLeft_ScaleLeft, startLeft_ScaleLeft_SwitchLeft, startLeft_ScaleLeft_SwitchRight,
		// Start Left Scale Right
		startLeft_ScaleRight_ScaleRight, startLeft_ScaleRight_SwitchLeft, startLeft_ScaleRight_SwitchRight,
		// Start Left Switch Left
		startLeft_SwitchLeft_SwitchLeft, startLeft_SwitchLeft_ScaleRight, startLeft_SwitchLeft_ScaleLeft,
		// Start Left Switch Right
		startLeft_SwitchRight_ScaleLeft, startLeft_SwitchRight_ScaleRight, startLeft_SwitchRight_SwitchRight,
		// Start Center Scale Left
		startCenter_ScaleLeft_ScaleLeft, startCenter_ScaleLeft_SwitchLeft, startCenter_ScaleLeft_SwitchRight,
		// Start Center Scale Right
		startCenter_ScaleRight_ScaleRight, startCenter_ScaleRight_SwitchLeft, startCenter_ScaleRight_SwitchRight,
		// Start Center Switch Left
		startCenter_SwitchLeft_SwitchLeft, startCenter_SwitchLeft_ScaleRight, startCenter_SwitchLeft_ScaleLeft,
		// Start Center Switch Right
		startCenter_SwitchRight_ScaleLeft, startCenter_SwitchRight_ScaleRight, startCenter_SwitchRight_SwitchRight,
		// Start Right Scale Left
		startRight_ScaleLeft_ScaleLeft, startRight_ScaleLeft_SwitchLeft, startRight_ScaleLeft_SwitchRight,
		// Start Right Scale Right
		startRight_ScaleRight_ScaleRight, startRight_ScaleRight_SwitchLeft, startRight_ScaleRight_SwitchRight,
		// Start Right Switch Left
		startRight_SwitchLeft_SwitchLeft, startRight_SwitchLeft_ScaleRight, startRight_SwitchLeft_ScaleLeft,
		// Start Right Switch Right
		startRight_SwitchRight_ScaleLeft, startRight_SwitchRight_ScaleRight, startRight_SwitchRight_SwitchRight,
	};

	public enum Action {
		UP, DOWN, OUTTAKE, INTAKE
	};

	public static final double kDistancePerPulse = kDistancePerRevolution / kPulsesPerRevolution;
	private Encoder leftEncoder = new Encoder(0, 1, true, EncodingType.k4X);
	private Encoder rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
	private Encoder speedTestEncoder = new Encoder(5, 4, false, EncodingType.k4X);
	// Search US Digital Encoder FRC Java E4P Encoder

	Timer autoTime = new Timer();
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(13);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(12);
	WPI_TalonSRX _rearRightMotor = new WPI_TalonSRX(11);
	WPI_TalonSRX _rearLeftMotor = new WPI_TalonSRX(10);
	WPI_TalonSRX _climber = new WPI_TalonSRX(14);
	SpeedControllerGroup leftMotors = new SpeedControllerGroup(_frontLeftMotor, _rearLeftMotor);
	SpeedControllerGroup rightMotors = new SpeedControllerGroup(_frontRightMotor, _rearRightMotor);
	DifferentialDrive _drive = new DifferentialDrive(leftMotors, rightMotors);
	Joystick _joy = new Joystick(0);
	Joystick _climberJoy = new Joystick(1);

	double desiredCubeHeight = 0;

	int autonStep = 1;

	double lastValidDirection = 0;

	public enum startingPosition {
		Left, Center, Right
	}

	public enum target {
		Switch, Scale, Vault
	}

	public enum cubePickup {
		One, Two, Three, Four, Five, Six
	}

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
		speedTestEncoder.setDistancePerPulse(kDistancePerPulse);
		leftEncoder.reset();
		rightEncoder.reset();
		speedTestEncoder.reset();
		// Setup the menu for position selection.
		startingPositionRadio = new SendableChooser();
		startingPositionRadio.addObject("LEFT", startingPosition.Left);
		startingPositionRadio.addDefault("CENTER", startingPosition.Center);
		startingPositionRadio.addObject("RIGHT", startingPosition.Right);
		SmartDashboard.putData("STARTING POSITION", startingPositionRadio);
		// First Target Selector.
		firstTargetRadio = new SendableChooser();
		firstTargetRadio.addDefault("SCALE", target.Scale);
		firstTargetRadio.addObject("SWITCH", target.Switch);
		firstTargetRadio.addObject("VAULT", target.Vault);
		SmartDashboard.putData("FIRST TARGET", firstTargetRadio);
		// Cube Pickup Selector.
		cubePickupRadio = new SendableChooser();
		cubePickupRadio.addDefault("One", cubePickup.One);
		cubePickupRadio.addObject("Two", cubePickup.Two);
		cubePickupRadio.addObject("Three", cubePickup.Three);
		cubePickupRadio.addObject("Four", cubePickup.Four);
		cubePickupRadio.addObject("Five", cubePickup.Five);
		cubePickupRadio.addObject("Six", cubePickup.Six);
		SmartDashboard.putData("CUBE PICKUP", cubePickupRadio);
		// Second Target Selector.
		secondTargetRadio = new SendableChooser();
		secondTargetRadio.addDefault("SCALE", target.Scale);
		secondTargetRadio.addObject("SWITCH", target.Switch);
		secondTargetRadio.addObject("VAULT", target.Vault);
		SmartDashboard.putData("SECOND TARGET", firstTargetRadio);
		// Get FMS Data to determine ownership sides.
		gameData = DriverStation.getInstance().getGameSpecificMessage();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		// Update the Smart Dashboard Data
		updateSmartDashboardData();
		if (_joy.getRawAxis(2) != 0) {
			_climber.set(_joy.getRawAxis(2));
		} else {
			_climber.set(_joy.getRawAxis(3) * -1.0);
		}

		if (_joy.getRawButton(4)) {
			testAutonomousPeriodic(); // When the yellow "Y" button is pressed
		} else {
			_drive.arcadeDrive(_joy.getRawAxis(1) * -1, _joy.getRawAxis(4));
		}
		if (_joy.getRawButton(3)) {
			initAndResetAll(); // When the blue "X" button is pressed
		}
		Object startPos = startingPositionRadio.getSelected();

	}

	// No code to be added in autonomousInit
	public void autonomousInit() {
		initAndResetAll();
	}

	private void initAndResetAll() {
		// if (autoChooser.getSelected() != null) {
		// autonomousCommand = (Command) autoChooser.getSelected();
		// autonomousCommand.start();
		// }
		imu.reset();
		leftEncoder.reset();
		rightEncoder.reset();
		speedTestEncoder.reset();
		autonStep = 1;
		lastValidDirection = 0;
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
		startingPosition startPosition = (startingPosition) startingPositionRadio.getSelected();
		target firstTarget = (target) firstTargetRadio.getSelected();
		cubePickup cubePickup = (cubePickup) cubePickupRadio.getSelected();
		target secondTarget = (target) secondTargetRadio.getSelected();
		char switchPosition = gameData.charAt(0);
		char scalePosition = gameData.charAt(1);
		AutonPlays curPlay = AutonPlays.startCenter_ScaleLeft_ScaleLeft;
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
					case Vault:
						curPlay = AutonPlays.startLeft_ScaleLeft_Vault;
						break;
					}
				} else {

				}
			} else if (firstTarget == target.Switch) {

			} else if (firstTarget == target.Vault) {

			}
		} else if (startPosition == startingPosition.Center) {

		} else if (startPosition == startingPosition.Right) {

		}
		// curPlay = AutonPlays.startLeft_ScaleLeft_ScaleLeft;
		// This is the 3667 play book for Autonomous options
		switch (curPlay) {
		case startCenter_ScaleLeft_ScaleLeft:
			break;
		case startCenter_ScaleRight_ScaleRight:
			break;
		case startCenter_ScaleRight_SwitchLeft:
			break;
		case startCenter_ScaleRight_SwitchRight:
			break;
		case startCenter_SwitchLeft_SwitchLeft:
			break;
		case startCenter_SwitchRight_ScaleLeft:
			break;
		case startCenter_SwitchRight_ScaleRight:
			break;
		case startCenter_SwitchRight_SwitchRight:
			break;
		case startLeft_ScaleLeft_ScaleLeft:
			driveRobot(Direction.FORWARD, 55, 60);
			break;
		case startLeft_ScaleRight_ScaleRight:
			break;
		case startLeft_ScaleRight_SwitchLeft:
			break;
		case startLeft_ScaleRight_SwitchRight:
			break;
		case startLeft_SwitchLeft_SwitchLeft:
			break;
		case startLeft_SwitchRight_ScaleLeft:
			break;
		case startLeft_SwitchRight_ScaleRight:
			break;
		case startLeft_SwitchRight_SwitchRight:
			break;
		case startRight_ScaleLeft_ScaleLeft:
			break;
		case startRight_ScaleRight_ScaleRight:
			break;
		case startRight_ScaleRight_SwitchLeft:
			break;
		case startRight_ScaleRight_SwitchRight:
			break;
		case startRight_SwitchLeft_SwitchLeft:
			break;
		case startRight_SwitchRight_ScaleLeft:
			break;
		case startRight_SwitchRight_ScaleRight:
			break;
		case startRight_SwitchRight_SwitchRight:
			break;
		default:
			break;

		}
	}

	private void switchSwitchR() {
		switch (autonStep) {
		case 1:
			driveRobot(Direction.FORWARD, 55, 60);
			autonStep++;
			break;
		case 2:
			turnRobot(Direction.LEFT, 90, 60);
			autonStep++;
			break;
		case 3:
			driveRobot(Direction.FORWARD, 10, 60);
			autonStep++;
			break;
		case 4:
			waitRobot(2);
			autonStep++;
			break;
		case 5:
			driveRobot(Direction.REVERSE, 10, 60);
			autonStep++;
			break;
		case 6:
			turnRobot(Direction.RIGHT, 90, 60);
			autonStep++;
			break;
		case 7:
			driveRobot(Direction.FORWARD, 13, 60);
			autonStep++;
			break;
		case 8:
			turnRobot(Direction.LEFT, 90, 60);
			autonStep++;
			break;
		case 9:
			driveRobot(Direction.FORWARD, 13, 60);
			autonStep++;
			break;
		case 10:
			turnRobot(Direction.RIGHT, 90, 60);
			autonStep++;
			break;
		case 11:
			driveRobot(Direction.FORWARD, 10, 60);
			autonStep++;
			break;
		case 12:
			driveRobot(Direction.REVERSE, 10, 60);
			autonStep++;
			break;
		case 13:
			turnRobot(Direction.RIGHT, 90, 60);
			autonStep++;
			break;
		case 14:
			driveRobot(Direction.FORWARD, 13, 60);
			autonStep++;
			break;
		case 15:
			turnRobot(Direction.LEFT, 90, 60);
			autonStep++;
			break;
		case 16:
			driveRobot(Direction.FORWARD, 10, 60);
			autonStep++;
			break;
		case 17:
			waitRobot(2);
			autonStep++;
			break;
		}
	}

	private void switchSwitchL() {
		switch (autonStep) {
		case 1:
			desiredCubeHeight = 0;
			driveRobot(Direction.FORWARD, 55, 60);
			autonStep++;
			break;
		case 2:
			desiredCubeHeight = 10;
			turnRobot(Direction.RIGHT, 90, 60);
			autonStep++;
			break;
		case 3:
			desiredCubeHeight = 20;
			driveRobot(Direction.FORWARD, 10, 60);
			autonStep++;
			break;
		case 4:
			waitRobot(2);
			autonStep++;
			break;
		case 5:
			driveRobot(Direction.REVERSE, 10, 60);
			autonStep++;
			break;
		case 6:
			turnRobot(Direction.LEFT, 90, 60);
			autonStep++;
			break;
		case 7:
			driveRobot(Direction.FORWARD, 13, 60);
			autonStep++;
			break;
		case 8:
			turnRobot(Direction.LEFT, 90, 60);
			autonStep++;
			break;
		case 9:
			driveRobot(Direction.FORWARD, 13, 60);
			autonStep++;
			break;
		case 10:
			turnRobot(Direction.LEFT, 90, 60);
			autonStep++;
			break;
		case 11:
			driveRobot(Direction.FORWARD, 10, 60);
			autonStep++;
			break;
		case 12:
			driveRobot(Direction.REVERSE, 10, 60);
			autonStep++;
			break;
		case 13:
			turnRobot(Direction.LEFT, 90, 60);
			autonStep++;
			break;
		case 14:
			driveRobot(Direction.FORWARD, 13, 60);
			autonStep++;
			break;
		case 15:
			turnRobot(Direction.RIGHT, 90, 60);
			autonStep++;
			break;
		case 16:
			driveRobot(Direction.FORWARD, 10, 60);
			autonStep++;
			break;
		case 17:
			waitRobot(2);
			autonStep++;
			break;
		}
	}

	private void scaleScaleR() {
		switch (autonStep) {
		case 1:

		}
	}

	private void scaleScaleL() {
		switch (autonStep) {
		case 1:

		}
	}

	private void scaleSwitchL() {
		switch (autonStep) {
		case 1:

		}
	}

	private void scaleSwitchR() {
		switch (autonStep) {
		case 1:

		}
	}

	private void switchScaleR() {
		switch (autonStep) {
		case 1:
			driveRobot(Direction.FORWARD, 55, 100);
			autonStep++;
			break;
		case 2:
			turnRobot(Direction.LEFT, 90, 100);
			autonStep++;
			break;
		case 3:
			driveRobot(Direction.FORWARD, 10, 100);
			autonStep++;
			break;
		case 4:
			waitRobot(2);
			autonStep++;
			break;
		case 5:
			driveRobot(Direction.REVERSE, 10, 100);
			autonStep++;
			break;
		case 6:
			turnRobot(Direction.RIGHT, 90, 100);
			autonStep++;
			break;
		case 7:
			driveRobot(Direction.FORWARD, 13, 100);
			autonStep++;
			break;
		case 8:
			turnRobot(Direction.LEFT, 90, 100);
			autonStep++;
			break;
		case 9:
			driveRobot(Direction.FORWARD, 13, 60);
			autonStep++;
			break;
		case 10:
			waitRobot(2);
			autonStep++;
			break;
		case 11:
			driveRobot(Direction.REVERSE, 13, 60);
			autonStep++;
			break;
		case 12:
			turnRobot(Direction.RIGHT, 90, 100);
			autonStep++;
			break;
		case 13:
			driveRobot(Direction.FORWARD, 40, 60);
			autonStep++;
			break;
		case 14:
			turnRobot(Direction.LEFT, 90, 100);
			autonStep++;
			break;
		case 15:
			driveRobot(Direction.FORWARD, 10, 60);
			autonStep++;
			break;
		case 16:
			waitRobot(2);
			autonStep++;
			break;
		}
	}

	private void switchScaleL() {
		switch (autonStep) {
		case 1:
			driveRobot(Direction.FORWARD, 55, 60);
			autonStep++;
			break;
		case 2:
			turnRobot(Direction.RIGHT, 90, 60);
			autonStep++;
			break;
		case 3:
			driveRobot(Direction.FORWARD, 10, 60);
			autonStep++;
			break;
		case 4:
			driveRobot(Direction.REVERSE, 10, 60);
			autonStep++;
			break;
		case 5:
			turnRobot(Direction.LEFT, 90, 60);
			autonStep++;
			break;
		case 6:
			driveRobot(Direction.FORWARD, 13, 60);
			autonStep++;
			break;
		case 7:
			turnRobot(Direction.RIGHT, 90, 60);
			autonStep++;
			break;
		case 8:
			driveRobot(Direction.FORWARD, 13, 60);
			autonStep++;
			break;
		case 9:
			driveRobot(Direction.REVERSE, 13, 60);
			autonStep++;
			break;
		case 10:
			turnRobot(Direction.LEFT, 90, 60);
			autonStep++;
			break;
		case 11:
			driveRobot(Direction.FORWARD, 40, 60);
			autonStep++;
			break;
		case 12:
			turnRobot(Direction.RIGHT, 90, 60);
			autonStep++;
			break;
		case 13:
			driveRobot(Direction.FORWARD, 10, 60);
			autonStep++;
			break;

		}
	}

	private void centerSwitchR() {
		switch (autonStep) {
		case 1:
			driveRobot(Direction.FORWARD, 5, 60);
			autonStep++;
			break;
		case 2:
			turnRobot(Direction.RIGHT, 40, 60);
			autonStep++;
			break;
		case 3:
			driveRobot(Direction.FORWARD, 30, 60);
			autonStep++;
			break;
		case 4:
			turnRobot(Direction.LEFT, 50, 60);
			autonStep++;
			break;
		case 5:
			driveRobot(Direction.FORWARD, 10, 30);
			autonStep++;
			break;
		}
	}

	private void centerSwitchL() {
		switch (autonStep) {
		case 1:
			driveRobot(Direction.FORWARD, 5, 60);
			autonStep++;
			break;
		case 2:
			turnRobot(Direction.LEFT, 40, 60);
			autonStep++;
			break;
		case 3:
			driveRobot(Direction.FORWARD, 30, 60);
			autonStep++;
			break;
		case 4:
			turnRobot(Direction.RIGHT, 50, 60);
			autonStep++;
			break;
		case 5:
			driveRobot(Direction.FORWARD, 10, 30);
			autonStep++;
			break;
		}
	}

	private void leftScale() {
		switch (autonStep) {
		case 1:
			driveRobot(Direction.FORWARD, 115, 60);
			autonStep++;
			break;
		case 2:
			turnRobot(Direction.RIGHT, 90, 60);
			autonStep++;
			break;
		}
	}

	private void rightScale() {
		switch (autonStep) {
		case 1:
			driveRobot(Direction.FORWARD, 115, 60);
			autonStep++;
			break;
		case 2:
			turnRobot(Direction.LEFT, 90, 60);
			autonStep++;
			break;
		}
	}

	private void leftSwitch() {
		switch (autonStep) {
		case 1:
			driveRobot(Direction.FORWARD, 55, 60);
			autonStep++;
			break;
		case 2:
			turnRobot(Direction.RIGHT, 90, 100);
			autonStep++;
			break;
		case 3:
			driveRobot(Direction.FORWARD, 10, 20);
			autonStep++;
			break;
		}
	}

	private void rightSwitch() {
		switch (autonStep) {
		case 1:
			driveRobot(Direction.FORWARD, 55, 60);
			autonStep++;
			break;
		case 2:
			turnRobot(Direction.LEFT, 90, 100);
			autonStep++;
			break;
		case 3:
			driveRobot(Direction.FORWARD, 10, 20);
			autonStep++;
			break;
		}
	}

	private void cubeAction(Action cubeAction, double distance, double powerPercent) {
		switch (cubeAction) {
		case UP:
			break;
		case DOWN:
			break;
		case OUTTAKE:
			break;
		case INTAKE:
			break;
		}
	}

	private void driveRobot(Direction driveDirection, double distance, double powerPercent) {
		robotAction(driveDirection, distance, powerPercent);
	}

	private void turnRobot(Direction driveDirection, double turnDegrees, double powerPercent) {
		robotAction(driveDirection, turnDegrees, powerPercent);
	}

	private void robotAction(Direction driveDirection, double distance, double powerPercent) {
		double startingLeftEncoder = leftEncoder.getDistance();
		double startingRightEncoder = rightEncoder.getDistance();
		double turnDegree = 0;
		double targetDegree = 0;

		switch (driveDirection) {
		case FORWARD:
			while (leftEncoder.getDistance() <= startingLeftEncoder + distance
					&& rightEncoder.getDistance() <= startingRightEncoder + distance) {
				_drive.arcadeDrive(powerPercent * .01, 0);
				double currentCubeHeight = desiredCubeHeight; // We need to
																// obtain the
																// height
																// encoder value
																// here to
																// determine if
																// the height
																// should change
																// while driving
				if (currentCubeHeight != desiredCubeHeight) {

				}
			}
			break;
		case REVERSE:
			while (leftEncoder.getDistance() >= startingLeftEncoder - distance
					&& rightEncoder.getDistance() >= startingRightEncoder - distance) {
				_drive.arcadeDrive(powerPercent * -.01, 0);
			}
			break;
		case LEFT:
			if (powerPercent > 85)
				powerPercent = 85;
			targetDegree = lastValidDirection - distance;
			while (targetDegree < imu.getAngleZ()) {
				SmartDashboard.putNumber("target Degree", targetDegree);
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
				_drive.arcadeDrive(0, .5);
			}
			break;
		case RIGHT:
			if (powerPercent > 85)
				powerPercent = 85;
			turnDegree = lastValidDirection + distance;
			while (turnDegree > imu.getAngleZ()) {
				// check if within 10 degrees and if so slow turn
				if (Math.abs(turnDegree - imu.getAngleZ()) > 10) {
					SmartDashboard.putNumber("speed", powerPercent * .01);
					_drive.arcadeDrive(0, powerPercent * .01);
				} else {
					_drive.arcadeDrive(0, .5);
				}
			}
			lastValidDirection += distance;
			// If we overshot on the turn, than correct
			while (lastValidDirection < imu.getAngleZ()) {
				_drive.arcadeDrive(0, -.5);
			}
			break;
		default:
			break;
		}
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
		// _drive.tankDrive();
		// _drive.mecanumDrive_Cartesian(adjXAxis, adjYAxis, adjRotation,
		// adjGyroAngle);
	}

	private void updateSmartDashboardData() {
		SmartDashboard.putNumber("encoderL", leftEncoder.getDistance());
		SmartDashboard.putNumber("encoderR", rightEncoder.getDistance());
		SmartDashboard.putBoolean("button1", _joy.getRawButton(1));
		SmartDashboard.putBoolean("button2", _joy.getRawButton(2));
		SmartDashboard.putNumber("IMU Angle Z", getDegreeZ());
		SmartDashboard.putNumber("Gyro Angle Z", imu.getAngleZ());
		SmartDashboard.putNumber("Last Valid", lastValidDirection);
		SmartDashboard.putNumber("actual Degree", imu.getAngleZ());
		SmartDashboard.putNumber("Auton Step", autonStep);
		SmartDashboard.putNumber("Timer", Timer.getMatchTime());
		SmartDashboard.putNumber("Test Encoder Distance Value", speedTestEncoder.getDistance());
		SmartDashboard.putNumber("Test Encoder Rate Value", speedTestEncoder.getRate());
		SmartDashboard.putNumber("Test Encoder Count", speedTestEncoder.get());
		SmartDashboard.putNumber("Test Encoder Raw", speedTestEncoder.getRaw());
		SmartDashboard.putString("FMS Data", gameData);
	}

	private void waitRobot(double waitTime) {
		// TODO Auto-generated method stub
		// double resumeTime = Timer.getMatchTime() - waitTime;
		// while (Timer.getMatchTime() > resumeTime) {
		// Do nothing
		// }
	}

}
