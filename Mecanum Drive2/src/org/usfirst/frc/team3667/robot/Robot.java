package org.usfirst.frc.team3667.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
//import edu.wpi.first.wpilibj.SampleRobot;
//import edu.wpi.first.wpilibj.Sendable;
//import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
		LEFTSWITCH, RIGHTSWITCH, LEFTSCALE, RIGHTSCALE, CENTERSWITCHR, CENTERSWITCHL, SWITCHSCALER, SWITCHSCALEL, SCALESWITCHR, SCALESWITCHL, SCALESCALEL, SWITCHSWITCHL, SWITCHSWITCHR, SCALESCALER
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
	RobotDrive _drive = new RobotDrive(_frontLeftMotor, _rearLeftMotor, _frontRightMotor, _rearRightMotor);
	Joystick _joy = new Joystick(0);
	Joystick _climberJoy = new Joystick(1);

	int autonStep = 1;

	double lastValidDirection = 0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		imu = new ADIS16448_IMU();
		imu.calibrate();
		imu.reset();

		_drive.setInvertedMotor(MotorType.kFrontRight, true);
		_drive.setInvertedMotor(MotorType.kRearRight, true);
		// autonomousCommand = new move() ;

		// start of encoders
		leftEncoder.setDistancePerPulse(kDistancePerPulse);
		rightEncoder.setDistancePerPulse(kDistancePerPulse);
		speedTestEncoder.setDistancePerPulse(kDistancePerPulse);
		leftEncoder.reset();
		rightEncoder.reset();
		speedTestEncoder.reset();

		// autoChooser = new SendableChooser();
		// autoChooser.addDefault("Default program", new Pickup());
		// autoChooser.addObject("Experimental auto", new ElevatorPickup());
		// SmartDashboard.putData("Autonomous mode chooser", autoChooser);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		// Update the Smart Dashboard Data
		updateSmartDashboardData();

		adjustedDrive(_joy.getRawAxis(0), _joy.getRawAxis(1), _joy.getRawAxis(4), 0);
		if (_joy.getRawAxis(2) != 0) {
			_climber.set(_joy.getRawAxis(2));
		} else {
			_climber.set(_joy.getRawAxis(3) * -1.0);
		}

		// Test auton
		if (_joy.getRawButton(4)) {
			testAutonomousPeriodic();
		}
		// reset auton
		if (_joy.getRawButton(3)) {
			initAndResetAll();
		}
	}

	// No code to be added in autonomousInit - Instead add code to
	// initAndResetAll
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
		// This is the 3667 play book for Autonomous options
		AutonPlays curPlay = AutonPlays.SWITCHSWITCHL;
		switch (curPlay) {
		case LEFTSCALE:
			leftScale();
			break;
		case LEFTSWITCH:
			leftSwitch();
			break;
		case RIGHTSCALE:
			rightScale();
			break;
		case RIGHTSWITCH:
			rightSwitch();
			break;
		case CENTERSWITCHR:
			centerSwitchR();
			break;
		case CENTERSWITCHL:
			centerSwitchL();
			break;
		case SWITCHSCALER:
			switchScaleR();
			break;
		case SCALESWITCHR:
			scaleSwitchR();
			break;
		case SCALESWITCHL:
			scaleSwitchL();
			break;
		case SCALESCALEL:
			scaleScaleL();
			break;
		case SCALESCALER:
			scaleScaleR();
			break;
		case SWITCHSWITCHL:
			switchSwitchL();
			break;
		case SWITCHSWITCHR:
			switchSwitchR();
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
			waitRobot(2);
			autonStep++;
			break;
		case 11:
			driveRobot(Direction.REVERSE, 13, 60);
			autonStep++;
			break;
		case 12:
			turnRobot(Direction.RIGHT, 90, 60);
			autonStep++;
			break;
		case 13:
			driveRobot(Direction.FORWARD, 40, 60);
			autonStep++;
			break;
		case 14:
			turnRobot(Direction.LEFT, 90, 60);
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
		double startingLeftEncoder = leftEncoder.getDistance();
		double startingRightEncoder = rightEncoder.getDistance();

		switch (driveDirection) {
		case FORWARD:
			while (leftEncoder.getDistance() <= startingLeftEncoder + distance
					&& rightEncoder.getDistance() <= startingRightEncoder + distance) {
				_drive.mecanumDrive_Cartesian(0, powerPercent * -.01, 0, 0);
			}
			break;
		case LEFT: // No action
			break;
		case REVERSE:
			while (leftEncoder.getDistance() >= startingLeftEncoder - distance
					&& rightEncoder.getDistance() >= startingRightEncoder - distance) {
				_drive.mecanumDrive_Cartesian(0, powerPercent * .01, 0, 0);
			}
			break;
		case RIGHT: // No action
			break;
		default:
			break;
		}
	}

	private void turnRobot(Direction driveDirection, double turnDegrees, double powerPercent) {
		// Let's cap the power percent to prevent issues
		if (powerPercent > 50) {
			powerPercent = 50;
		} else if (powerPercent < 20) {
			powerPercent = 20;
		}
		double turnDegree = 0;
		double targetDegree = 0;
		switch (driveDirection) {
		case FORWARD: // No action
			break;
		case LEFT:
			targetDegree = lastValidDirection - turnDegrees;
			while (targetDegree < imu.getAngleZ()) {
				SmartDashboard.putNumber("target Degree", targetDegree);
				// check if within 10 degrees and if so slow turn
				if (Math.abs(targetDegree - imu.getAngleZ()) > 10) {
					SmartDashboard.putNumber("speed", powerPercent * -.01);
					_drive.mecanumDrive_Cartesian(0, 0, powerPercent * -.01, 0);
				} else {
					_drive.mecanumDrive_Cartesian(0, 0, -.25, 0);
				}
			}
			lastValidDirection -= turnDegrees;
			// If we overshot on the turn, than correct
			while (lastValidDirection > imu.getAngleZ()) {
				_drive.mecanumDrive_Cartesian(0, 0, .25, 0);
			}
			break;
		case REVERSE: // No action
			break;
		case RIGHT:
			turnDegree = lastValidDirection + turnDegrees;
			while (turnDegree > imu.getAngleZ()) {
				// check if within 10 degrees and if so slow turn
				if (Math.abs(turnDegree - imu.getAngleZ()) > 10) {
					SmartDashboard.putNumber("speed", powerPercent * .01);
					_drive.mecanumDrive_Cartesian(0, 0, powerPercent * .01, 0);
				} else {
					_drive.mecanumDrive_Cartesian(0, 0, .25, 0);
				}
			}
			lastValidDirection += turnDegrees;
			// If we overshot on the turn, than correct
			while (lastValidDirection < imu.getAngleZ()) {
				_drive.mecanumDrive_Cartesian(0, 0, -.25, 0);
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
		_drive.mecanumDrive_Cartesian(adjXAxis, adjYAxis, adjRotation, adjGyroAngle);
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
	}

	private void waitRobot(double waitTime) {
		// TODO Auto-generated method stub
		double resumeTime = Timer.getMatchTime() - waitTime;
		while (Timer.getMatchTime() > resumeTime) {
			// Do nothing
		}
	}

}
