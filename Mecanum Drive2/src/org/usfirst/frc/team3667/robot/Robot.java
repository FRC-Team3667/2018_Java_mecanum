package org.usfirst.frc.team3667.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon;
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
	SendableChooser autoChooser;

	public static final double kPulsesPerRevolution = 1024; // for an AS5145B
															// Magnetic Encoder
	public static final double kDistancePerPulse = kDistancePerRevolution / kPulsesPerRevolution;
	private Encoder leftEncoder = new Encoder(0, 1, true, EncodingType.k4X);
	private Encoder rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
	private RobotDrive drive = new RobotDrive(1, 2);
	Timer autoTime = new Timer();
	CANTalon _frontLeftMotor = new CANTalon(13);
	CANTalon _frontRightMotor = new CANTalon(12);
	CANTalon _rearRightMotor = new CANTalon(11);
	CANTalon _rearLeftMotor = new CANTalon(10);
	CANTalon _climber = new CANTalon(14);
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
		leftEncoder.reset();
		rightEncoder.reset();

		autoChooser = new SendableChooser();
		autoChooser.addDefault("Default program", new Pickup());
		autoChooser.addObject("Experimental auto", new ElevatorPickup());
		SmartDashboard.putData("Autonomous mode chooser", autoChooser);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		adjustedDrive(_joy.getRawAxis(0), _joy.getRawAxis(1), _joy.getRawAxis(4), 0);
		if (_joy.getRawAxis(2) != 0) {
			_climber.set(_joy.getRawAxis(2));
		} else {
			_climber.set(_joy.getRawAxis(3) * -1.0);
		}

		SmartDashboard.putNumber("encoderL", leftEncoder.getDistance());
		SmartDashboard.putNumber("encoderR", rightEncoder.getDistance());

		// Display button values
		SmartDashboard.putBoolean("button1", _joy.getRawButton(1));
		SmartDashboard.putBoolean("button2", _joy.getRawButton(2));

		// Timer.delay(0.005); // wait for a motor update time

		SmartDashboard.putNumber("IMU Angle Z", getDegreeZ());

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

	public void autonomousInit() {
		// if (autoChooser.getSelected() != null) {
		// autonomousCommand = (Command) autoChooser.getSelected();
		// autonomousCommand.start();
		// }
		imu.reset();
		leftEncoder.reset();
		rightEncoder.reset();
		autonStep = 1;
		lastValidDirection = 0;
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putNumber("IMU Angle Z", getDegreeZ());
		SmartDashboard.putNumber("encoderL", leftEncoder.getDistance());
		SmartDashboard.putNumber("encoderR", rightEncoder.getDistance());
		SmartDashboard.putNumber("Last Valid Direction", lastValidDirection);
		SmartDashboard.putNumber("Auton Step", autonStep);
		SmartDashboard.putNumber("Actual Gryo", imu.getAngleZ());
		int planNum = 5;
		switch (planNum) {
		case 0: {
			if (Timer.getMatchTime() > 10.5) {
				adjustedDrive(0, -.3, 0, 0);
			} else if (Timer.getMatchTime() > 9.75) {
				adjustedDrive(0, 0, 0.5, 0);
			} else {
				adjustedDrive(0, -.1, 0, 0);
			}
			break;
		}
		case 1: {
			if (Timer.getMatchTime() > 10.5) {
				adjustedDrive(0, -.3, 0, 0);
			} else if (Timer.getMatchTime() > 9.75) {
				adjustedDrive(0, 0, -0.5, 0);
			} else {
				adjustedDrive(0, -.1, 0, 0);
			}
			break;
		}
		case 2: {

			if (leftEncoder.getDistance() < 120 && rightEncoder.getDistance() < 120) {
				adjustedDrive(0, -.4, 0, 0);
			} else if (leftEncoder.getDistance() <= 122 && rightEncoder.getDistance() >= 120) {
				adjustedDrive(0, 0, -0.5, 0);
			} else if (leftEncoder.getDistance() <= 107 && rightEncoder.getDistance() >= 136) {
				adjustedDrive(0, -.1, 0, 0);
			} else if (leftEncoder.getDistance() >= 112 && rightEncoder.getDistance() >= 141) {
				_climber.set(.5);
			}
			break;
		}
		case 3: {
			// from right starting position-right switch
			switch (autonStep) {
			case 1: {
				driveForward(-.6, 49);
				autonStep++;
				break;
			}
			case 2:
				// turnRobot(270);
				turnLeft(90, 50);
				autonStep++;
				break;
			}
		}
		case 4: {
			// from right starting position-right switch
			switch (autonStep) {
			case 1: {
				driveForward(-.6, 49);
				autonStep++;
				break;
			}
			case 2:
				// turnRobot(270);
				turnRight(90, 50);
				autonStep++;
				break;
			}
		}
		case 5: {
			// from right starting position-right scale
			switch (autonStep){
			case 1: {
				driveForward(-.8, 60);
				autonStep++;
				break;
			}
			}
		}
		}

		SmartDashboard.putNumber("Timer", Timer.getMatchTime());
	}

	private void adjustedDrive(double xAxis, double yAxis, double rotation, double gyroAngle) {
		double curHeight = 0;
		// Round to the nearest Hundredth to remove game pad inaccuracy 
		double adjXAxis = Math.round(xAxis * 10.0) / 10.0;
		double adjYAxis = Math.round(yAxis * 10.0) / 10.0;
		double adjRotation = Math.round(rotation * 10.0) / 10.0;
		double adjGyroAngle = Math.round(gyroAngle * 10.0) / 10.0;
		double minSpeedNum = .1;
		double maxSpeedNum = 1;
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

	private void turnRight(double turnDegrees, double speedPercent) {
		double curDegree = imu.getAngleZ();
		double targetDegree = lastValidDirection + turnDegrees;

		while (targetDegree > imu.getAngleZ()) {
			SmartDashboard.putNumber("target Degree", targetDegree);

			if (Math.abs(targetDegree - imu.getAngleZ()) > 10) { // check if
																	// within 10
																	// degrees
																	// and if so
																	// slow turn
				SmartDashboard.putNumber("speed", speedPercent * .01);
				_drive.mecanumDrive_Cartesian(0, 0, speedPercent * .01, 0);
			} else {
				_drive.mecanumDrive_Cartesian(0, 0, .25, 0);
			}
		}
		SmartDashboard.putNumber("Actual Gryo", imu.getAngleZ());
		lastValidDirection -= turnDegrees;
	}

	private void turnLeft(double turnDegrees, double speedPercent) {
		double curDegree = imu.getAngleZ();
		double targetDegree = lastValidDirection - turnDegrees;

		while (targetDegree < imu.getAngleZ()) {
			SmartDashboard.putNumber("target Degree", targetDegree);

			if (Math.abs(targetDegree - imu.getAngleZ()) > 10) { // check if
																	// within 10
																	// degrees
																	// and if so
																	// slow turn
				SmartDashboard.putNumber("speed", speedPercent * -.01);
				_drive.mecanumDrive_Cartesian(0, 0, speedPercent * -.01, 0);
			} else {
				_drive.mecanumDrive_Cartesian(0, 0, -.25, 0);
			}
		}
		SmartDashboard.putNumber("Actual Gryo", imu.getAngleZ());
		lastValidDirection -= turnDegrees;
	}

	private void driveForward(double travelSpeed, double travelDistance) {
		double startingLeftEncoder = leftEncoder.getDistance();
		double startingRightEncoder = rightEncoder.getDistance();
		while (leftEncoder.getDistance() <= startingLeftEncoder + travelDistance
				&& rightEncoder.getDistance() <= startingRightEncoder + travelDistance) {
			_drive.mecanumDrive_Cartesian(0, travelSpeed, 0, 0);
		}
	}
}
