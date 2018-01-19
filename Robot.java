package org.usfirst.frc.team3667.robot;

// encoder import
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
// other imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	// Command autonomousCommand;

	// gyro variables
	// private Gyro gyro;

	public static final double kDistancePerRevolution = 18.84;
	public static final double kPulsesPerRevolution = 1024;

	public static final double kDistancePerPulse = kDistancePerRevolution / kPulsesPerRevolution;
	private Encoder leftEncoder = new Encoder(0, 1, true, EncodingType.k4X);
	private Encoder rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);

	Timer autoTime = new Timer();
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(13);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(12);
	WPI_TalonSRX _rearRightMotor = new WPI_TalonSRX(11);
	WPI_TalonSRX _rearLeftMotor = new WPI_TalonSRX(10);
	WPI_TalonSRX _climber = new WPI_TalonSRX(14);
	MecanumDrive _drive = new MecanumDrive(_frontLeftMotor, _rearLeftMotor, _frontRightMotor, _rearRightMotor);
	Joystick _joy = new Joystick(0);
	Joystick _climberJoy = new Joystick(1);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		// autonomousCommand = new move() ;

		// start of encoders
		leftEncoder.setDistancePerPulse(kDistancePerPulse);
		rightEncoder.setDistancePerPulse(kDistancePerPulse);
		leftEncoder.reset();
		rightEncoder.reset();

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		_drive.driveCartesian(_joy.getRawAxis(0), _joy.getRawAxis(1), _joy.getRawAxis(4), 0);
		if (_joy.getRawAxis(2) != 0) {
			_climber.set(_joy.getRawAxis(2));
		} else {
			_climber.set(_joy.getRawAxis(3) * -1.0);
		}

		// encoder logic
		if (_joy.getRawButton(2) == true) {
			leftEncoder.reset();
			rightEncoder.reset();
		}
		SmartDashboard.putNumber("encoderL", leftEncoder.getDistance());
		SmartDashboard.putNumber("encoderR", rightEncoder.getDistance());

		// Display button values
		SmartDashboard.putBoolean("button1", _joy.getRawButton(1));
		SmartDashboard.putBoolean("button2", _joy.getRawButton(2));

		// Gyro logic
		/*
		 * if (_joy.getRawButton(1) == true) { gyro.reset(); } double angle =
		 * gyro.getAngle(); SmartDashboard.putNumber("angle", angle);
		 */
	}

	public void autonomousInit() {
		SmartDashboard.putString("statusInit", "Initialize auton");
	}

	public void autonomousPeriodic() {
		int sdNum = 0;
		//Sendable sdNum = SmartDashboard.getData("Decision");
		switch (sdNum) {
		case 0: {
			if (Timer.getMatchTime() > 9.75) {
				_drive.driveCartesian(0, 0, -0.5, 0);
			} else {
				_drive.driveCartesian(0, -.1, 0, 0);
			}
			SmartDashboard.putNumber("Timer", Timer.getMatchTime());
			break;
		}
		case 1: 
			{
				
				break;
			}
		}
	}

	// public void autonomous() {
	// autoTime.start();
	// autoTime.reset();
	// while (autoTime.get() < 5000)
	// {
	// _drive.mecanumDrive_Cartesian(0.0, 0.5, 0.0, 0);
	// }
	// autoTime.stop();
	// }

}
