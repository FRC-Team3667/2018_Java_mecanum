package org.usfirst.frc.team3667.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon;
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
	public static final double kDistancePerRevolution = 18.84; // guestimate
																// from your
																// code
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

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		_drive.setInvertedMotor(MotorType.kFrontRight, true);
		_drive.setInvertedMotor(MotorType.kRearRight, true);
		// autonomousCommand = new move() ;
		
		//start of encoders
		leftEncoder.setDistancePerPulse(kDistancePerPulse);
        rightEncoder.setDistancePerPulse(kDistancePerPulse);
        leftEncoder.reset();
        rightEncoder.reset();
        
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		_drive.mecanumDrive_Cartesian(_joy.getRawAxis(0), _joy.getRawAxis(1), _joy.getRawAxis(4), 0);
		if (_joy.getRawAxis(2) != 0) {
			_climber.set(_joy.getRawAxis(2));
		} else {
			_climber.set(_joy.getRawAxis(3) * -1.0);
		}
		
		
		//encoder logic
		if (_joy.getRawButton(2) == true)
		{
			leftEncoder.reset();
			rightEncoder.reset();
		}
		SmartDashboard.putNumber("encoderL", leftEncoder.getDistance());
		SmartDashboard.putNumber("encoderR", rightEncoder.getDistance());
		
		// Display button values
		SmartDashboard.putBoolean("button1", _joy.getRawButton(1));
		SmartDashboard.putBoolean("button2", _joy.getRawButton(2));
	}

	public void autonomousInit()
	{
		SmartDashboard.putString("statusInit", "Initialize auton");
	}
	
	public void autonomousPeriodic()
	{
		SmartDashboard.putString("status", "Auton periodic");
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
