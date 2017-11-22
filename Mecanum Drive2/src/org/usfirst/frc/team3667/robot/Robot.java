package org.usfirst.frc.team3667.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
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
 /*test change */
public class Robot extends IterativeRobot {
	//Command autonomousCommand;
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
     //autonomousCommand = new move() ;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
     
     _drive.mecanumDrive_Cartesian(_joy.getRawAxis(0), _joy.getRawAxis(1), _joy.getRawAxis(4), 0);
     if(_joy.getRawAxis(2) != 0) {
    	 _climber.set(_joy.getRawAxis(2));
     }
     else {
     _climber.set(_joy.getRawAxis(3) * -1.0);
     }
    }
    
   // public void autonomousInit(){
    	//If (autonomousCommand != null) autonomousCommand.start();
   
   // }
    
    
    
 //   public void autonomous() {
 //   	autoTime.start();
 //   	autoTime.reset();
 //   	while (autoTime.get() < 5000)
 //   	{
 //   		_drive.mecanumDrive_Cartesian(0.0, 0.5, 0.0, 0);
 //   	}
 //       autoTime.stop();	
 //	}

}

