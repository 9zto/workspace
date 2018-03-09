/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3495.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3495.robot.commands.ExampleCommand;
import org.usfirst.frc.team3495.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static final ExampleSubsystem kExampleSubsystem
	= new ExampleSubsystem();
	public static OI m_oi;

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	TalonSRX t0 = new TalonSRX(0);
	TalonSRX t1 = new TalonSRX(1);
	TalonSRX t2 = new TalonSRX(2);
	TalonSRX t3 = new TalonSRX(3);
	TalonSRX climber = new TalonSRX(2);

	

	boolean tank = false;
	boolean slow2 = false;



	Joystick left = new Joystick(2);
	Joystick right = new Joystick(0);
	Joystick controller = new Joystick(1);

	JoystickButton b0 = new JoystickButton (right, 1);
	JoystickButton b1 = new JoystickButton (controller, 2);
	JoystickButton b2 = new JoystickButton (controller, 3);
	JoystickButton b3 = new JoystickButton (controller, 4);
	JoystickButton climb = new JoystickButton (controller, 5);
	JoystickButton slowSpeed = new JoystickButton (left, 1);
	
	JoystickButton normSpeed = new JoystickButton (right, 2);
	JoystickButton medSpeed = new JoystickButton (left, 2);
	
	
	int currentP = t1.getSelectedSensorPosition(1); //Gets up encoder position
	int currentV = t1.getSelectedSensorVelocity(1); 
	int currentP3 = t3.getSelectedSensorPosition(3);
	double inches;
	double inchestoticks(){
		return (inches/3.14 * 6)* 4096;
	}


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_oi = new OI();
		m_chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.e
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();
		t1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		t1.setSelectedSensorPosition(0, 0, 0);
		t3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		t3.setSelectedSensorPosition(0, 0, 0);
		//t1.configForwardSoftLimitEnable(isEnabled(), 20000);
		//t1.configForwardSoftLimitEnable(isEnabled(), 20000);
		/*t1.clearStickyFaults(0);
		t3.clearStickyFaults(0);
		t1.getSensorCollection();
		t1.getSelectedSensorPosition(1);
		t3.getSensorCollection();
		t1.getSelectedSensorPosition(1);*/
		
		//t1.overrideSoftLimitsEnable(isDisabled());
		//t3.overrideSoftLimitsEnable(isDisabled());
		inches  = 120;
		
		t1.set(ControlMode.Position, inches);
		t3.set(ControlMode.Position, -inches);
		System.out.println("Talon 1:");
		System.out.println(currentP);
		System.out.println("Talon 3:");
		System.out.println(currentP3);
		/*if(t1.getSelectedSensorPosition(1) < 10){
			tankDrive(.14,.13);
		}else{
			tankDrive(0,0);*/
		
		/*t1.set(ControlMode.Position, 0);//sets talon to position measuring
		t1.getSelectedSensorPosition(currentP);//prints position
		t1.setSelectedSensorPosition(0, 0, 0);
		currentP = 0;
		tankDrive(.25, .25);*/
			
		
		
		

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
			auton();
		}
		
		
		
		
		
	

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		/*
		t1.configNominalOutputForward(0, 10);
		t1.configNominalOutputReverse(0, 10);
		t1.configPeakOutputForward(1, 10);
		t1.configPeakOutputReverse(-1, 10);
		*/
		t1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);//sets the QuadEncoder
		//t1.setSensorPhase(true);
		t3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);//sets the QuadEncoder
		//t3.setSensorPhase(true);
		/*
		t1.configAllowableClosedloopError(0, 0, 10);
		t1.config_kP(0, 0.1, 10);//sets your P value
		t1.config_kI(0, 0.0, 10);//sets your I value
		t1.config_kD(0, 0.0, 10);//sets your D value
		t1.setStatusFramePeriod(10, 10, 10);
		*/
		//t1.set(ControlMode.Position, 10);//sets talon to position measuring
		//t3.set(ControlMode.Position, 10);
		//t1.setSelectedSensorPosition(0, 0, 10);//sets position of encoder to 0
		//t3.setSelectedSensorPosition(0, 0, 10);
		
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() { 
		Scheduler.getInstance().run();
		
	
		t1.getSensorCollection();
		t1.getSelectedSensorPosition(1);
		System.out.println(t1.getSelectedSensorPosition(1));//prints position
		
		
		if(b0.get()){
			tankDrive(left.getY()*.5, right.getY()*.5);
		}
		else{
			tankDrive(left.getY(), right.getY());
		}
		

	//	climber();
		
		
	}
	

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	public void tankDrive(double left, double right){
		t1.set(ControlMode.PercentOutput,left);
		t3.set(ControlMode.PercentOutput,-right);
		t0.set(ControlMode.PercentOutput, left);
		t2.set(ControlMode.PercentOutput, -right);

		
		//t0.set(ControlMode.Follower, 1);
		//t2.set(ControlMode.Follower, 3);
	}

	
	void auton(){
		/*if(currentP < 10){
			t1.getSelectedSensorPosition(currentP);
			tankDrive(.13,.14);
			System.out.println(currentP);
		}else{
			t1.getSelectedSensorPosition(currentP);
			tankDrive(0,.0);
			System.out.println(currentP);
			*/
		/*t1.configForwardSoftLimitEnable(isEnabled(), 7700);
		t3.configForwardSoftLimitEnable(isEnabled(), 7700);
		t1.configReverseSoftLimitEnable(isEnabled(), -7700);
		t3.configReverseSoftLimitEnable(isEnabled(), -7700);
		*/
		System.out.println("Talon 1:");
		System.out.println(currentP);
		System.out.println("Talon 3:");
		System.out.println(currentP3);
		System.out.println("Inches");
		System.out.println(inchestoticks());
	}
}
		/*
	}
}
	//public void climber(){
	//	if(climb.get()){
	//		climber.set(1);

	//	}
//}
*/