/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3495.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3495.robot.commands.ExampleCommand;
import org.usfirst.frc.team3495.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
	
	Joystick left = new Joystick(0);
	Joystick right = new Joystick(2);
	Joystick controller = new Joystick(1);
	Joystick arm = new Joystick(3);

	JoystickButton armlift = new JoystickButton(left, 1);
	JoystickButton intake =  new JoystickButton(arm, 5);
	JoystickButton intakeReverse = new JoystickButton(arm, 4);
	JoystickButton configPos = new JoystickButton(arm, 3);
	JoystickButton fivePercent = new JoystickButton(arm, 2);
	JoystickButton throttle = new JoystickButton(arm, 1);
	
	TalonSRX t0 = new TalonSRX(0);
	TalonSRX t1 = new TalonSRX(1);
	TalonSRX t2 = new TalonSRX(5);
	TalonSRX t3 = new TalonSRX(6);
	TalonSRX armLeft = new TalonSRX(8);
	TalonSRX armRight = new TalonSRX(2);
	TalonSRX intakeLeft = new TalonSRX(9);
	TalonSRX intakeRight = new TalonSRX(4);

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
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();

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
		t0.neutralOutput();
		t1.neutralOutput();
		t2.neutralOutput();
		t3.neutralOutput();
	}
	

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		arm(arm.getY()); 
		tankDrive(left.getY(), right.getY());
		
		if (intake.get()) {
			intakeLeft.set(ControlMode.PercentOutput,-.75);
			intakeRight.set(ControlMode.PercentOutput, .75);
		}else if(intakeReverse.get()){
			intakeLeft.set(ControlMode.PercentOutput,.75);
			intakeRight.set(ControlMode.PercentOutput, -.75);
		}else if(configPos.get()){
			intakeLeft.set(ControlMode.PercentOutput,.30);
			intakeRight.set(ControlMode.PercentOutput, .30);
		}
		else{
			intakeLeft.set(ControlMode.PercentOutput, 0);
			intakeRight.set(ControlMode.PercentOutput, 0);
		}
		
		if(throttle.get()){
			armRight.set(ControlMode.PercentOutput, -.1);
			armLeft.set(ControlMode.PercentOutput, .1);
		}else if(fivePercent.get()){
			armRight.set(ControlMode.PercentOutput, -.05);
			armLeft.set(ControlMode.PercentOutput, .05);
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void tankDrive(double left, double right){

		t1.set(ControlMode.PercentOutput, right);
		t0.set(ControlMode.Follower, 1);
		t2.set(ControlMode.PercentOutput, -left);
		t3.set(ControlMode.Follower, 3);
	}
	
	public void arm(double right){
		armLeft.set(ControlMode.PercentOutput, right);
		//t0.set(ControlMode.Follower, 1);

		armRight.set(ControlMode.PercentOutput, -right);
		//t3.set(ControlMode.Follower, 3);
	}
	
}
