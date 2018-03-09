
package org.usfirst.frc.team3495.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3495.robot.commands.ExampleCommand;
import org.usfirst.frc.team3495.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	Talon t0 = new Talon(0);
	Talon t1 = new Talon(1);
	Talon t2 = new Talon(2);
	Talon t3 = new Talon(3);
	Talon t4 = new Talon(4);
	Talon t5 = new Talon(5);
	Talon t6 = new Talon(6);
	Talon t7 = new Talon(7);

	Joystick left = new Joystick (0);
	Joystick right = new Joystick (2);
	Joystick controller = new Joystick (1);


	JoystickButton b1 = new JoystickButton(controller, 1);
	JoystickButton b2 = new JoystickButton(controller, 2);
	JoystickButton b3 = new JoystickButton(controller, 3);







	//FLYWHEEL MOTOR
	//TANKDRIVE
	//AUTON forward 3 sec, flywheel full on 3 sec, stop, backwards 3 sec 50% power

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();




	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	// chooser.addObject("My Auto", new MyAutoCommand());
	//SmartDashboard.putData("Auto mode", chooser);


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
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();
		/**
		 * robot
		 *6 Talons
		 *joysticks 
		 *joystick button
		 *tankdrive
		 *flywheel - motor
		 *climber - motor
		 */

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
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
		if (autonomousCommand != null)
			autonomousCommand.cancel();

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {

		TankDrive(left.getY(), right.getY());
		
		flywheel();


		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}

	public void TankDrive(double left, double right){
		t0.set(left);
		t1.set(left);
		t2.set(-right);
		t3.set(-right);

	}

	public void tankDrive(double left, double right){
		t0.set(left);
		t1.set(left);
		t2.set(-right);
		t3.set(-right);
	}

	public void flywheel(){
		if (b1.get()) {
			t1.set(1);
		}else{
			t1.set(0);
		}
	}
}










