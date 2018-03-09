
package org.usfirst.frc.team3495.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3495.robot.commands.ExampleCommand;
import org.usfirst.frc.team3495.robot.subsystems.ExampleSubsystem;

import com.ctre.CANTalon;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	NetworkTable camera = NetworkTable.getTable("GRIP/contours");

	NetworkTable table = NetworkTable.getTable("SmartDashboard");
	
	CANTalon t0 = new CANTalon(5);
	CANTalon t1 = new CANTalon(3);
	CANTalon t2 = new CANTalon(0);
	CANTalon t3 = new CANTalon(8); 
	CANTalon climbert = new CANTalon(1);


	Joystick left = new Joystick(0);
	Joystick right = new Joystick(2);
	Joystick controller = new Joystick(1);
	

	JoystickButton climberb = new JoystickButton(controller, 1);
	Encoder enc1 = new Encoder(2, 3, true);
	Encoder enc2 = new Encoder(4,5, true);
//  Device  Name   device info(port a, port b, forwards or backwards, device.Indextype.IndexTypeName)
	// Indexing is the storage of data, kind of like taking an order at a restaurant, the specific attributes you need. Encoders have three
//	types of Indexes. K1x: Counts forward only, the standard encoder type. K2X: Counts in both directions K4X: Counts all values.
	//There are many different types of encoders, some of which we might cover during Madtown, but the one we use is called the AMT103-V CUI Encoder. 
	//You don't need to know this, but remember that this is a Quadrature Axial Encoder, Axial referring to its position on the axle and quadrature 
	//to a lot of complex math that is really cool but not necessary to learn. If you're interested, go to:
	//https://wpilib.screensteplive.com/s/currentCS/m/java/599717-encoders-measuring-the-rotation-of-a-wheel-or-other-shaft
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	int climberv = 0;
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
		oi = new OI();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		
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
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();
		auton1();
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
		Scheduler.getInstance().run();
		CameraServer.getInstance().startAutomaticCapture();
		tankDrive(left.getY(), right.getY());
		Climber();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	void auton1(){
		tankDrive(.25, .25);
		Timer.delay(3);
		tankDrive(0, 0);
		Timer.delay(1);
		tankDrive(-.5, -.5);
		Timer.delay(3);
		tankDrive(0, 0);
	}
	
	void auton5(){
		tankDrive(-.5, -.5);
		Timer.delay(3);
		tankDrive(0, 0);
		Timer.delay(.5);
		tankDrive(-.5, -.5);
		Timer.delay(1);
		tankDrive(0,0);

	}
	void auton2(){
		enc1.reset();
		enc1.setDistancePerPulse(.2617);
		while(enc1.getDistance() < 78 || enc2.getDistance() < 78){
			if(enc1.getDistance() < 78 && enc2.getDistance() < 78){
				tankDrive(-.5, -.5);
			}else if(enc1.getDistance() < 78){
				tankDrive(-.5, 0);
			}else if(enc2.getDistance() < 78){
				tankDrive(0, -.5);
			}else{
				break;
			}
		}
	}
	void auton3(){
		enc1.reset();
		enc1.setDistancePerPulse(.2617);
		while(enc1.getDistance() < 100){

			System.out.println("Encoder 1: " + enc1.getDistance());
			tankDrive(-.25, -.25);
			enc1.getDistance();
		}
		tankDrive(0,0);
			}
	

	public void tic(double left, double right){
		t0.set(-left);
		t1.set(-left);
		t2.set(right);
		t3.set(right);
	}
/**	public void Climber2(){



		if(climbert.getOutputVoltage() > 500){
			climberv = climberv + 1;
		}

		if(climberb.get()){
			climbert.set(.5);
		}else if(climberv == 1){
			climbert.set(1);
		}else if(climberv == 2){
			climbert.set(0);
			climberv = 0;
		}else{
			climbert.set(0);
		}

	}
**/
	public void Climber(){ 
	if(climberb.get() && climbert.getOutputCurrent() < 2){
		climbert.set(.3);
	}else if(climberb.get() && climbert.getOutputCurrent() > 2){
		climbert.set(.8);
	}else{
		climbert.set(0);
			
			
		}
	}
}

