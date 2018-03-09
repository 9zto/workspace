
package org.usfirst.frc.team3495.robot;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Servo;
//import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import java.util.Set;

import org.usfirst.frc.team3495.robot.commands.ExampleCommand;
import org.usfirst.frc.team3495.robot.subsystems.ExampleSubsystem;

import com.ctre.CANTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;

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

	AnalogPotentiometer pot = new AnalogPotentiometer(0);

	double[] area;
	double[] x;

	boolean turnLeft;
	boolean turnRight;
	boolean center;
	boolean aligned = false;

	 CANTalon t1 = new CANTalon(3);
	CANTalon t2 = new CANTalon(0);
	CANTalon t0 = new CANTalon(5); 
	CANTalon climbert = new CANTalon(2);
	CANTalon t3 = new CANTalon (8);


	//Talon gear = new Talon(8);
	//Talon screw = new Talon(9);

	Joystick left = new Joystick(2);
	Joystick right = new Joystick(0);
	Joystick controller = new Joystick(1);

	double lastCheck = 0;
	boolean firstRun = true;

	JoystickButton climberb = new JoystickButton(controller, 3); 
	Encoder enc1 = new Encoder(2, 3,true);
	Encoder enc2 = new Encoder(4, 5, true);
	//JoystickButton spitGear = new JoystickButton(controller, 2); 
	//	JoystickButton pistons1 = new JoystickButton(controller, 5); 
	//	JoystickButton defaultGear = new JoystickButton(controllser, 2);
	//JoystickButton servoB = new JoystickButton(controller, 6);//Reverses Intake
	//JoystickButton shooterb = new JoystickButton(controller, 4);
	//JoystickButton comp = new JoystickButton(controller, 7);
	//	JoystickButton gearDown = new JoystickButton(controller, 3);
	//JoystickButton gearIn = new JoystickButton(controller, 1);
	//JoystickButton toggleLock = new JoystickButton(controller, 4);
	//JoystickButton reverseServo = new JoystickButton(controller, 3);

	//DoubleSolenoid small = new DoubleSolenoid(1, 4, 5);//good
	//DoubleSolenoid gearPiston = new DoubleSolenoid(1, 4, 5); //bad
	//DoubleSolenoid intakeG = new DoubleSolenoid(1, 0, 1);

	DigitalInput gearThing = new DigitalInput(0);
	//DigitalInput b2 = new DigitalInput(1);

	//Servo servo = new Servo(3);

	String currentAuton;

	boolean bool = true;
	boolean bool1 = true;
	boolean hasGear = false;
	boolean boolc = true;
	//boolean boolc = true;

	int climberv = 0;

	Compressor compressor;
	//	Thread visionThread;


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		chooser.addDefault("Default Auto", new ExampleCommand());
		hasGear = false;
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		compressor = new Compressor(1);
		CameraServer.getInstance().startAutomaticCapture();
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
		auton1();
		autonomousCommand = chooser.getSelected();
		//		currentAuton = table.getString("Auto Selector");
		enc1.setDistancePerPulse(.2617);
		//enc2.setDistancePerPulse(.2617);
		/*d3.enableBrakeMode(true);
		d0.enableBrakeMode(true);
		d5.enableBrakeMode(true);
		d8.enableBrakeMode(true);*9
		auton1();
		//		switch(currentAuton){
		//		case "auton1":
		//			auton1();
		//			break;
		//		case "auton2":
		//			auton2();
		//			break;
		//		case "gear3":
		//			gear3();
		//			break;
		//		}




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

		//System.out.println("Encoder 2: " + enc2.get());
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		bool = true;
		//		small.set(Value.kReverse);
		//		big.set(Value.kReverse);
		/*d3.enableBrakeMode(false);
		d0.enableBrakeMode(false);
		d5.enableBrakeMode(false);
		d8.enableBrakeMode(false);*/
		//servo.setAngle(140);
		enc1.reset();
		if (autonomousCommand != null)
			autonomousCommand.cancel();

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();	

		//System.out.println("Encoder Value 2: " + enc2.getDistance());


		TankDrive(left.getY(), right.getY());

		enc1.setDistancePerPulse(.2617);
		System.out.println("Encoder Value: " + enc1.getDistance());
		enc2.setDistancePerPulse(.2617);
		System.out.println("Encoder 2 Value: " + enc2.getDistance());
		//gearSystem();
		
		climber3();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
		//System.out.println(sonic.getRangeInches());



	}

	public void TankDrive (double left, double right){
		t0.set(ControlMode.PercentOutput, left);
		t1.set(ControlMode.Follower, left);
		t2.set(ControlMode.PercentOutput, -right);
		t3.set(ControlMode.Follower, -right);
	
	}	

	public void climber3(){
		if (climberb.get()){
			climbert.set(ControlMode.PercentOutput, .8);
		}else{
			climbert.set(ControlMode.PercentOutput, 0);
		}
	}

	public void climber(){



		if(climbert.getMotorOutputVoltage() > 500){
			climberv = climberv + 1;
		}

		if(climberb.get()){
			climbert.set(ControlMode.PercentOutput, -.5);
		}else if(climberv == 1){
			climbert.set(ControlMode.PercentOutput, -1);
		}else if(climberv == 2){
			climbert.set(ControlMode.PercentOutput, 0);
			climberv = 0;
		}else{
			climbert.set(ControlMode.PercentOutput, 0);
		}

	}


	void climberTest(){
		if(climberb.get() && climbert.getOutputCurrent() < 2){
			climbert.set(ControlMode.PercentOutput, -.3);
		}	
		else if(climberb.get() && climbert.getOutputCurrent() > 2){
			climbert.set(ControlMode.PercentOutput, -.8);
		}else{
			climbert.set(ControlMode.PercentOutput, 0);
		}
	}
	/**	
	public void gearSystem(){
		if(gearIn.get() && gearThing.get()){
//			gearPiston.set(Value.kForward);
//			Timer.delay(.5);
			gearIntake.set(.8);
		}else if(spitGear.get()){
			gearIntake.set(-.8);
		}else if(!gearThing.get()){
//			gearPiston.set(Value.kReverse);
//			Timer.delay(.5);
			gearIntake.set(0);
		}else{
			gearIntake.set(0);
//			gearPiston.set(Value.kReverse);
		}

	}

	 **/
	void auton1(){
		TankDrive(-.4, -.4);
		Timer.delay(2);
		TankDrive(0, 0);

	}
	
	void auton2(){
		TankDrive(-.5, -.5);
		Timer.delay(.5);
		TankDrive(.5, -.5);
		Timer.delay(.64);
		TankDrive(-.5,-.5);
		Timer.delay(1.5);
		TankDrive(-.5, .5);
		Timer.delay(.62);
		TankDrive(-.5, -.5);
		Timer.delay(3);
		TankDrive(0, 0);
		Timer.delay(.5);
		TankDrive(-.5, -.5);
		Timer.delay(1);
		TankDrive(0,0);
	}

	void auton3(){
		TankDrive(-.25, -.25);
		Timer.delay(5);
		TankDrive(0,0);
		Timer.delay(.25);
		TankDrive(.25, .25);
		Timer.delay(1);
		TankDrive(0,0);
	}
	
	void auton4(){
		TankDrive(-.4, -.4);
		Timer.delay(2);
		TankDrive(0, 0);
		Timer.delay(4);
		TankDrive(.4, .4);
		Timer.delay(1);
		TankDrive(0,0);

	}
	/**
	void auton4(){
		while(enc1.get() < 6500 && enc2.get() < 6500){
			tankDrive(.5,.5);
		}
		tankDrive(0,0);
		Timer.delay(.5);
		gearIntake.set(-.25);
		Timer.delay(.25);
		Drive(-.5, -.5);
		Timer.delay(1);
		gearIntake.set(0);
		tankDrive(0,0);
	}

	void auton5(){
		enc1.reset();
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
		tankDrive(0,0);
		Timer.delay(.5);
		gearIntake.set(-.25);
		Timer.delay(.25);
		tankDrive(.25, .25);
		Timer.delay(1);
		gearIntake.set(0);
		tankDrive(0,0);
	}
	
	**/

	void auton6(){
		
		enc1.reset();

		enc1.setDistancePerPulse(.2617);
		while(enc1.getDistance() < 100){

			System.out.println("Encoder 1: " + enc1.getDistance());
			TankDrive(-.25, .25);
			enc1.getDistance();
		}
		TankDrive(0,0);

	}

	 

	void gear1(){
		TankDrive(-.5, .5);
		Timer.delay(1);
		cameraAlign(400, 500);
		while(!center){
			cameraAlign(400, 500);
			if(turnLeft){
				TankDrive(-.3, .3);
			}else if(turnRight){
				TankDrive(.3, -.3);
			}
		}
		TankDrive(.5, .5);
		Timer.delay(1);
		TankDrive(0, 0);
		Timer.delay(.5);
		TankDrive(-.5, -.5);
		Timer.delay(1);
		TankDrive(0,0);
	}

	void gear2(){
		TankDrive(.5, .5);
		Timer.delay(1);
		cameraAlign(400, 500);
		while(!center){
			cameraAlign(400, 500);
			if(turnLeft){
				TankDrive(-.3, .3);
			}else if(turnRight){
				TankDrive(.3, -.3);
			}
		}
		TankDrive(.5, .5);
		Timer.delay(.5);
		TankDrive(0, 0);
		Timer.delay(.5);
		TankDrive(-.5, -.5);
		Timer.delay(1);
		TankDrive(0,0);
	}

	void autonEnc(){


	}

	void gear3(){
		TankDrive(.5, -.5);
		Timer.delay(1);
		cameraAlign(400, 500);
		while(!center){
			cameraAlign(400, 500);
			if(turnLeft){
				TankDrive(-.3, .3);
			}else if(turnRight){
				TankDrive(.3, -.3);
			}
		}
		TankDrive(.5, .5);
		Timer.delay(0);
		TankDrive(0, 0);
		Timer.delay(.5);
		TankDrive(-.5, -.5);
		Timer.delay(1);
		TankDrive(0,0);
	}

	void cameraArea(){
		//		if(camera.getNumberArray("area") != null){
		//			area = camera.getNumberArray("area");	
	}
	//	}

	void cameraAlign(double minX, double maxX){
		//		if(camera.getNumberArray("centerX") != null){
		//			x = camera.getNumberArray("centerX");

		//		}
		if(x != null && x.length != 0){
			if(!aligned && x[x.length - 1] < minX){
				//				tankDrive(-.2, .2);
				turnRight = true;
				turnLeft = false;
				center = false;
			}else if(!aligned && x[x.length - 1] > maxX){
				//				tankDrive(.2, -.2);
				turnRight = false;
				turnLeft = true;
				center = false;
			}else if(x[x.length - 1] > minX && x[x.length - 1] < maxX){
				turnRight = false;
				turnLeft = false;
				center = true;

			}	
		}
	}


}

