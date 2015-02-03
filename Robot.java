/**************** Operator Interface*******************//*

Right Joystick
	-Trigger = Lateral Drive
	-Button 2 = Shifters
	-Button 3 = Range Finder (Human Player)
	-Button 4 = Camera Tilt
	-Button 5 = Joystick Sensitivity
	-Z Axis = Flip Forward/Reverse & Left/Right

Left Joystick
	-Trigger = Pick up to Level 1

*/
package org.usfirst.frc.team175.robot;

import edu.wpi.first.wpilibj.AnalogInput; import edu.wpi.first.wpilibj.AnalogPotentiometer; import edu.wpi.first.wpilibj.CANJaguar; import edu.wpi.first.wpilibj.CameraServer; import edu.wpi.first.wpilibj.Compressor; import edu.wpi.first.wpilibj.DigitalInput; import edu.wpi.first.wpilibj.DoubleSolenoid; import edu.wpi.first.wpilibj.DriverStation; import edu.wpi.first.wpilibj.Encoder; import edu.wpi.first.wpilibj.Gyro; import edu.wpi.first.wpilibj.IterativeRobot; import edu.wpi.first.wpilibj.Jaguar; import edu.wpi.first.wpilibj.Joystick; import edu.wpi.first.wpilibj.PIDController.AbsoluteTolerance; import edu.wpi.first.wpilibj.Relay; import edu.wpi.first.wpilibj.RobotDrive; import edu.wpi.first.wpilibj.Servo; import edu.wpi.first.wpilibj.Talon; import edu.wpi.first.wpilibj.Timer; import edu.wpi.first.wpilibj.Ultrasonic; import edu.wpi.first.wpilibj.Victor; import edu.wpi.first.wpilibj.image.NIVisionException; import edu.wpi.first.wpilibj.livewindow.LiveWindow; import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; import edu.wpi.first.wpilibj.vision.AxisCamera; //import edu.wpi.first.wpilibj.Label;

/**

The VM is configured to automatically run this class, and to call the
functions corresponding to each mode, as described in the IterativeRobot
documentation. If you change the name of this class or the package after
creating this project, you must also update the manifest file in the resource
directory. */

public class Robot extends IterativeRobot { 
	RobotDrive myRobot;
	Joystick stick; 
	Joystick stick2; 
	DriverStation ds;

	Jaguar motor; 
	Jaguar drive; 
	Talon lateral; 
	Jaguar lateral2; 
	CANJaguar canJag; 
	Talon victor; 
	Servo camServo;

	DigitalInput toteSensor1; 
	DigitalInput toteSensor2; 
	DigitalInput earlavatorStop; 
	DigitalInput irSensor; 
	DigitalInput irSensor2;
	DigitalInput irSensor3;

	AnalogPotentiometer toteTracker; 
	AnalogInput ultra; 
	double range;

	Relay light; 
	Relay light2;

	CameraServer server;

	Compressor Compressor; 
	DoubleSolenoid shifter; 
	DoubleSolenoid lateralDrop; 
	DoubleSolenoid grabber1; 
	DoubleSolenoid grabber2;
	DoubleSolenoid brake;
	
	Gyro gyro;

	Encoder rEncoder; 
	Encoder lEncoder;

	public static boolean lButtonState_1 = false; 
	public static boolean lButtonState_2 = false;

	double Kp = 0.15; //gyro sensitivity

	public static double motorSpeed; 
	public static double victorSpeed; 
	public static double autoStartTime; 
	public static double stopTime; 
	public static double autoSelect; 
	public static double speedrpm = 10;

	public static boolean rButton1State = false; 
	public static boolean rButton2State = false; 
	public static boolean rButton3State = false; 
	public static boolean rButton4State = false; 
	public static boolean rButton5State = false;

	public static boolean lButton1State = false; 
	public static boolean lButton2State = false; 
	public static boolean lButton3State = false; 
	public static boolean lButton4State = false; 
	public static boolean lButton5State = false; 
	public static boolean lButton6State = false;

	public static boolean prevShifterState = false; 
	public static boolean prevServoState = false; 
	public static boolean prevPrintState = false;

	public static int autoMode; 
	public static int innerAutoMode;
	
/**
This function is run when the robot is first started up and should be
used for any initialization code. */ 
	
public void robotInit(){
/***************************** PWM ************************************************/
		myRobot = new RobotDrive(1,0); 
		stick = new Joystick(0); 
		stick2 = new Joystick(1); 
		motor = new Jaguar(5); 
		lateral = new Talon(3); 
		lateral2 = new Jaguar (2); 
		camServo = new Servo(9);

/**************************** PCM *************************************************/
		shifter = new DoubleSolenoid(6, 7); 
		lateralDrop = new DoubleSolenoid(0, 1); 
		grabber1 = new DoubleSolenoid(2, 3); 
		grabber2 = new DoubleSolenoid(4, 5); 
		Compressor = new Compressor(0); 
		Compressor.start();
		
		brake = new DoubleSolenoid(5, 0, 1);						//5 = PCM CAN ID, 0,1 = PCM Ports

/**************************** Analog In ******************************************/
		gyro = new Gyro(0); 
		toteTracker = new AnalogPotentiometer(1); 
		ultra = new AnalogInput(3);

/**************************** CAN ************************************************/ 
		canJag = new CANJaguar(6); 
		canJag.setSpeedMode(CANJaguar.kQuadEncoder, 128, 1, 0, 0);
		canJag.enableControl();

/**************************** Digital Input **************************************/ 
		toteSensor1 = new DigitalInput(0);
		toteSensor2 = new DigitalInput(1);
		irSensor = new DigitalInput(2); 
		irSensor2 = new DigitalInput(3);
		earlavatorStop = new DigitalInput(4);
		irSensor3 = new DigitalInput(5);

/**************************** Relay **********************************************/
		light = new Relay(1); 
		light2 = new Relay(3);

/**************************** Camera **********************************************/
		server = CameraServer.getInstance(); 
		server.setQuality(100); 
		server.startAutomaticCapture("cam0");

/**************************** Encoders ********************************************/
		rEncoder = new Encoder(8, 9, false, Encoder.EncodingType.k4X); 
		lEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);

/**************************** Driver Station **************************************/
		ds = DriverStation.getInstance();
     }
/**
 * This function is run once each time the robot enters autonomous mode
 */
public void autonomousInit() {

    gyro.reset();
    rEncoder.reset();
    lEncoder.reset();
    autoSelect = SmartDashboard.getNumber("DB/Slider 0", 0.0);
    autoStartTime = Timer.getFPGATimestamp();

    if(autoSelect == 0){                                        // Do Nothing
        autoMode = 0;
    }

    if(autoSelect > 0 && autoSelect < 1){                       // Drive Forward
        autoMode = 1;
    }
    if(autoSelect > 1 && autoSelect < 2){
        autoMode = 2;
    }
}

/**
 * This function is called periodically during autonomous
 */
public void autonomousPeriodic() {

    switch(autoMode){
        case 0:
            autoDoNothing();
        break;

        case 1:										// Drive Forward
            switch(innerAutoMode){
                case 0:
                    innerAutoMode++;
                    break;
                case 1:
                    driveForward();
                    if(rEncoder.getDistance() > 100 && lEncoder.getDistance() > 100){
                        innerAutoMode++;
                    }
                    break;
                case 2:
                    autoDoNothing();
                    break;

            }
            break;

        case 2:										// 3 Tote Stack + Deposit   ****NEW
            switch(innerAutoMode){
                case 0:
                    innerAutoMode++;
                    break;
                case 1:
                    driveForward();
                    if(rEncoder.getDistance() > 100 && lEncoder.getDistance() > 100 ){
                        rEncoder.reset();
                        lEncoder.reset();
                        innerAutoMode++;
                    }
                    break;
                case 2:
                    if(toteSensor1.get() == false && toteSensor2.get() == false){
                        grabber1.set(DoubleSolenoid.Value.kForward);
                        grabber2.set(DoubleSolenoid.Value.kForward);
                            if(irSensor.get() == false){
                                canJag.set(50);
                            }else if(irSensor.get() == true){
                            	stopTime = Timer.getFPGATimestamp();
                                canJag.set(0);
                                if(Timer.getFPGATimestamp() - stopTime > 1){
                                	brake.set(DoubleSolenoid.Value.kReverse);
                                }
                                innerAutoMode++;
                        }
                    }

                    break;
                case 3:
                    driveForward();
                    if(rEncoder.getDistance() > 200 && lEncoder.get() > 200){
                        rEncoder.reset();
                        lEncoder.reset();
                        innerAutoMode++;
                    }
                    break;
                case 4:
                	if(toteSensor1.get() == false && toteSensor2.get() == false){
                		brake.set(DoubleSolenoid.Value.kForward);
                		grabber1.set(DoubleSolenoid.Value.kReverse);
                		grabber2.set(DoubleSolenoid.Value.kReverse);
                		if(earlavatorStop.get() == true){
                			canJag.set(-50);
                		}else{
                			canJag.set(0);
                			innerAutoMode++;
                		}
                	}
                    break;
                case 5:
                	if(toteSensor1.get() == false && toteSensor2.get() == false){
                        grabber1.set(DoubleSolenoid.Value.kForward);
                        grabber2.set(DoubleSolenoid.Value.kForward);
                            if(irSensor2.get() == false){
                                canJag.set(50);
                            }else if(irSensor2.get() == true){
                            	stopTime = Timer.getFPGATimestamp();
                                canJag.set(0);
                                if(Timer.getFPGATimestamp() - stopTime > 1){
                                	brake.set(DoubleSolenoid.Value.kReverse);
                                }
                                innerAutoMode++;
                        }
                    }

                    break;
                case 6:
                	 driveForward();
                     if(rEncoder.getDistance() > 200 && lEncoder.get() > 200){
                         rEncoder.reset();
                         lEncoder.reset();
                         innerAutoMode++;
                     }
                     break;
                case 7:
                	if(toteSensor1.get() == false && toteSensor2.get() == false){
                		brake.set(DoubleSolenoid.Value.kForward);
                		grabber1.set(DoubleSolenoid.Value.kReverse);
                		grabber2.set(DoubleSolenoid.Value.kReverse);
                		if(earlavatorStop.get() == true){
                			canJag.set(-50);
                		}else{
                			canJag.set(0);
                			innerAutoMode++;
                		}
                	}
                    break;
                case 8:
                	if(toteSensor1.get() == false && toteSensor2.get() == false){
                        grabber1.set(DoubleSolenoid.Value.kForward);
                        grabber2.set(DoubleSolenoid.Value.kForward);
                            if(irSensor3.get() == false){
                                canJag.set(50);
                            }else if(irSensor3.get() == true){
                            	stopTime = Timer.getFPGATimestamp();
                                canJag.set(0);
                                if(Timer.getFPGATimestamp() - stopTime > 1){
                                	brake.set(DoubleSolenoid.Value.kReverse);
                                }
                                innerAutoMode++;
                        }
                    }

                    break;
                case 9:
                	turn90();
                	if(rEncoder.getDistance() > 100 && lEncoder.getDistance() < 10){
                		rEncoder.reset();
                		lEncoder.reset();
                		driveForward();
                		if(rEncoder.getDistance() > 100 && lEncoder.getDistance() > 100){
                			innerAutoMode++;
                		}
                	}
                	break;
                case 10:
                	backUp();
                	autoDoNothing();
                	break;
            }
            break;
    }

}

/**
 * This function is called once each time the robot enters tele-operated mode
 */
public void teleopInit(){
    gyro.reset();
    rEncoder.reset();
    lEncoder.reset();
    shifter.set(DoubleSolenoid.Value.kReverse);
    lateralDrop.set(DoubleSolenoid.Value.kForward);
}

/**
 * This function is called periodically during operator control
 */

public void teleopPeriodic() {

    if(stick.getRawButton(1)){                                          // Lateral Drive
        myRobot.arcadeDrive(0,0);
        Timer.delay(0.1);
        lateralDrop.set(DoubleSolenoid.Value.kReverse);
        lateral.set(stick.getX());
    }else{
        lateral.set(0);
        lateralDrop.set(DoubleSolenoid.Value.kForward);
        myRobot.arcadeDrive(stick, false);                              //Drive the robot from stick 1 during normal tele-op
    }
    
    if(stick.getZ() > 1){												//Flip which way is Forward and Reverse, Flip Left & Right
    	myRobot.arcadeDrive(-stick.getX(), -stick.getY());
    }
    
    if(stick.getRawButton(5) && !rButton5State){						//Increase Sensitivity. Full Forward  = 0.5 Throttle
    	myRobot.arcadeDrive(0.5*stick.getX(), 0.5*stick.getY());
    }

    if(stick.getRawButton(2) && !rButton2State){                        // Shifters
        if(prevShifterState){
        myRobot.arcadeDrive(0,0);
        shifter.set(DoubleSolenoid.Value.kForward);
    }else{
        shifter.set(DoubleSolenoid.Value.kReverse);
    }
        prevShifterState =! prevShifterState;
    }

    if(stick.getRawButton(3)){						//Ultrasonic Range Finder for Human Player
        rButton3State = true;
    }
    if(rButton3State){
        if(range > 30){                             //Change for larger limit
            myRobot.drive(-0.5, 0);
        }else if(range < 20){                       //Change for smaller limit
            myRobot.drive(0.5, 0);
        }else{
            rButton3State = false;
        }
    }
        range = ultra.getVoltage()/.009766;

    if(stick.getRawButton(4) && !rButton4State){                        // Tilting Camera
        if(prevServoState){
        camServo.set(0.2);  
    }else{
        camServo.set(0);
    }
        prevServoState = !prevServoState;
    }

   if(stick2.getRawButton(1) && !lButton1State){						//IR Sensor Level 1
       lButton1State = true;
   }

   if(lButton1State){
	   if(irSensor.get() == false){ 
		   canJag.set(50); 
	   }else if(irSensor.get() == true){ 
		   stopTime = Timer.getFPGATimestamp();
		   canJag.set(0);
		   if(Timer.getFPGATimestamp() - stopTime > 1){
			   brake.set(DoubleSolenoid.Value.kReverse);
		   }
		   lButton1State = false;
	   }
   }

   if(toteSensor1.get() == false && toteSensor2.get() == false){                    //Check to See if Tote is indexed square
        light.set(Relay.Value.kForward);                        
        grabber1.set(DoubleSolenoid.Value.kReverse);
        grabber2.set(DoubleSolenoid.Value.kReverse);
   }else{
        light.set(Relay.Value.kReverse);
        grabber1.set(DoubleSolenoid.Value.kForward);
        grabber2.set(DoubleSolenoid.Value.kForward);
   }


    rButton1State = stick.getRawButton(1); 
    rButton2State = stick.getRawButton(2);
    rButton4State = stick.getRawButton(4);
    rButton5State = stick.getRawButton(5);

}

public void autoDoNothing(){
    myRobot.drive(0, 0);
    System.out.println("AutoMode 1");
}

public void driveForward(){
    myRobot.drive(1, 0);
    System.out.println("AutoMode 2");
}

public void pickUp(){
    if(toteSensor1.get() == false && toteSensor2.get() == false){
        grabber1.set(DoubleSolenoid.Value.kForward);
        grabber2.set(DoubleSolenoid.Value.kForward);
            if(irSensor.get() == true){
                canJag.set(50);
            }else{
                canJag.set(0);
        }
    }
}

public void turn90(){
	myRobot.drive(0, 0.8);
}

public void backUp(){
	myRobot.drive(-1, 0);
}

/**
 * This function is called periodically during test mode
 */
public void testPeriodic() {
    LiveWindow.run();
}
}