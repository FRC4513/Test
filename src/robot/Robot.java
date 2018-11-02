// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.commands.*;
import robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {
	
	public static PowerDistributionPanel pdp = new PowerDistributionPanel();

    Command autoCmd;
    SendableChooser<Command> chooser = new SendableChooser<Command>();
    SendableChooser<String> locChooser = new SendableChooser<String>();
    SendableChooser<String> actionChooser = new SendableChooser<String>();
    SendableChooser<String> crossChooser = new SendableChooser<String>();
	public static Timer sysTimer = new Timer();
	String fmsGameData;
	Boolean switchLeftLit, scaleLeftLit, switchRightLit, scaleRightLit;
	public static Preferences prefs;
	String line;
	

    public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static Drivetrain drivetrain;
    public static LEDSubSys lEDSubSys;
    public static Logger logger;
    public static I2CsubSys i2CsubSys;
    public static ArmSubSys armSubSys;
    public static ElevSubSys elevSubSys;
    public static IntakeSubSys intakeSubSys;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    // Arm position is in degrees 90 = straight out 180 = top
    public static final double ARMTOPVALUE = 180;
    public static final double ARMBOTTOMVALUE = 0;
    public static final double ARMSCALEVALUE = 180;
    public static final double ARMSWITCHVALUE = 110;
    public static final double ARMSWITCHVALUE2 = 50;    
    public static final double ARMSTARTVALUE = 180;
    public static final double ARMRETRACTVALUE = 0;
    public static final double ARMCLIMBVALUE = 0;
    public static final double ARMTESTVALUE = 45;
    
    // Elev position is in inches 0 = bottom 40=top
    public static final double ELEVTOPVALUE = 40;
    public static final double ELEVBOTTOMVALUE = 0;
    public static final double ELEVSCALEVALUE = 40;
    public static final double ELEVSWITCHVALUE = 0;
    public static final double ELEVSWITCHVALUE2 = 20;
    public static final double ELEVSTARTVALUE = 0;
    public static final double ELEVRETRACTVALUE = 0;
    public static final double ELEVCLIMBVALUE = 15.25;
    public static final double ELEVTESTVALUE = 20;    

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        prefs = Preferences.getInstance();
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        drivetrain = new Drivetrain();
        lEDSubSys = new LEDSubSys();
        logger = new Logger();
        i2CsubSys = new I2CsubSys();
        armSubSys = new ArmSubSys();
        elevSubSys = new ElevSubSys();
        intakeSubSys = new IntakeSubSys();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        chooser.addDefault("AutoDoNothingCmdGrp", new AutoDoNothingCmdGrp());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        

        //gameData="   ";
    	locChooser.addObject("1 Left", "Left");
    	locChooser.addDefault("2 Center", "Ctr");
    	locChooser.addObject("3 Right", "Right");
    	SmartDashboard.putData("Choose-Location",locChooser);
    	
    	actionChooser.addDefault("1 Do Nothing", "Nothing");
    	actionChooser.addObject("2 Line Only", "Line");
    	actionChooser.addObject("3 Place PC (Scale then Switch)", "Scale"); 
    	actionChooser.addObject("4 Place PC (Switch then Scale)", "Switch");
    	actionChooser.addObject("5 Place PC (Switch Only)", "SwitchOnly");    	
    	SmartDashboard.putData("Choose-Action", actionChooser);
    	
    	crossChooser.addDefault("0 NO Do NOT Cross Field (Stop in middle)", "NoCross");
    	crossChooser.addObject("1 YES CROSS Field to Place Cude (if no lights lit)", "Cross");
    	SmartDashboard.putData("Cross-Flag", crossChooser);
    	userInit();
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){

    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        fmsGameData = DriverStation.getInstance().getGameSpecificMessage();
        if ((fmsGameData == null) || (fmsGameData.isEmpty())) fmsGameData = "XXX";
    }

    @Override
    public void autonomousInit() {
    	Robot.elevSubSys.elevBrakeOff();
    	Robot.logger.logTimer.reset();
    	Robot.logger.logTimer.start();    	
        autoCmd = chooser.getSelected();
        // schedule the autonomous command (example)
        if (autoCmd != null) autoCmd.start();
    	sysTimer.reset();			// System timer for Competition run
    	sysTimer.start();   
    	Robot.logger.reSetLogTime();
    	Robot.drivetrain.resetGyro();
    	Robot.drivetrain.resetEncodersAndStats();
    	line = "******************  AutonomousInit started ******************";
    	Robot.logger.appendLog(line);
    	System.out.println(line);
    	line = "  ***** Battery Voltage = " + pdp.getVoltage() + " *****";
    	Robot.logger.appendLog(line);
    	System.out.println(line);    	
    	DriverStation.Alliance color;
    	color = DriverStation.getInstance().getAlliance();
    	if (color == DriverStation.Alliance.Blue){
    		Robot.logger.appendLog("Alliance Color = BLUE");
    	} else {
    		Robot.logger.appendLog("Alliance Color = RED");
    	}
    	//gameData = fmsGameData;				// get current FMS game data
    	
    	String locChoice = locChooser.getSelected();    	    	
    	String actionChoice = actionChooser.getSelected();
    	String crossChoice = crossChooser.getSelected();
        switchLeftLit = false;
        switchRightLit = false;
        scaleLeftLit = false;
        scaleRightLit = false;
        if(fmsGameData.equals("XXX")) {
        	System.out.println("FMS Data Null");
    		Robot.logger.appendLog("FMS Data Null");
        	return;
        }
    	if (fmsGameData.charAt(0) == 'L')
    		switchLeftLit = true;	// Makes it easier to read if statements
    	else
    		switchRightLit = true;
    	if (fmsGameData.charAt(1) == 'L')
    		scaleLeftLit = true;
    	else 
    		scaleRightLit = true;
    	
    	if (actionChoice.equals("Nothing")){
    		System.out.println("Do Nothing Selected");
    		Robot.logger.appendLog("Do Nothing Selected");
    		autoCmd = new AutoDoNothingCmdGrp();
    		autoCmd.start();
    		return;
    	}
    	
    	if (actionChoice.equals("Line")){
    		if (locChoice.equals("Left")) {
        		System.out.println("Line Left Selected");
        		Robot.logger.appendLog("Line Left Selected");
        		autoCmd = new AutoLineOnlyCmdGrp(); 	// run line left command
        		autoCmd.start();
        		return;
    		}
    		if (locChoice.equals("Right")) {
            	System.out.println("Line Right Selected");
            	Robot.logger.appendLog("Line Right Selected");
            	autoCmd = new AutoLineOnlyCmdGrp(); 	// run line Right command
            	autoCmd.start();
            	return;
    		}
    		if (locChoice.equals("Ctr")) {
    			if (switchLeftLit) {
        			// Place Cube onto Left switch platform from front
        			System.out.println("Center Left Line Selected");
        			Robot.logger.appendLog("Center Left Line Selected");
        			autoCmd = new AutoCtrLeftLineCmdGrp();		// Drop Cube in left switch
        			autoCmd.start();
        			return;        		
        		} else {
        			// Place Cube onto Right Switch platform from front
        			System.out.println("Center Right Line Selected");
        			Robot.logger.appendLog("Center Right Line Selected");
        			autoCmd = new AutoCtrRightLineCmdGrp();		// Drop Cube in Right switch
        			autoCmd.start();
        			return;    			
        		}
    		}
    	}
    	
    	// **** All following actions are placing cubes !! ********   	
    	if (locChoice.equals("Ctr")){
    		if (switchLeftLit) {
    			// Place Cube onto Left switch platform from front
    			System.out.println("Center Left Cube Selected");
    			Robot.logger.appendLog("Center Left Cube Selected");
    			autoCmd = new AutoCtrLeftSwCmdGrp();		// Drop Cube in left switch
    			autoCmd.start();
    			return;        		
    		} else {
    			// Place Cube onto Right Switch platform from front
    			System.out.println("Center Right Cube Selected");
    			Robot.logger.appendLog("Center Right Cube Selected");
    			autoCmd = new AutoCtrRightSwCmdGrp();		// Drop Cube in Right switch
    			autoCmd.start();
    			return;    			
    		}
    	}

    	if (locChoice.equals("Left")){
    		if ((actionChoice.equals("SwitchOnly")) && (switchLeftLit)) {
   				// Place Cube onto Left Switch platform from side
   				System.out.println("Left Switch Only Selected");
   				Robot.logger.appendLog("Left Switch Only Selected");
   				autoCmd = new AutoLeftSwCmdGrp();		// Drop Cube in left Switch
   				autoCmd.start();
    			return;
    		}
    		if ((actionChoice.equals("Switch")) && (switchLeftLit)) {
    			// Place Cube onto Right Switch platform from side
				System.out.println("Left Switch Selected");
				Robot.logger.appendLog("Left Switch Selected");
				autoCmd = new AutoLeftSwCmdGrp();		// Drop Cube in left Switch
   				autoCmd.start();
   				return; 			
    		}
    		if ((actionChoice.equals("Scale")) && (scaleLeftLit)) {
    			// Place Cube onto Right Scale platform    			
				System.out.println("Left Scale Selected");
				Robot.logger.appendLog("Left Scale Selected");
				autoCmd = new AutoLeftScaleCmdGrp();		// Drop Cube in left Scale
   				autoCmd.start();
   				return; 			
    		}
    		if ((actionChoice.equals("Switch")) && (scaleLeftLit)) {
    			// Place Cube onto Left Scale platform
				System.out.println("Left Scale Selected");
				Robot.logger.appendLog("Left Scale Selected");
				autoCmd = new AutoLeftScaleCmdGrp();		// Drop Cube in left Switch
   				autoCmd.start();
   				return; 			
    		}
    		if ((actionChoice.equals("Scale")) && (switchLeftLit)) {
    			// Place Cube onto Left Switch platform from side
				System.out.println("Left Switch Selected");
				Robot.logger.appendLog("Left Switch Selected");
				autoCmd = new AutoLeftSwCmdGrp();		// Drop Cube in left Switch
   				autoCmd.start();
   				return; 			
    		}
    		// Left side has no Lit conditions just go forward to cross line or cross field
    		if (crossChoice.equals("Cross")) {
    			// Place Cube onto Right Switch platform from crossing over
				System.out.println("Left Cross to Right Switch Only Selected");
				Robot.logger.appendLog("Left Cross to Right Switch Only Selected");
				autoCmd = new AutoLeftRightSwCmdGrp();		// Drop Cube in left Switch
   				autoCmd.start();
   				return; 			
    		}
			System.out.println("\"Left Position");
			Robot.logger.appendLog("\"Left Position");
			autoCmd = new AutoLeftPosCmdGrp();		// Cross Left Line
			autoCmd.start();
			return;    		
    	} // endif (locChoice == "Left")
    	
    	
    	if (locChoice.equals("Right")){
    		if ((actionChoice.equals("SwitchOnly")) && (switchRightLit)) {
    			// Place Cube onto Left Switch platform from side
				System.out.println("Right Switch Only Selected");
				Robot.logger.appendLog("Right Switch Only Selected");
				autoCmd = new AutoRightSwCmdGrp();		// Drop Cube in left Switch
   				autoCmd.start();
   				return; 			
    		}
    		if ((actionChoice.equals("Switch")) && (switchRightLit)) {
    			// Place Cube onto Right Switch platform from side
				System.out.println("Right Switch Selected");
				Robot.logger.appendLog("Right Switch Selected");
				autoCmd = new AutoRightSwCmdGrp();		// Drop Cube in Right Switch
   				autoCmd.start();
   				return; 			
    		}
    		if ((actionChoice.equals("Scale")) && (scaleRightLit)) {
    			// Place Cube onto Right Scale platform    			
				System.out.println("Right Scale Selected");
				Robot.logger.appendLog("Right Scale Selected");
				autoCmd = new AutoRightScaleCmdGrp();		// Drop Cube in Right Scale
   				autoCmd.start();
   				return; 			
    		}
    		if ((actionChoice.equals("Switch")) && (scaleRightLit)) {
    			// Place Cube onto Right Scale platform
				System.out.println("Right Scale Selected");
				Robot.logger.appendLog("Right Scale Selected");
				autoCmd = new AutoRightScaleCmdGrp();		// Drop Cube in Right Switch
   				autoCmd.start();
   				return; 			
    		}
    		if ((actionChoice.equals("Scale")) && (switchRightLit)) {
    			// Place Cube onto Right Switch platform from side
				System.out.println("Right Switch Selected");
				Robot.logger.appendLog("Right Switch Selected");
				autoCmd = new AutoRightSwCmdGrp();		// Drop Cube in Right Switch
   				autoCmd.start();
   				return; 			
    		}

    		// Right side has no Lit conditions just go forward to cross line or cross field
    		if (crossChoice.equals("Cross")) {
    			// Place Cube onto Left Switch platform from crossing over
				System.out.println("Right Cross to Left Switch Only Selected");
				Robot.logger.appendLog("Right Cross to Left Switch Only Selected");
				autoCmd = new AutoRightLeftSwCmdGrp();		// Drop Cube in left Switch
   				autoCmd.start();
   				return; 			
    		}    		
			System.out.println("Right Position");
			Robot.logger.appendLog("Right Position");
			autoCmd = new AutoRightPosCmdGrp();		// Cross Right Line
			autoCmd.start();
			return;    		
    	} // endif (locChoice == "Right")
 
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
    	Robot.elevSubSys.elevBrakeOff();
    	if (autoCmd != null) autoCmd.cancel();
    	Robot.logger.appendLog("******************  TeleopInit started ******************");
    	line = "  ***** Battery Voltage = " + pdp.getVoltage() + " *****";
    	Robot.logger.appendLog(line);
    	System.out.println(line); 
    	Robot.logger.printLog();		// write out logfile from autonomous run
    	} 
    
 

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        
        //show/log battery voltage every 15 seconds
        if ((sysTimer.get() % 15) <= 1){
        	line = "  ***** Battery Voltage = " + pdp.getVoltage() + " *****";
        	Robot.logger.appendLog(line);
        	System.out.println(line);        	
        }
    }
    
    public void userInit() {
    	Robot.drivetrain.resetGyro();
    	Robot.drivetrain.resetEncodersAndStats();
    	sysTimer.reset();			// System timer for Competition run
    	sysTimer.start();  
    }
}
