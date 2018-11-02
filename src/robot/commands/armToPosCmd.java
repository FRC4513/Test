// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/**
 *
 */
public class armToPosCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private String m_position;
    private double m_speed;
    private double m_mode;
    private double m_timeOut;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    
    String line;
    double mCurrPos = 0;
    double mTgtPos;
    int mDir;
    int state;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public armToPosCmd(String position, double speed, double mode, double timeOut) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        m_position = position;
        m_speed = speed;
        m_mode = mode;
        m_timeOut = timeOut;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.armSubSys);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }
    

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	/*If mode = 0, PID with no return, mode = 1, then manual drive with no return,
    	 *   mode = 2, PID with return,    mode = 3, then manual drive with return*/
        mTgtPos = initCmd();		// Look ud target position based on string 
    	mCurrPos = Robot.armSubSys.getArmPos();
    	line = "Arm To Position Cmd Init!   Mode=" + m_mode + 
    			" Target=" + m_position + " at ( Tgt=" + mTgtPos + ")  (CurrPos=" + mCurrPos + ")";
    	System.out.println(line) ;
    	Robot.logger.appendLog(line);
    	if ((m_mode == 0) && (m_mode == 2)) {
    		// Let PID do everything
    		Robot.armSubSys.driveArmByPID(mTgtPos) ;
    		return;
    	}
    	// This is not a PID command so prpocess target data 
    	state = 0;
    	if (Math.abs(mCurrPos - mTgtPos) <= 2) {
    		// were within +- 2 degrees of target ... no need to move
    		state = 1;	// hold this position were done
    		return;
    	}
    	if (mCurrPos <= mTgtPos ) {
    		// We need to raise to target
    		mDir = +1;
    	} else {
    		// We need to lower to target
    		mDir = -1;
    	}
    	line = "Arm To Position Dir = " + mDir;
    	System.out.println(line) ;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	mCurrPos = Robot.armSubSys.getArmPos();
    	if ((m_mode == 0) && (m_mode == 3)){
    		// This is a PID Command dont need to do anything
    		return;
    	}
    	
    	if (state == 0) {
    		// Have we reached our target yet ??
    		if (mDir > 0) {
    			if ((mCurrPos <= mTgtPos) && (Robot.armSubSys.isUpperLmtSwNotPressed())) {
    				Robot.armSubSys.armRaise();
    			} else {
    				state = 1;	// we are where we need to be
    		    	line = "Arm To Position Cmd Raise state=" + state + 
    		    			" (CurrPos=" + mCurrPos + ")";
    		    	System.out.println(line) ;
    		    	Robot.logger.appendLog(line);
    			}
    		} else {
    			if ((mCurrPos >= mTgtPos) && (Robot.armSubSys.isLowerLmtSwNotPressed())) {
    				Robot.armSubSys.armLower();
    			} else {
    				state = 1;	// we are where we need to be
    		    	line = "Arm To Position Cmd Lower state=" + state + 
    		    			" (CurrPos=" + mCurrPos + ")";
    		    	System.out.println(line) ;
    		    	Robot.logger.appendLog(line);
   				}    			
    		}
    	}
    	
    	if (state == 1) {
    		// hold position
    		Robot.armSubSys.armHoldMtr();
    		if (m_mode == 3) {
				state = 9;    
		    	line = "Arm To Position Cmd state=" + state;
		    	System.out.println(line) ;
		    	Robot.logger.appendLog(line);
    		}
    	}    	
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
    	if(( state == 9) && (m_mode > 1)) {
    			return true; // mode 2 needs to be addressed
    		}    	
    	return false ;			// Keep moving or holding
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	if (Robot.armSubSys.isLowerLmtSwNotPressed()) {
    			Robot.armSubSys.armHoldMtr();
    	} else {
			Robot.armSubSys.armStopMtr();
    	}
    	line = "Arm To Position Cmd at end!" ;
    	System.out.println(line) ;
    	Robot.logger.appendLog(line);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	end();
    }
    
    public double initCmd(){
    	double tgt;
    	switch ( m_position ) {
    	case "Test" :
			tgt = Robot.ARMTESTVALUE ;
    		break;
    	case "Scale" :
			tgt = Robot.ARMSCALEVALUE ;
			break ;
    	case "Switch" :
			tgt = Robot.ARMSWITCHVALUE ;
			break ;
    	case "Switch2" :
			tgt = Robot.ARMSWITCHVALUE2 ;
			break ;
    	case "Start" :
			tgt = Robot.ARMSTARTVALUE ;
			break ;
    	case "Retract" :
			tgt = Robot.ARMRETRACTVALUE ;
			break ;	
    	case "Climb" :
			tgt = Robot.ARMCLIMBVALUE ;
			break ;
    	case "Top" :
			tgt = Robot.ARMTOPVALUE ;			
			break ;
    	case "Bottom" :
			tgt = Robot.ARMBOTTOMVALUE ;
			break ;
    	default:
    		tgt = 0;
    	}
		line = "Arm (" + m_position + ") Position selected! Pos=(" + tgt + ")";
		System.out.println(line);
		Robot.logger.appendLog(line);
    	return tgt;
    }
}

