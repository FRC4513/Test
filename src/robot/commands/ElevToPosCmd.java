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
public class ElevToPosCmd extends Command {

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
    public ElevToPosCmd(String position, double speed, double mode, double timeOut) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

    	m_position = position;
        m_speed = speed;
        m_mode = mode;
        m_timeOut = timeOut;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.elevSubSys);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	/*If mode = 0, PID with no return, mode = 1, then manual drive with no return,
    	 *   mode = 2, PID with return,    mode = 3, then manual drive with return*/

        mTgtPos = initCmd();		// Look up target position based on string
    	mCurrPos = Robot.elevSubSys.getElevPos();
    	line = "Elev To Position Cmd Init!   Mode=" + m_mode + 
    			" Target=" + m_position + " at (" + mTgtPos + ")  (CurrPos=" + mCurrPos + ")";
    	System.out.println(line) ;
    	Robot.logger.appendLog(line);
    	
    	if ((m_mode == 0) || (m_mode == 2)) {
    		// Let PID do everything
    		Robot.elevSubSys.driveElevByPID(mTgtPos) ;
    		return;
    	}
    	
    	// This is not a PID command so process target data 
    	mCurrPos = Robot.elevSubSys.getElevPos();
    	state = 0;
    	
    	if (Math.abs(mCurrPos - mTgtPos) <= 0.25) {
    		// were within +- 0.25 inches of target ... no need to move
    		state = 1;	// hold this position were done
    		return;
    	}
    	
    	if 		(mTgtPos == 0)			{ mDir = -1; }		// We were going to bottom
    	else if (mTgtPos >= 40)			{ mDir = +1; }		// Were going to the top
    	else if (mCurrPos <= mTgtPos ) 	{ mDir = +1; } 		// We need to raise to target
    	else 							{ mDir = -1; } 		// We need to lower to target
    	
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
       	mCurrPos = Robot.elevSubSys.getElevPos();
    	if ((m_mode == 0) || (m_mode == 2)) {
    		// This is a PID Command dont need to do anything
    		return;
    	}
    	
    	if (state == 0) {
    		// Non PID Command
    		if (mDir > 0) {
    			if ((mTgtPos >= 40) && (Robot.elevSubSys.isUpperLmtSwNotPressed()) ) {
        				Robot.elevSubSys.elevRaise();
    			} else if (( mCurrPos < mTgtPos ) && ( Robot.elevSubSys.isUpperLmtSwNotPressed() ) ) {
    					Robot.elevSubSys.elevRaise();
    			} else {
    					state = 1;				// We have reached are target
    			}
    		}
    		
    		if (mDir < 0) {
    			if((mTgtPos == 0) && (Robot.elevSubSys.isLowerLmtSwNotPressed() ) ) {
    				Robot.elevSubSys.elevLower();
    			} else if (( mCurrPos > mTgtPos ) && ( Robot.elevSubSys.isLowerLmtSwNotPressed() ) ) {
					Robot.elevSubSys.elevLower();
    			} else {
					state = 1;				// We have reached are target				
    			}
    		}
    	}
    		
		if (state == 1) {
			Robot.elevSubSys.elevHoldMtr();    					
			if (m_mode == 3) {
				state = 9;
			}
		}
    }
 
    	

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
    	if ((state == 9) &&  (m_mode > 1)) {
   			return true; // mode 2 needs to be addressed
   		}
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	Robot.elevSubSys.elevHoldMtr();
    	if (Robot.elevSubSys.isLowerLmtSwNotPressed()) {
			Robot.elevSubSys.elevHoldMtr();
    	} else {
    		Robot.elevSubSys.elevStopMtr();
    	}
    	line = "Elev To Position Cmd at end!" ;
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
    	double tgt = 0;
    	switch ( m_position ) {
    	case "Test" :
			tgt = Robot.ELEVTESTVALUE ;
    		break;
    	case "Scale" :
			tgt = Robot.ELEVSCALEVALUE ;
			break ;
    	case "Switch" :
			tgt = Robot.ELEVSWITCHVALUE ;
			break ;
    	case "Switch2" :
			tgt = Robot.ELEVSWITCHVALUE2 ;
			break ;
    	case "Start" :
			tgt = Robot.ELEVSTARTVALUE ;
			break ;
    	case "Retract" :
			tgt = Robot.ELEVRETRACTVALUE ;
			break ;	
    	case "Climb" :
			tgt = Robot.ELEVCLIMBVALUE ;
			break ;
    	case "Top" :
			tgt = Robot.ELEVTOPVALUE ;			
			break ;
    	case "Bottom" :
			tgt = Robot.ELEVBOTTOMVALUE ;
			break ;
    	default:
    		tgt = 0;
    	}
		line = "Elev (" + m_position + ") Position selected! Pos=(" + tgt + ")";
		System.out.println(line);
		Robot.logger.appendLog(line);
    	return tgt;
    }
}