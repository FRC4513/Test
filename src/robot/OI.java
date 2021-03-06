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

import robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import robot.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public JoystickButton driveByJoystickBtn;
    public JoystickButton intakeRetractOffDBtn;
    public JoystickButton intakeRetractOnDBtn;
    public JoystickButton intakeEjectCmdDBtn;
    public JoystickButton autoRetract2Btn;
    public Joystick driverJoystick;
    public JoystickButton armByJoystickBtn;
    public JoystickButton elevByJoystickBtn;
    public JoystickButton intakeRetractOffBtn;
    public JoystickButton intakeRetractOnBtn;
    public JoystickButton intakeEjectCmdBtn;
    public JoystickButton liftToTopBtn;
    public JoystickButton liftToRetractBtn;
    public JoystickButton liftToScaleBtn;
    public JoystickButton liftToSwitch2CmdBtn;
    public JoystickButton liftToSwitchBtn;
    public JoystickButton autoRetractBtn;
    public Joystick coPilotJoystick;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        coPilotJoystick = new Joystick(1);
        
        autoRetractBtn = new JoystickButton(coPilotJoystick, 6);
        autoRetractBtn.whenPressed(new IntakeAutoRetractCmd());
        liftToSwitchBtn = new JoystickButton(coPilotJoystick, 12);
        liftToSwitchBtn.whenPressed(new LiftToSwitchCmdGrp());
        liftToSwitch2CmdBtn = new JoystickButton(coPilotJoystick, 10);
        liftToSwitch2CmdBtn.whenPressed(new LiftToSwitch2CmdGrp());
        liftToScaleBtn = new JoystickButton(coPilotJoystick, 9);
        liftToScaleBtn.whenPressed(new LiftToScaleCmdGrp());
        liftToRetractBtn = new JoystickButton(coPilotJoystick, 11);
        liftToRetractBtn.whenPressed(new LiftToRetractCmdGrp());
        liftToTopBtn = new JoystickButton(coPilotJoystick, 7);
        liftToTopBtn.whenPressed(new LiftToTopCmdGrp());
        intakeEjectCmdBtn = new JoystickButton(coPilotJoystick, 4);
        intakeEjectCmdBtn.whileHeld(new IntakeEjectCmd());
        intakeRetractOnBtn = new JoystickButton(coPilotJoystick, 3);
        intakeRetractOnBtn.whileHeld(new IntakeRetractCmd());
        intakeRetractOffBtn = new JoystickButton(coPilotJoystick, 5);
        intakeRetractOffBtn.whenPressed(new IntakeStopCmd());
        elevByJoystickBtn = new JoystickButton(coPilotJoystick, 2);
        elevByJoystickBtn.whileHeld(new ElevByJoystickCmd());
        armByJoystickBtn = new JoystickButton(coPilotJoystick, 1);
        armByJoystickBtn.whileHeld(new armByJoystickCmd());
        driverJoystick = new Joystick(0);
        
        autoRetract2Btn = new JoystickButton(driverJoystick, 6);
        autoRetract2Btn.whenPressed(new IntakeAutoRetractCmd());
        intakeEjectCmdDBtn = new JoystickButton(driverJoystick, 4);
        intakeEjectCmdDBtn.whileHeld(new IntakeEjectCmd());
        intakeRetractOnDBtn = new JoystickButton(driverJoystick, 3);
        intakeRetractOnDBtn.whileHeld(new IntakeRetractCmd());
        intakeRetractOffDBtn = new JoystickButton(driverJoystick, 5);
        intakeRetractOffDBtn.whenPressed(new IntakeStopCmd());
        driveByJoystickBtn = new JoystickButton(driverJoystick, 1);
        driveByJoystickBtn.whileHeld(new DriveByJoystickCmd());


        // SmartDashboard Buttons
        SmartDashboard.putData("armPidTestCmdGrp", new armPidTestCmdGrp());
        SmartDashboard.putData("ArmStopCmd", new ArmStopCmd());
        SmartDashboard.putData("DrivePointTurnCmd: default", new DrivePointTurnCmd(0, 0, 0, 0));
        SmartDashboard.putData("ElevBrakeOffCmd", new ElevBrakeOffCmd());
        SmartDashboard.putData("ElevBrakeOnCmd", new ElevBrakeOnCmd());
        SmartDashboard.putData("IntakeEjectCmd", new IntakeEjectCmd());
        SmartDashboard.putData("LiftToBottomCmdGrp", new LiftToBottomCmdGrp());
        SmartDashboard.putData("LoggerWrite", new LoggerWrite());
        SmartDashboard.putData("ResetEncodersCmd", new ResetEncodersCmd());
        SmartDashboard.putData("ResetGyro", new ResetGyro());
        SmartDashboard.putData("TestArcTurnCmdGrp", new TestArcTurnCmdGrp());
        SmartDashboard.putData("TestFwdCmdGrp", new TestFwdCmdGrp());
        SmartDashboard.putData("TestFwdPIDCmdGrp", new TestFwdPIDCmdGrp());
        SmartDashboard.putData("TestPtTurnCmdGrp", new TestPtTurnCmdGrp());
        SmartDashboard.putData("TestPtTurnPIDCmdGrp", new TestPtTurnPIDCmdGrp());
        SmartDashboard.putData("TestAuto1", new TestAuto1());
        SmartDashboard.putData("TestLeds", new TestLeds());
        SmartDashboard.putData("ElevPIDCmd", new ElevPIDCmd());
        SmartDashboard.putData("TestAuto2", new TestAuto2());
        SmartDashboard.putData("DriveClearPosDataCmd", new DriveClearPosDataCmd());
        SmartDashboard.putData("TestFwd2CmdGrp", new TestFwd2CmdGrp());
        SmartDashboard.putData("TestRightSwCmdGrp", new TestRightSwCmdGrp());
        SmartDashboard.putData("TestLeftSwCmdGrp", new TestLeftSwCmdGrp());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getDriverJoystick() {
        return driverJoystick;
    }

    public Joystick getCoPilotJoystick() {
        return coPilotJoystick;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

