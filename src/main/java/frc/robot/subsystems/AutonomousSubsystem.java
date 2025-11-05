package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.libraries.ConsoleAuto;
import frc.robot.subsystems.DriveSubsystem.BranchSide;

// Something interesting I found was DriverStation.getMatchTime() It returns how much time is left, might be useful.

public class AutonomousSubsystem extends SubsystemBase{

  // limited by the rotary switch settings of 6 and POV max of 8.
  public enum AutonomousCommands {
    DRIVEOUT,
    REEFCENTER,
    REEFLEFT,
    REEFRIGHT,
    LIFT;

    public String getSelectName() {
        return this.toString();
    }

    public int getSelectIx() {
        return this.ordinal();
    }
  }

  /*
   * AutonomousSteps details the steps and step parameters available for auto
   * 
   * parameters
   * stepCmd: step structure or what type of command
   * waitTime: wait time in seconds for wait commands
   * Switch True: switch number that will turn on this step if true
   * Switch False: switch number that will turn off this step if true
   * 
   * tbd - auto or path names to coordinate with PathPlanner routines
   * for 2025 the first path must be an Auto that sets the starting pose 
   * after that they should be paths only
   */
  public enum AutonomousSteps {
    WAIT1("W", 1.0, 0, 0, ""),
    WAIT2("W", 2.0, 0, 0, ""),
    WAIT_HAND_SCORE("W", 4.0, 4, 0, ""),
    WAIT_DRIVE_TO_REEF("W", 2.5, 3, 0, ""),
    WAITLOOP("W", 99.9, 0, 0, ""),
    DRIVE_OUT("D", 0.0, 1, 0, "drive out - Auto"),
    DRIVE_REEF_LEFT("D", 0.0, 1, 0, "reef barge left - auto"),
    ELEVATOR_LVL4("L",0.0,2,0,""),
    DRIVE_TO_REEF_RIGHT("D2R" , 0.0 , 3 , 0 , ""),
    HAND_SCORE("HS" , 0.0 , 4 , 0 , "")
    ;

    private final String m_stepCmd;
    private final double m_waitTime;
    private final int m_iSwATrue;
    private final int m_iSwBFalse;
    private final String m_planName;

    private AutonomousSteps(String cstepCmd, double dWaitTime, int iSwATrue, int iSwBFalse, String planName) {
      this.m_stepCmd = cstepCmd;
      this.m_waitTime = dWaitTime;
      this.m_iSwATrue = iSwATrue;
      this.m_iSwBFalse = iSwBFalse;
      this.m_planName = planName;
    }

    public String getStepStruc() {
      return m_stepCmd;
    }

    public double getWaitTIme() {
      return m_waitTime;
    }

    public int getASwitch() {
      return m_iSwATrue;
    }

    public int getBSwitch() {
      return m_iSwBFalse;
    }
    public String getplanName() {
      return m_planName;
    }
  }

  private int kSTEP_MAX = 12;

  ConsoleAuto m_ConsoleAuto;
  RobotContainer m_robotContainer;
  DriveSubsystem m_drive;
  ElevatorSubsystem m_elevator;
  HandSubsystem m_hand;

  AutonomousCommands m_autoSelectCommand[] = AutonomousCommands.values();
  AutonomousCommands m_selectedCommand;

  private String m_strCommand;
  private int m_iWaitCount;
  private AutonomousSteps[] m_autoStep = new AutonomousSteps[kSTEP_MAX];
  private String[] m_strStepList = new String[kSTEP_MAX];
  private String[] m_strStepSwitch = new String[kSTEP_MAX];
  private boolean[] m_bStepSWList = new boolean[kSTEP_MAX];
  private int m_iCmdCount = 0;

  private String m_autoCmd = "n/a";
  private int m_iPatternSelect;

  private AutonomousSteps[][] m_cmdSteps;
  private Command[] m_stepCommands;

  public AutonomousSubsystem(ConsoleAuto consoleAuto,
        RobotContainer robotContainer,
        DriveSubsystem drive,
        ElevatorSubsystem elevator,
        HandSubsystem hand) {

    m_ConsoleAuto = consoleAuto;
    m_robotContainer = robotContainer;
    m_drive = drive;
    m_elevator = elevator;
    m_hand = hand;
    m_selectedCommand = m_autoSelectCommand[0];
    m_strCommand = m_selectedCommand.toString();
    m_iPatternSelect = 0;

    for (int iat = 0; iat < kSTEP_MAX; iat++) {
      initStepList(iat);
      fmtDisplay(iat);
    }
  
    m_stepCommands = new Command[ AutonomousSteps.values().length];
    int cmdIx = 0;
    for (AutonomousSteps stepCommands: AutonomousSteps.values()) {
    //  System.out.println(stepCommands);
      m_stepCommands [cmdIx] = getAutoCmd(stepCommands);
      cmdIx++;
    }


/*
 *  CRITICAL PIECE
 * This two dimensional array defines the steps for each selectable Auto pattern
 * First dimension is set by the ConsoleAuto selector switch (passed in via POV 0)
 * Second dimension is the sequence of the possible step(s) for the pattern
 */
    m_cmdSteps = new AutonomousSteps[][] {
      //DRIVE OUT
          {AutonomousSteps.WAITLOOP,
           AutonomousSteps.DRIVE_OUT},

            
      //REEF CENTER
          {AutonomousSteps.WAITLOOP, 
           // AutonomousSteps.SHOOTNOTE, 
            //AutonomousSteps.SpkrCntrOut1, 
            AutonomousSteps.WAIT1
            //AutonomousSteps.SpkrCntrRtrn1
          },
      //REEF LEFT
          {AutonomousSteps.WAITLOOP,
             AutonomousSteps.DRIVE_REEF_LEFT,
             AutonomousSteps.ELEVATOR_LVL4,
             AutonomousSteps.WAIT_DRIVE_TO_REEF,
             AutonomousSteps.DRIVE_TO_REEF_RIGHT,
             AutonomousSteps.WAIT_HAND_SCORE,
             AutonomousSteps.HAND_SCORE
          },
      //REEF RIGHT
          {AutonomousSteps.WAITLOOP
           // AutonomousSteps.SHOOTNOTE
          },      // lift arm
          {AutonomousSteps.WAITLOOP,
              AutonomousSteps.DRIVE_OUT,
            AutonomousSteps.ELEVATOR_LVL4}
    };

    if (m_autoStep.length < m_cmdSteps.length ) {
      System.out.println("WARNING - Auto Commands LT Command Steps");
    }
    // more? like more commands than supported by the switch
    // printline means a discrepancy in the auto steps selected
  }

  private void fmtDisplay(int ix) {
  
    String labelName = "Step " + ix;

    labelName = "Step " + ix;
    SmartDashboard.putString(labelName, m_strStepList[ix]);

    labelName = "Switch(es) " + ix;
    SmartDashboard.putString(labelName, m_strStepSwitch[ix]);

    labelName = "SwState " + ix;
    SmartDashboard.putBoolean(labelName, m_bStepSWList[ix]);
  
  }

  private void initStepList(int ix) {
      m_strStepList[ix] = "";
      m_strStepSwitch[ix] = "";
      m_bStepSWList[ix] = false;
  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run
    SmartDashboard.putString("Selected Pattern", m_autoCmd);
    SmartDashboard.putNumber("WaitLoop", m_iWaitCount);
    for (int iat = 0; iat < kSTEP_MAX; iat++) {
      fmtDisplay(iat);
    }
  }
   

  public void selectAutoCommand() {

    int autoSelectIx = m_ConsoleAuto.getROT_SW_0();
    m_iPatternSelect = autoSelectIx;
    if (autoSelectIx >= m_cmdSteps.length) {
      autoSelectIx = 0;
      m_iPatternSelect = 0;
    }

    m_selectedCommand = m_autoSelectCommand[autoSelectIx];
    m_strCommand = m_selectedCommand.toString();
    m_autoCmd = m_strCommand;

    m_iWaitCount = m_ConsoleAuto.getROT_SW_1();

    m_iCmdCount = 0;
    for (int ix = 0; ix < m_cmdSteps[autoSelectIx].length; ix++) {
      m_autoStep[ix] = m_cmdSteps[autoSelectIx][ix];
      m_strStepList[ix] = m_autoStep[ix].name();
      m_strStepSwitch[ix] = getStepSwitch(m_autoStep[ix]);
      m_bStepSWList[ix] = getStepBoolean(m_autoStep[ix]);
      if (m_bStepSWList[ix]) {
        m_iCmdCount++;
      }
    }
    for (int ix = m_cmdSteps[autoSelectIx].length; ix < kSTEP_MAX; ix++) {
      initStepList(ix);
    }

  }

  private String getStepSwitch(AutonomousSteps stepName) {
    String stepSwName = "";
    int stepSwitch = stepName.getASwitch();
    if (stepSwitch > 0) {
      stepSwName = String.valueOf(stepSwitch);
    }
    stepSwitch = stepName.getBSwitch();
    if (stepSwitch > 0) {
      stepSwName = stepSwName + " & !" + String.valueOf(stepSwitch);
    }
    return stepSwName;
  }
    
  private boolean getStepBoolean(AutonomousSteps stepName) {
    boolean stepBool = true;
    int stepSwitch = stepName.getASwitch();
    if (stepSwitch > 0) {
      stepBool = m_ConsoleAuto.getButton(stepSwitch);
    }
    stepSwitch = stepName.getBSwitch();
    if (stepSwitch > 0) {
      stepBool = stepBool & !m_ConsoleAuto.getButton(stepSwitch);
    }
    return stepBool;
  }


  /*
   * Command to run the Auto selection process with Operator Console interaction
   * This should be handled by a trigger that is started on Disabled status
   */
  public Command cmdAutoSelect() {
    return Commands.run(this::selectAutoCommand, this)
          .ignoringDisable(true);
  }

  /*
   * Command to process the selected command list
  */
  public Command cmdAutoControl() {

    Command autoCmdList[] = new Command[m_iCmdCount];
    System.out.println("Cmd Count " + m_iCmdCount);

    int cmdIx = 0;
    for (int ix = 0; ix < m_cmdSteps[m_iPatternSelect].length; ix++) {
      if (m_bStepSWList[ix]) {
        System.out.print("Selected command " + ix);
        System.out.println("-" + m_strStepList[ix]);
        autoCmdList[cmdIx] = m_stepCommands[m_autoStep[ix].ordinal()];
        cmdIx++;
      }
    }

    SequentialCommandGroup autoCmd = new SequentialCommandGroup(autoCmdList);
    return autoCmd;
   
  }
  private Command getAutoCmd(AutonomousSteps autoStep) {

    Command workCmd = Commands.print("command not found for " + autoStep.name());
    switch (autoStep.getStepStruc()) {
      case "W":
        double waitTime = autoStep.getWaitTIme();
        if (waitTime == 99.9) {
          workCmd = getWaitLoop(() -> m_ConsoleAuto.getROT_SW_1());
        } else {
          workCmd = getWaitCommand(waitTime);
        }

        break;
      case "D":
        workCmd =  m_drive.getPathStep(autoStep.getplanName());
        break;
      case "L":
        workCmd = m_elevator.cmdSetElevatorPosition(ElevatorConstants.kLevel4Inches, m_hand.isHandDownSplr());
        break;
      case "D2R":
        workCmd = m_drive.driveToBranch(BranchSide.RIGHT);
        break;
      case "HS":
        workCmd = m_hand.cmdHandScore();
        break;
      default:
        break;
    }
    return workCmd;
  }

  private Command getWaitCommand(double seconds) {
    return Commands.waitSeconds(seconds);
  }

  private Command getWaitLoop(DoubleSupplier loopTime) {
    return Commands.waitSeconds(loopTime.getAsDouble());
  }
  
}
