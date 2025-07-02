package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.libraries.ConsoleAuto;

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
    WAIT1('W', 1.0, 0, 0, ""),
    WAIT2('W', 2.0, 0, 0, ""),
    WAITLOOP('W', 99.9, 0, 0, ""),
    DRIVE_OUT('D', 0.0, 1, 0, "drive out - Auto"),
    DRIVE_REEF_LEFT('D', 0.0, 1, 0, "reef barge left - auto"),
    LIFT_ARM('L',0.0,2,0,"lift elevator")
    //SHOOTNOTE('S', 0.0, 1),
   // SpkrCntrOut1('D', 0.0, 2),
   // SpkrCntrRtrn1('D', 0.0, 3),
   // DRV_INTK_1('I', 0.5, 2),
   // DRV_STRT_1('D', 0.0, 3, 2),
   // DRV_BACK_1('D', 0.0, 4),
    //DRV_INTK_2('I', 0.0, 5),
   // DRV_STRT_2('D', 0.0, 6, 5),
    //END('E')
    ;

    private final char m_stepCmd;
    private final double m_waitTime;
    private final int m_iSwATrue;
    private final int m_iSwBFalse;
    private final String m_planName;

    private AutonomousSteps(char cstepCmd, double dWaitTime, int iSwATrue, int iSwBFalse, String planName) {
      this.m_stepCmd = cstepCmd;
      this.m_waitTime = dWaitTime;
      this.m_iSwATrue = iSwATrue;
      this.m_iSwBFalse = iSwBFalse;
      this.m_planName = planName;
    }

  /*   private AutonomousSteps(char cstepCmd, double dWaitTime, int iSwATrue) {
      this.m_stepCmd = cstepCmd;
      this.m_waitTime = dWaitTime;
      this.m_iSwATrue = iSwATrue;
      this.m_iSwBFalse = 0;
    }

    private AutonomousSteps(char cstepCmd, double dWaitTime) {
      this.m_stepCmd = cstepCmd;
      this.m_waitTime = dWaitTime;
      this.m_iSwATrue = 0;
      this.m_iSwBFalse = 0;
    }
    
    private AutonomousSteps(char cstepCmd) {
      this.m_stepCmd = cstepCmd;
      this.m_waitTime = 0;
      this.m_iSwATrue = 0;
      this.m_iSwBFalse = 0;
    }*/

    public char getStepStruc() {
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

  public AutonomousSubsystem(ConsoleAuto consoleAuto, RobotContainer robotContainer) {

    m_ConsoleAuto = consoleAuto;
    m_robotContainer = robotContainer;
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
          {AutonomousSteps.WAITLOOP
            // AutonomousSteps.DRIVE_REEF_LEFT
           // AutonomousSteps.SHOOTNOTE
          },
      //REEF RIGHT
          {AutonomousSteps.WAITLOOP
           // AutonomousSteps.SHOOTNOTE
          },      // lift arm
          {AutonomousSteps.WAITLOOP,
              AutonomousSteps.DRIVE_OUT,
            AutonomousSteps.LIFT_ARM}
    };

    if (m_autoStep.length < m_cmdSteps.length ) {
      System.out.println("WARNING - Auto Commands LT Command Steps");
    }
    // more? like more commands than supported by the switch

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

    int cmdIx = 0;
    for (int ix = 0; ix < m_cmdSteps[m_iPatternSelect].length; ix++) {
      if (m_bStepSWList[ix]) {
        //autoCmdList[cmdIx] = getAutoCmd(m_autoStep[ix]);
     //   System.out.println("Ordinal" + m_autoStep[ix].ordinal());
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
      case 'W':
        double waitTime = autoStep.getWaitTIme();
       // workCmd = getWaitCommand(waitTime == 99.9 ? m_ConsoleAuto.getROT_SW_1() : waitTime);
       if (waitTime == 99.9) {
        workCmd = getWaitLoop(() -> m_ConsoleAuto.getROT_SW_1());
       } else {
        workCmd = getWaitCommand(waitTime);
       }

        break;
      case 'D':
        workCmd =  m_robotContainer.getDrivePlanCmd(autoStep.getplanName());
       // workCmd = Commands.print("Drive command");
        break;
      case 'L':
          workCmd = m_robotContainer.getLiftCmd();
      break;
      case 'I':
       // workCmd =  m_robotContainer.getIntakePathCommand(autoStep.toString(), autoStep.getWaitTIme());
       workCmd = Commands.print("Intake command");
       break;
     // case 'S':
        //workCmd = getWaitCommand(2);
       // switch (autoStep) {
         // case SHOOTNOTE:
           // workCmd = m_robotContainer.cmdShootNote().withTimeout(2);
           // break;
        
        //  default:
          //  break;
       // }
      //  break;
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
