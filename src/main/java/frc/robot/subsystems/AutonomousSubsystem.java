package frc.robot.subsystems;

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
  public enum AutoPlans {
    DRIVEOUT,
    REEFCENTER,
    REEFLEFT,
    REEFRIGHT
    ;

    public String getSelectName() {
        return this.toString();
    }

    public int getSelectIx() {
        return this.ordinal();
    }
  }

  /*
   * AutoStep details the steps and step parameters available for auto
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
  public enum AutoStep {
    WAIT1("W", 1.0, ""),
    WAIT2("W", 2.0, ""),
    WAITLOOP("W", 99.9, ""),
    DRIVE_OUT("D", 0.0, "drive out - Auto"),
    DRIVE_REEF_LEFT("D", 0.0, "reef barge left - auto"),
    ELEVATOR_LVL4("L", 0.0, ""),
    DRIVE_TO_REEF_RIGHT("D2R", 0.0, ""),
    HAND_SCORE("HS", 0.0, "")
    ;

    private final String m_stepCmdType;
    private final double m_waitTime;
    private final String m_planName;

    private AutoStep(String stepCmdType, double waitTime, String planName) {
      this.m_stepCmdType = stepCmdType;
      this.m_waitTime = waitTime;
      this.m_planName = planName;
    }

    public String getCmdType() {
      return m_stepCmdType;
    }

    public double getWaitTIme() {
      return m_waitTime;
    }

    public String getplanName() {
      return m_planName;
    }
  }

  /*
   * class to associate the step with switches
   * for the step to be selected,
   * the True switch must be on (true)
   * the False switch must be off (false)
   * 
   * The False switch is there to provide two path options 
   * e.g. step 3 and step 5 are mutually exclusive, to one switch is selecting between them
   * 
   */
  private class PlanStep{
    private AutoStep m_autoStep;
    private int m_switchTrue;
    private int m_switchFalse;

    public PlanStep(AutoStep autoStep) {
      this.m_autoStep = autoStep;
      this.m_switchTrue = 0;
      this.m_switchFalse = 0;
    }

    public PlanStep(AutoStep autoStep, int switchTrue) {
      this.m_autoStep = autoStep;
      this.m_switchTrue = switchTrue;
      this.m_switchFalse = 0;
    }

    public PlanStep(AutoStep autoStep, int switchTrue, int switchFalse) {
      this.m_autoStep = autoStep;
      this.m_switchTrue = switchTrue;
      this.m_switchFalse = switchFalse;
    }

    public AutoStep getAutoStep() {
      return m_autoStep;
    }

    public int getASwitch() {
      return m_switchTrue;
    }

    public int getBSwitch() {
      return m_switchFalse;
    }

  }

  // save for passed in control box and subsystems
  ConsoleAuto m_ConsoleAuto;
  RobotContainer m_robotContainer;
  DriveSubsystem m_drive;
  ElevatorSubsystem m_elevator;
  HandSubsystem m_hand;

  // work for plan selection
  AutoPlans m_autoPlans[] = AutoPlans.values();
  AutoPlans m_selectedPlan;

  private String m_selectedPlanName = "n/a";
  private int m_iWaitCount;

  // work for plan steps for current plan selection
  // used in dashboard display for step selection
  private int kSTEP_MAX = 12;
  private PlanStep[] m_autoStep = new PlanStep[kSTEP_MAX];
  private String[] m_strStepList = new String[kSTEP_MAX];
  private String[] m_strStepSwitch = new String[kSTEP_MAX];
  private boolean[] m_bStepSWList = new boolean[kSTEP_MAX];
  private int m_iCmdCount = 0;

  private int m_iPatternSelect;

  private PlanStep[][] m_planSteps;

  // constructor
  /*
   * pass in the control box and required subsystems
   * 
   */
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
    m_selectedPlan = m_autoPlans[0];
    m_selectedPlanName = m_selectedPlan.toString();
    m_iPatternSelect = 0;

    // set up dashboard display
    for (int iat = 0; iat < kSTEP_MAX; iat++) {
      initStepList(iat);
      fmtDisplay(iat);
    }
  
/*
 *  CRITICAL PIECE
 * This two dimensional array defines the steps for each selectable Auto Plan
 * First dimension is set by the ConsoleAuto selector switch (passed in via POV 0)
 * Second dimension is the sequence of the possible step(s) for the pattern
 * 
 * data elements are from the AutoStep ENUM of steps, run if true switch, and run if false switch
 * the switch numbers are optional, although if the false one is used, then the true is required
 * 
 */
    m_planSteps = new PlanStep[][] {
      //DRIVE OUT
          {new PlanStep(AutoStep.WAITLOOP),
           new PlanStep(AutoStep.DRIVE_OUT, 1)
          },  
      //REEF CENTER
          {new PlanStep(AutoStep.WAITLOOP), 
            new PlanStep(AutoStep.WAIT1)
          },
      //REEF LEFT
          {new PlanStep(AutoStep.WAITLOOP),
             new PlanStep(AutoStep.DRIVE_REEF_LEFT, 1),
             new PlanStep(AutoStep.ELEVATOR_LVL4, 2),
             new PlanStep(AutoStep.DRIVE_TO_REEF_RIGHT, 3),
             new PlanStep(AutoStep.HAND_SCORE, 4)
          },
      //REEF RIGHT
          {new PlanStep(AutoStep.WAITLOOP)
          }
    };

    if (m_autoStep.length < m_planSteps.length ) {
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
    SmartDashboard.putString("Selected Pattern", m_selectedPlanName);
    SmartDashboard.putNumber("WaitLoop", m_iWaitCount);
    for (int iat = 0; iat < kSTEP_MAX; iat++) {
      fmtDisplay(iat);
    }
  }
   
/*
 * method to handle building the plan step selection
 * called by command running while disabled
 * 
 * saves the possible steps for the selected AutoPlan
 * tests true/false switches to set step selection
 * 
 */
  public void selectAutoCommand() {

    int autoSelectIx = m_ConsoleAuto.getROT_SW_0();
    m_iPatternSelect = autoSelectIx;
    if (autoSelectIx >= m_planSteps.length) {
      autoSelectIx = 0;
      m_iPatternSelect = 0;
    }

    m_selectedPlan = m_autoPlans[autoSelectIx];
    m_selectedPlanName = m_selectedPlan.toString();

    m_iWaitCount = m_ConsoleAuto.getROT_SW_1();

    // save the possible step list for the selected pattern to a work list
    m_iCmdCount = 0;
    for (int ix = 0; ix < m_planSteps[autoSelectIx].length; ix++) {
      m_autoStep[ix] = m_planSteps[autoSelectIx][ix];
      m_strStepList[ix] = m_autoStep[ix].getAutoStep().name();
      m_strStepSwitch[ix] = getStepSwitch(m_autoStep[ix]);
      m_bStepSWList[ix] = getStepBoolean(m_autoStep[ix]);
      if (m_bStepSWList[ix]) {
        m_iCmdCount++;
      }
    }
    for (int ix = m_planSteps[autoSelectIx].length; ix < kSTEP_MAX; ix++) {
      initStepList(ix);
    }

  }

  /*
   * return a string value of the true/false switches for the step
   * used to display on the dashboard for reference
   */
  private String getStepSwitch(PlanStep stepName) {
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
    
  /*
   * return true/false for step selection based on switch settings
   */
  private boolean getStepBoolean(PlanStep stepName) {
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
  public Command selectAuto() {
    return Commands.run(this::selectAutoCommand, this)
          .ignoringDisable(true);
  }

  /*
   * Command to process the selected command list
   * 
   * creates a sequential command group of all selected commands for the auto pattern
   * returns the group 
   * 
  */
  public Command runAuto() {

    SequentialCommandGroup autoCmd = new SequentialCommandGroup();
    System.out.println("Cmd Count " + m_iCmdCount);

    for (int ix = 0; ix < m_planSteps[m_iPatternSelect].length; ix++) {
      if (m_bStepSWList[ix]) {
        //System.out.print("Selected command " + ix);
        //System.out.println("-" + m_strStepList[ix]);
        autoCmd.addCommands(Commands.print("Starting: " + m_strStepList[ix]));
        autoCmd.addCommands(autoStepCmd(m_autoStep[ix].getAutoStep()));
        autoCmd.addCommands(Commands.print("Just completed: " + m_strStepList[ix]));
      }
    }

    return autoCmd;
   
  }

  /*
   * Return the Command for each selected step
   * 
   */
  private Command autoStepCmd(AutoStep autoStep) {

    Command workCmd = Commands.print("command not found for " + autoStep.name());
    switch (autoStep.getCmdType()) {
      case "W":
        double waitTime = autoStep.getWaitTIme();
        System.out.println("Wait time " + autoStep.getWaitTIme());
        if (waitTime == 99.9) {
          workCmd = Commands.waitSeconds(m_ConsoleAuto.getROT_SW_1());
        } else {
          workCmd = Commands.waitSeconds(waitTime);
        }

        break;
      case "D":
        workCmd =  m_drive.getPathStep(autoStep.getplanName());
        break;
      case "L":
        // build sequence to raise elevator, move hand up, and wait until elevator is in position
        workCmd = Commands.sequence(m_elevator.cmdSetElevatorPosition(ElevatorConstants.kLevel4Inches, m_hand.isHandDownSplr()),
                                    Commands.waitUntil(() -> m_elevator.isElevatorAtCrossbar()),
                                    m_hand.cmdSetHandUp(),
                                    Commands.waitUntil(m_elevator.isElevatorAtLevel())
                                    );
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
  
}
