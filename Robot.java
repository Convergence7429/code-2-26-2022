package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {

  static Shooter shooter = new Shooter();
  static DriveTrain drive = new DriveTrain();
  static Intake intake = new Intake();
  static Climber climber = new Climber();
  static PowerDistribution PDP = new PowerDistribution(6, ModuleType.kRev);
  int autoStage = 0;
  // CANSparkMax canSparkMax = new CANSparkMax(15, MotorType.kBrushless);
  Timer timer = new Timer();
  static boolean isRed;

  ///////////////////////////////////////////////////////
  // Robot

  @Override
  public void robotInit() {
    clearStickyFaults();
    drive.driveTrainInit();
    shooter.shooterInit();
    climber.climberInit();
    CameraServer.startAutomaticCapture();
    // CameraServer.startAutomaticCapture(new VideoSource())
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(3.0);
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3.0);
    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      isRed = true;
    } else {
      isRed = false;
    }

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(3.0);
  }

  @Override
  public void robotPeriodic() {

  }

  ////////////////////////////////////////////////////////
  // Autonomous

  @Override
  public void autonomousInit() {
    clearStickyFaults();
    intake.ballCount = 1;
    drive.isDriverControlEnabled = true;
    timer.reset();
    timer.start();
    autoStage = 0;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(3.0);
    shooter.hoodMotor.getEncoder().setPosition(10.5);
    shooter.shooting = false;
  }

  @Override
  public void autonomousPeriodic() {
    // System.out.println(shooter.hoodMotor.getEncoder().getPosition());
    // System.out.println(shooter.calculatedVelocity);
    // System.out.println(Math.toDegrees(Math.toDegrees(shooter.calculatedAngle)));
    
    shooter.testingShooter();

    switch (autoStage) {
      case 0: {
        drive.resetDriveTrainEncoders();
        autoStage = 1;
        break;
      }
      case 1: {
        drive.driveTrainByInches(8.0, 5);
        if (timer.get() > 5.0) {
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          intake.intakeMotor.set(-0.6);
          autoStage = 2;
        }
        break;
      }

      case 2: {
        drive.driveTrainByInches(115.0, 0);
        if (timer.get() > 12.0) {
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          intake.intakeMotor.set(0.0);
          shooter.shootInit();
          autoStage = 3;
        }
        break;
      }

      case 3 : {
        shooter.shooting = true;
        if(timer.get() > 20.0){
          shooter.shooting = false;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          autoStage = 4;
        }
        break;
      }

      case 4 : {
        drive.driveTrainByInches(7.5, 5);
        if (timer.get() > 25.0) {
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          intake.intakeMotor.set(-0.6);
          autoStage = 5;
        }
        break;
      }

      case 5 : {
        drive.driveTrainByInches(65.0, 0);
        if(timer.get() > 35.0){
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          intake.intakeMotor.set(0.0);
          autoStage = 6;
        }
        break;
      }

      case 6 : {
        drive.driveTrainByInches(65.0, 1);
        if(timer.get() > 40.0){
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          shooter.shootInit();
          autoStage = 7;
        }
      }

      case 7 : {
        shooter.shooting = true;
        if(timer.get() > 47.0){
          shooter.shooting = false;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          autoStage = 8;
        }
        break;
      }

      // case 5 : {
      //   drive.driveTrainByInches(50.0, 0);
      //   if (timer.get() > 35.0) {
      //     drive.stopMotors();
      //     drive.resetDriveTrainEncoders();
      //     autoStage = 5;
      //   }
      //   break;
      // }
    }
  }

  ///////////////////////////////////////////////////////////
  // Tele-operated

  double shooterSpeed = 0.05;
  boolean canSetPosition = true; // front climber angle
  double frontClimberAnglePosition = 0.00;
  boolean shooterActivated = false;
  boolean canShoot = false;

  @Override
  public void teleopInit() {
    clearStickyFaults();
    shooter.shooterInit(); // take away later

    // System.out.println("TELEOP INIT CALLED");
    // intake.intakeInit();
    drive.isDriverControlEnabled = true;
    // shooter.shooterInit();
    shooterSpeed = 0.00;
    shooterActivated = true;
    // shooter.testVelocity(7 + 11.125 + 33.875);
    // shooter.calculatedAngle = 50.0*Math.PI/180.0;
    // distance from shooter to front of robot (no bumper (CHANGE LATER)
    // + distance from fender to center of hub + distance from front of robot to
    // fender
    timer.reset();
    timer.start();
    canShoot = false;
    shooter.masterShooterMotor.set(ControlMode.PercentOutput, 0.00);
    // shooter.testVelocity(36.0f + 33.875f + 7.25f);
    // shooter.testVelocity(shooter.getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(3.0);
  }

  // CANSparkMax climberMotor = new CANSparkMax(23, MotorType.kBrushless);
  // CANSparkMax intakeAngleMotor = new CANSparkMax(22, MotorType.kBrushless);

  boolean ballCountBoolean = false;
  boolean intakeBoolean = false;

  @Override
  public void teleopPeriodic() {
    
    //System.out.println(intake.intakeAngleMotor.getEncoder().getPosition());

    // if(Constants.stick.getRawButtonPressed(9)){
    // if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(-1).equals(1.0)){
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3.0);
    // }
    // if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(-1).equals(3.0)){
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    // }
    // }

    // drive.driveTrainByControls();
    // shooter.testingShooter();

    // System.out.println("Projectile velocity: " + shooter.calculatedVelocity +
    // "\nAngle: " + Math.toDegrees((double) shooter.calculatedAngle));
    // System.out.println("hood position: " +
    // shooter.hoodMotor.getEncoder().getPosition());
    // System.out.println("projectile velocity: " + shooter.calculatedVelocity);
    // System.out.println("calculated angle: " + shooter.calculatedAngle);
    // System.out.println("angle: " + Math.toDegrees(shooter.calculatedAngle));
    // shooter.testingShooter();
    // drive.driveTrainByControls();

    // if(Constants.stick.getRawButton(2)){
    // intake.intakeMotor.set(-0.6);
    // } else {
    // intake.intakeMotor.set(0.0);
    // }

    // System.out.println("HI");
    // drive.driveTrainByControls();
    // intake.testingIntake();

    // if(Constants.stick.getRawButton(5)){ // adjust front climber angle // (Really
    // intake)
    // climberMotor.set(-0.25);
    // //canSetPosition = true;
    // } else if (Constants.stick.getRawButton(6)){
    // climberMotor.set(0.25);
    // //canSetPosition = true;
    // } else {
    // climberMotor.set(0.0);
    // // if(canSetPosition){
    // // frontClimberAnglePosition = climberMotor.getEncoder().getPosition();
    // // canSetPosition = false;
    // // }
    // climberMotor.getPIDController().setReference(climberMotor.getEncoder().getPosition(),
    // ControlType.kPosition);
    // }

    // if(Constants.stick.getRawButton(3)){ // adjust front climber angle // (Really
    // intake)
    // intakeAngleMotor.set(-0.87);
    // //canSetPosition = true;
    // } else if (Constants.stick.getRawButton(4)){
    // intakeAngleMotor.set(0.87);
    // //canSetPosition = true;
    // } else {
    // intakeAngleMotor.set(0.0);
    // // if(canSetPosition){
    // // frontClimberAnglePosition = climberMotor.getEncoder().getPosition();
    // // canSetPosition = false;
    // // }
    // intakeAngleMotor.getPIDController().setReference(intakeAngleMotor.getEncoder().getPosition(),
    // ControlType.kPosition);
    // }

    /*
     * drive.driveTrainByControls();
     * intake.intakeOperation();
     * shooter.shooterTeleop();
     */

    // intake.testingIntake();
    // shooter.testingShooter();

    // System.out.println(intake.intakeMotor.getEncoder().getVelocity()); // 6500
    // System.out.println("Shooter speed: " + shooterSpeed);

    // System.out.println(shooter.getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    // System.out.println(shooter.calculatedVelocity);
    // System.out.println(shooter.calculatedAngle);
    // drive.driveTrainByControls();

    // System.out.println("Timer: " + timer.get());
    /*
     * System.out.println(shooter.calculatedAngle);
     * System.out.println(shooter.calculatedVelocity);
     * System.out.println(shooter.hoodMotor.getEncoder().getPosition());
     * //System.out.println(shooter.shooterWheelLinearVelocityToMotorVelocity(311));
     * System.out.println("Sensor velocity: " +
     * shooter.masterShooterMotor.getSelectedSensorVelocity());
     * 
     * System.out.println(canShoot);
     * 
     * if(timer.get() > 5.0){
     * System.out.println(shooter.shooterWheelLinearVelocityToMotorVelocity(shooter.
     * calculatedVelocity));
     * if(shooter.hoodMotor.getEncoder().getPosition() <
     * ((90-(shooter.calculatedAngle*180/Math.PI)))){
     * shooter.hoodMotor.set(0.1);
     * } else {
     * shooter.hoodMotor.set(0.0);
     * shooter.hoodMotor.getPIDController().setReference(shooter.calculatedAngle*180
     * /Math.PI, ControlType.kPosition);
     * shooter.masterShooterMotor.set(ControlMode.PercentOutput,
     * (shooter.shooterWheelLinearVelocityToMotorVelocity(shooter.
     * calculatedVelocity) / 18850.6));
     * if(shooter.masterShooterMotor.getSelectedSensorVelocity() >
     * shooter.shooterWheelLinearVelocityToMotorVelocity(shooter.calculatedVelocity)
     * - 500.0){
     * canShoot = true;
     * } else {
     * canShoot = false;
     * }
     * }
     * } else {
     * shooter.hoodMotor.set(0.0);
     * }
     * 
     * if(canShoot){
     * if(Constants.stick.getRawButton(2)){
     * intake.indexerMotor.set(-0.4);
     * } else {
     * intake.indexerMotor.set(0.0);
     * }
     * }
     */

    // shooter.masterShooterMotor.set(ControlMode.Velocity, 9600.0);
    // System.out.println(shooter.masterShooterMotor.getSelectedSensorVelocity());

    // if(Constants.stick.getRawButton(1)){
    // shooter.hoodMotor.getPIDController().setReference(40, ControlType.kPosition);
    // }

    // //drive.driveTrainByControls();

    // System.out.println(shooter.hoodMotor.getInverted());
    // System.out.println("intakeMotor: " + intake.intakeMotor.get());
    // System.out.println("Shooter set speed: " + shooterSpeed);
    // System.out.println("Shooter velocity: " +
    // shooter.masterShooterMotor.getSelectedSensorVelocity());
    // System.out.println("Hood Position: " +
    // shooter.hoodMotor.getEncoder().getPosition()); // also measure change in
    // degrees if possible
    // System.out.println("Shooter activated: " + shooterActivated);
    // //System.out.println("Front Climber Position: " +
    // climber.frontAngleMotor.getEncoder().getPosition());
    // System.out.println("*************************************************************");

    // if(Constants.stick.getRawButtonPressed(7)){
    // shooterActivated = !shooterActivated;
    // }
    // if(Constants.stick.getRawButtonPressed(11)){ // adjust shooter speed
    // shooterActivated = true;
    // shooterSpeed -= 0.05;
    // }
    // if(Constants.stick.getRawButtonPressed(12)){
    // shooterActivated = true;
    // shooterSpeed += 0.05;
    // }
    // if(shooterActivated){
    // shooter.masterShooterMotor.set(ControlMode.PercentOutput, shooterSpeed);
    // } else {
    // shooter.masterShooterMotor.set(0.0);
    // }

    // if(Constants.stick.getRawButton(2)){ // run indexer
    // intake.indexerMotor.set(-0.4);
    // } else {
    // intake.indexerMotor.set(0.0);
    // }

    // if(Constants.stick.getRawButton(1)){ // run intake wheels
    // intake.intakeMotor.set(-0.6);
    // } else {
    // intake.intakeMotor.set(0.0);
    // }

    // if(Constants.stick.getRawButton(3)){ // adjust hood
    // shooter.hoodMotor.set(-0.2);
    // } else if (Constants.stick.getRawButton(4)){
    // shooter.hoodMotor.set(0.2);
    // } else {
    // shooter.hoodMotor.set(0.0);
    // }

    // if(Constants.stick.getRawButton(5)){ // adjust front climber angle // (Really
    // intake)
    // climber.frontAngleMotor.set(-0.075);
    // //climber.frontAngleMotor.set(-0.1);
    // //climber.frontAngleMotor.getPIDController().setReference(0.1,
    // ControlType.kDutyCycle);
    // //climber.frontAngleMotor.getPIDController().setReference(frontClimberAnglePosition
    // - 2, ControlType.kPosition); // setting reference position further
    // canSetPosition = true;
    // } else if (Constants.stick.getRawButton(6)){
    // //climber.frontAngleMotor.set(0.0);
    // climber.frontAngleMotor.set(0.075);
    // //climber.frontAngleMotor.getPIDController().setReference(frontClimberAnglePosition
    // + 2, ControlType.kPosition); // setting reference position further
    // canSetPosition = true;
    // } else {
    // climber.frontAngleMotor.set(0.0);
    // if(canSetPosition){
    // frontClimberAnglePosition =
    // climber.frontAngleMotor.getEncoder().getPosition();
    // canSetPosition = false;
    // }
    // climber.frontAngleMotor.getPIDController().setReference(frontClimberAnglePosition,
    // ControlType.kPosition);
    // }

    // // // does this keep you from moving again?

    // TEST CODE:

    // if (!shooter.shooting) {
    //   drive.driveTrainByControls(Constants.stick.getRawAxis(1), Constants.stick.getRawAxis(0), Constants.stick.getRawAxis(2), false);
    // }
    // shooter.testingShooter();

    // shooter.testingShooter();

    // if(Constants.stick.getRawButtonPressed(5)){
    //   shooter.angleError -= 0.05;
    //   } else if(Constants.stick.getRawButton(6)){
    //   shooter.angleError += 0.05;
    //   } else if(Constants.stick.getRawButton(3)){
    //   shooter.angleError -= 0.01;
    //   } else if(Constants.stick.getRawButtonPressed(4)){
    //   shooter.angleError += 0.01;
    //   }

    // System.out.println(shooter.getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    // System.out.println(shooter.angleError);

    // if (Constants.stick.getRawButton(1)) {
    // drive.isDriverControlEnabled = false;
    // if
    // (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0)
    // < -5.0) {
    // Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, -0.2); // should be able to
    // // strafe and maintain robot looking
    // // at target
    // // isTargetCenter = false;
    // } else if
    // (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0)
    // > 5.0) {
    // Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
    // // isTargetCenter = false;
    // } else {
    // Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
    // // isTargetCenter = true;
    // }
    // } else {
    // drive.isDriverControlEnabled = true;
    // }

    // intake
    // if(Constants.stick.getRawButton(1)){ // intake in // Trigger
    // intake.intakeMotor.set(-0.6);
    // } else if(Constants.stick.getRawButton(2)){ // intake out // Thumb
    // intake.intakeMotor.set(0.6);
    // } else {
    // intake.intakeMotor.set(0.0); // intake stop
    // }

    // if (Constants.stick.getRawButtonPressed(1)) {
    //   intakeBoolean = !intakeBoolean;
    //   if (intakeBoolean) {
    //     timer.reset();
    //     timer.start();
    //   }
    // }

    // if (intakeBoolean) {
    //   intake.intakeMotor.set(-0.6);
    // } else {
    //   intake.intakeMotor.set(0.0);
    // }

    // System.out.println(intake.ballCount);
    // if ((intake.intakeMotor.getEncoder().getVelocity() < 6000.0) &&
    //     (intake.intakeMotor.get() == -0.6)
    //     && (!ballCountBoolean) && (timer.get() > 0.2)) { // can double-check this number
    //   ballCountBoolean = true;
    //   intake.ballCount++;
    // } else {
    //   ballCountBoolean = false;
    // }

    // // intake angle
    if (Constants.stick.getRawButton(11)) { // intake angle down
      intake.intakeAngleMotor.set(-0.28);
    } else if (Constants.stick.getRawButton(12)) { // intake angle up
      intake.intakeAngleMotor.set(0.28);
    } else {
      intake.intakeAngleMotor.set(0.0); // intake angle stop
      intake.intakeAngleMotor.getPIDController().setReference(intake.intakeAngleMotor.getEncoder().getPosition(),
          ControlType.kPosition);
    }

    // // hood
    // if (!shooter.shooting) {
    //   if (Constants.xbox.getPOV() == 270) { // hood down // D-Pad Left
    //     if (shooter.hoodMotor.getEncoder().getPosition() < 11.0) {
    //       shooter.hoodMotor.set(0.0);
    //     } else {
    //       shooter.hoodMotor.set(-0.1);
    //     }
    //   } else if (Constants.xbox.getPOV() == 90) { // hood up // D-Pad Right
    //     shooter.hoodMotor.set(0.1);
    //   } else {
    //     shooter.hoodMotor.set(0.0); // hood stop
    //   }
    // }

    // // indexer
    // if (!shooter.shooting) {
    //   if (Constants.xbox.getPOV() == 0) { // indexer in // D-Pad Up
    //     intake.indexerMotor.set(-0.4);
    //   } else if (Constants.xbox.getPOV() == 180) { // indexer out // D-Pad Down
    //     intake.indexerMotor.set(0.4);
    //   } else {
    //     intake.indexerMotor.set(0.0); // indexer stop
    //   }
    // }

    // // shooter
    // if (!shooter.shooting) {
    //   if (Constants.xbox.getRawButton(1)) { // A Button
    //     shooter.masterShooterMotor.set(0.4);
    //   } else {
    //     shooter.masterShooterMotor.set(0.0);
    //   }
    // }

    // climber
    // double climberSpeed = Constants.xbox.getRawAxis(1); // left stick up and down
    // if((climberSpeed > -0.2) && (climberSpeed < 0.2)){
    // climberSpeed = 0.0;
    // }
    // climber.centerClimberHeightMotor.set(climberSpeed);

  }

  /////////////////////////////////////////////////////////////
  // Test

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }

  //////////////////////////////////////////////////////////////
  // Disabled

  @Override
  public void disabledInit() {
    // turn limelight off
    // climberMotor.getPIDController().setReference(frontClimberAnglePosition,
    // ControlType.kPosition);
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1.0);
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(1.0);
  }

  @Override
  public void disabledPeriodic() {
    // climber.frontAngleMotor.getPIDController().setReference(frontClimberAnglePosition,
    // ControlType.kPosition);
  }

  public void clearStickyFaults() {
    PDP.clearStickyFaults();

    shooter.masterShooterMotor.clearStickyFaults();
    shooter.slaveShooterMotor.clearStickyFaults();
    shooter.hoodMotor.clearFaults();

    drive.flMotor.clearFaults();
    drive.frMotor.clearFaults();
    drive.blMotor.clearFaults();
    drive.brMotor.clearFaults();

    intake.intakeMotor.clearFaults();
    intake.indexerMotor.clearFaults();
    intake.intakeAngleMotor.clearFaults();

    climber.centerClimberHeightMotor.clearFaults();
    // add more climber motors
  }

}