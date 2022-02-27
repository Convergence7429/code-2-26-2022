package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

public class Intake {

    // 1 spark max for intake
    // 1 spark max for indexer (built in for intake I would say. No reason for
    // another class if possible)
    // 1 spark max for intake angle

    CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorIndex, MotorType.kBrushless);
    CANSparkMax indexerMotor = new CANSparkMax(Constants.indexerMotorIndex, MotorType.kBrushless);
    CANSparkMax intakeAngleMotor = new CANSparkMax(Constants.intakeAngleMotorIndex, MotorType.kBrushless);

    I2C.Port i2cPort = I2C.Port.kOnboard;
    ColorSensorV3 indexerColorSensor = new ColorSensorV3(i2cPort);
    ColorMatch colorMatch = new ColorMatch();

    Color redBall = new Color(0.561, 0.232, 0.114);
    Color blueBall = new Color(0.143, 0.427, 0.429);
    // might have to make a third color for whatever it is looking at without a ball
    // so it doesn't just match

    int ballCount;

    // public double intakeIndexerEncoderCount = 200; // encoder Count to get from
    // indexer to under shooter

    boolean intaking = false;
    // Timer intakeTimer = new Timer();

    // boolean hasIntakeVelocityBeenMeasured = false;

    // two Color Sensors

    public void intakeInit() {
        // put intake down?
        // hasIntakeVelocityBeenMeasured = false;
        intaking = false;

        intakeMotor.enableVoltageCompensation(12.5); // same speed every time
        indexerMotor.enableVoltageCompensation(12.5);

        colorMatch.addColorMatch(redBall);
        colorMatch.addColorMatch(blueBall);
        // add third match?
    }

    public void testingIntake() {
        Color detectedColor = indexerColorSensor.getColor();
        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);
        // if (match.de)

        if (Constants.stick.getRawButtonPressed(1)) {
            intaking = !intaking;
            if (intaking) {
                // timer.reset();
                // timer.start();
                intakeMotor.set(-0.6);
            } else {
                intakeMotor.set(0.0);
            }
        }

        if(!Robot.shooter.shooting){
            //if() // if red alliance and detected color is red, indexerMotor.set(0.0);
            // else {indexerMotor.set(0.0)}
        }

        // if (intaking) {

        // } else {

        // }

        /*
         * 
         * print ballCount
         * 
         * if get raw button
         * intaking = true;
         * 
         * if(intaking){
         * intakeMotor set 0.6
         * if(get slow down velocity)
         * ball count++
         * }
         * 
         */

        // System.out.println("ballCount: " + ballCount);
        // System.out.println("Intake velocity: " +
        // intakeMotor.getEncoder().getVelocity());

        // if(Constants.stick.getRawButtonPressed(10)){
        // intaking = !intaking;
        // if(intaking){
        // intakeTimer.reset();
        // intakeTimer.start();
        // }
        // }

        // if(intaking){
        // intakeMotor.set(0.6);
        // if(intakeTimer.get() > 1.0){
        // if(intakeMotor.getEncoder().getVelocity() < 6480.0){ // can double-check this
        // number
        // ballCount++;
        // }
        // }
        // }
    }

    // public void intakeOperation(){

    // // if(ColorSensor1.getColor()){
    // // colorSensor1Activated = true;
    // // }

    // if((ballCount < 2) && Constants.stick.getRawButtonPressed(1)){
    // intaking = true;
    // intakeTimer.reset();
    // intakeTimer.start();
    // }

    // if(ballCount == 2){
    // intaking = false;
    // }

    // if(intaking){
    // intakeMotor.set(0.6);
    // if(intakeTimer.get() > 1.0){ // time for intake motor to speed up
    // if((intakeMotor.getEncoder().getVelocity() < (6480.0/* velocity for 0.6 -
    // 10.0% */)) && !hasIntakeVelocityBeenMeasured){
    // ballCount++;
    // indexerMotor.getEncoder().setPosition(0.0);
    // hasIntakeVelocityBeenMeasured = true;
    // }
    // if(hasIntakeVelocityBeenMeasured && ballCount == 1){
    // if(indexerMotor.getEncoder().getPosition() < 500){ // number needs to change
    // indexerMotor.set(0.4);
    // } else {
    // hasIntakeVelocityBeenMeasured = false;
    // indexerMotor.set(0.0);
    // }
    // }
    // }

    /*
     * if(!colorSensor1Activated){
     * intakeMotor.set(0.5);
     * indexerMotor.getEncoder().setPosition(0.0);
     * } else {
     * if(ballCount == 0){
     * intakeMotor.set(0.0);
     * //indexerMotor.set(Constants.quadraticPositionAndSpeed(0.05, 0.4,
     * intakeIndexerEncoderCount, indexerMotor.getEncoder().getPosition()));
     * indexerMotor.set(0.2);
     * ballCount++;
     * if(indexerMotor.getEncoder().getPosition() > intakeIndexerEncoderCount){
     * indexerMotor.set(0.0);
     * indexerMotor.getEncoder().setPosition(0.0);
     * }
     * } else {
     * indexerMotor.set(0.0);
     * indexerMotor.getEncoder().setPosition(0.0);
     * colorSensor1Activated = false;
     * }
     * }
     */
    // }
    // }

    /*
     * public void teleopIntake1(){
     * 
     * // if(ColorSensor1.getColor()){
     * // colorSensor1Activated = true;
     * // }
     * 
     * if((ballCount < 2) && Constants.stick.getRawButtonPressed(1)){
     * intaking = true;
     * }
     * 
     * if(ballCount == 2){
     * intaking = false;
     * }
     * 
     * 
     * 
     * if(intaking){
     * if(!colorSensor1Activated){
     * intakeMotor.set(0.5);
     * indexerMotor.getEncoder().setPosition(0.0);
     * } else {
     * if(ballCount == 0){
     * intakeMotor.set(0.0);
     * //indexerMotor.set(Constants.quadraticPositionAndSpeed(0.05, 0.4,
     * intakeIndexerEncoderCount, indexerMotor.getEncoder().getPosition()));
     * indexerMotor.set(0.2);
     * ballCount++;
     * if(indexerMotor.getEncoder().getPosition() > intakeIndexerEncoderCount){
     * indexerMotor.set(0.0);
     * indexerMotor.getEncoder().setPosition(0.0);
     * }
     * } else {
     * indexerMotor.set(0.0);
     * indexerMotor.getEncoder().setPosition(0.0);
     * colorSensor1Activated = false;
     * }
     * }
     * }
     * }
     * 
     * 
     * 
     * // if(!colorSensor1Activated){
     * // intakeMotor.set(intakeMotorSpeed);
     * // } else {
     * 
     * // }
     * 
     * // if(colorSensor1Activated){
     * // // reset encoder counts of indexerMotor
     * // // indexerMotor.set(quadraticController(0.05, indexerMotorSpeed,
     * indexerMotor.getEncoder().getPosition(), encoderCount));
     * // if(indexerMotor.getEncoder().getPosition() > encoderCount){
     * // colorSensor1Activated = false;
     * // }
     * // }
     * // } else {
     * // intakeMotor.set(0.0);
     * // }
     */

}