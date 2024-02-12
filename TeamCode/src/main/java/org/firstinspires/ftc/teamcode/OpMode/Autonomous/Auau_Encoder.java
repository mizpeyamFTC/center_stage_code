/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Robot:_Auto_Drive_By_Encoder", group="Robot")

public class Auau_Encoder extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftFront   = null;
    private DcMotor         rightFront  = null;
    private DcMotor         leftRear   = null;
    private DcMotor         rightRear  = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    //כמות צעדים עבור סיבוב מנוע
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;     // יחס גירים
    static final double     GOB_WHEEL_DIAMETER_CM   = 9.6 ;     // For figuring circumference
    static final double     REV_WHEEL_DIAMETER_CM   = 7.5 ;     // For figuring circumference
    static final double     GOB_WHEEL_BASE_DISTANCE   = 39 ;     // For figuring circumference
    static final double     REV_WHEEL_BASE_DISTANCE   = 37 ;     // For figuring circumference

    private Rev2mDistanceSensor sensorTimeOfFlight;
    private DistanceSensor sensorDistance;
    static boolean MAIN_ROBOT  = true; // false for ROBOT_B
    private double DIST_NORM, SIDE_DIST_NORM;
    private static double  WHEEL_DIAMETER_CM;
    static final double     DRIVE_SPEED             = 0.3;
    static final double     DRIVE_2BLUE_SPEED       = 0.3;
    static final double     TURN_SPEED              = 0.3;
    static final double MIN_TURN_SPEED = 0.2;
    double WHEEL_BASE_DISTANCE;
    double FULL_ROUND ;
    static int newLeftFrontTarget;
    static int newRightFrontTarget;
    static int newLeftRearTarget;
    static int newRightRearTarget;
    double     COUNTS_PER_CM;
    IMU.Parameters myIMUparameters;
    int sleepTime = 30;
    IMU imu;
    private double  targetHeading = 0;
    private double  headingError  = 0;
    static final double     HEADING_THRESHOLD       = 0.5 ;    // How close must the heading get to the target before moving to next step.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable

    ColorSensor colorSensor;
    private static int COLOR_THRESHOLD = 400;

    private double  driveSpeed    = 0.6;
    private double  turnSpeed     = 0.6;

    private final double CORE_HEX_COUNTS_PER_REVOLUTION = 288;
    private final double CORE_HEX_DEGREES_PER_COUNT = 360/CORE_HEX_COUNTS_PER_REVOLUTION;
    @Override
    public void runOpMode() {

        initRobot();
        initIMU();
        // Initialize the drive system variables.
        initializeMotors();
        sendSuccessfulMotorResetMessage();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        stopOnBlue();
        //firstAutonomous();
  //      exercise1();
         // exercise2();
//        check_color_distance_sensors();
//        turnExerc();
//        exercise4();
//        exercise5();


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(sleepTime);  // pause to display final telemetry message.
    }



    private void turnDegrees(double degrees, double turnSpeed) {
        imu.resetYaw();
        degrees = degrees == 180? 179.9999:degrees;
        degrees = degrees == -180? -179.9999:degrees;



        while (Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))<=degrees&& opModeIsActive()){
            leftFront.setPower(turnSpeed);
            leftRear.setPower(turnSpeed);
            rightFront.setPower(-turnSpeed);
            rightRear.setPower(-turnSpeed);
            telemetry.addData("imu", String.valueOf(getHeading()));
            telemetry.addData("yaw", String.valueOf(getYaw()));
            telemetry.addData("pitch", String.valueOf(getPitch()));
            telemetry.addData("roll", String.valueOf(getRoll()));
            telemetry.update();


        }
        stopAllMotion();
        while(opModeIsActive()){
            telemetry.addData("imu", String.valueOf(getHeading()));
            telemetry.update();
        }

    }

    private void initRobot() {
        boolean bLedOn = true;
        sensorDistance = hardwareMap.get(DistanceSensor.class, "ds-1");
        colorSensor = hardwareMap.get(ColorSensor.class, "cs-1");
        colorSensor.enableLed(bLedOn);
        if (MAIN_ROBOT) {
            WHEEL_DIAMETER_CM = GOB_WHEEL_DIAMETER_CM;
            WHEEL_BASE_DISTANCE = GOB_WHEEL_BASE_DISTANCE;
            DIST_NORM = 1;
            SIDE_DIST_NORM = 1.15;
        }
        else {
            WHEEL_DIAMETER_CM =REV_WHEEL_DIAMETER_CM;
            WHEEL_BASE_DISTANCE = REV_WHEEL_BASE_DISTANCE;
            DIST_NORM = 0.9;
            SIDE_DIST_NORM =0.9;

        }
        COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_CM * 3.1415);
        FULL_ROUND = COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION*(WHEEL_BASE_DISTANCE/WHEEL_DIAMETER_CM)*1.65;
    }

    private void initIMU() {

        myIMUparameters =  new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                ));

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
// Adjust the orientation parameters to match your robot

// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(myIMUparameters);
        imu.resetYaw();


        /*imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(myIMUparameters);

        imu.resetYaw();

         */
    }

    private void exercise1() {
        double distance;
        distance =  60 ;
        right(DRIVE_SPEED,distance, sleepTime);

   //     turnDegreesByTicks(-90, TURN_SPEED);


    }
    private void firstAutonomous() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

// Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

        telemetry.addData("Yaw", Yaw);
        telemetry.update();
        sleep(1000*10);  // pause to display final telemetry message.
         Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
         Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
         Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
        turnDegreesByTicks(90, TURN_SPEED);

        double distance =  30  ;
        forward(DRIVE_SPEED, distance, sleepTime);

        robotOrientation = imu.getRobotYawPitchRollAngles();
        Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw", Yaw);
        telemetry.update();
        sleep(1000*10);  // pause to display final telemetry message.
    }
    private void exercise2() {
        double distance;

        distance =  60 ;
        forward(DRIVE_SPEED, distance, sleepTime) ;
       turnToHeading( TURN_SPEED, -90);
        distance =  90;
        forward(DRIVE_SPEED,distance, sleepTime);
        turnToHeading( TURN_SPEED, 0);

        distance =  180  ;
        forward(DRIVE_SPEED, distance, sleepTime);

      //  turnDegreesByTicks(180, TURN_SPEED);
        turnToHeading( TURN_SPEED, 180.0);               // Turn  CW to -45 Degrees


        distance =  180  ;
        forward(DRIVE_SPEED, distance, sleepTime);

        turnToHeading( TURN_SPEED, 90);               // Turn  CW to -45 Degrees
        distance = 90;
        forward(DRIVE_SPEED,distance, sleepTime);

        turnToHeading( TURN_SPEED, 0);               // Turn  CW to -45 Degrees
       distance =  180;
        forward(DRIVE_SPEED, distance, sleepTime);

        turnToHeading( TURN_SPEED, 90);               // Turn  CW to -45 Degrees

        distance =  90  ;
        forward(DRIVE_SPEED,distance, sleepTime);

       // turnDegreesByTicks(180, TURN_SPEED);
        turnToHeading( TURN_SPEED, 0.0);
        MoveForwardToBlueLine();
    }


    private void exercise3(){
        double distance;
        distance = 50 ;
        forward(DRIVE_SPEED, distance, sleepTime);
    }
    private void exercise4(){
        turnDegreesByTicks(360, TURN_SPEED);
    }

    private void exercise5(){
        double distance;
        distance = 120;
        forward(DRIVE_SPEED, distance, sleepTime);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    private void updateTelemetryRunTime() {
        telemetry.addData("Status", "Run Timed: " + runtime.toString());
        telemetry.update();
    }

    private void forward(double speed, double distance, double timeOut){
        distance *= DIST_NORM;
        encoderDrive(speed, distance, distance,distance,distance, timeOut);
    }
    private void right(double speed, double distance, double timeOut){
        distance *= SIDE_DIST_NORM;
        encoderDrive(speed, distance, -distance, -distance,distance, timeOut);
    }
    private void backwards(double speed, double distance, double timeOut){
        distance *= DIST_NORM;
        encoderDrive(speed, -distance, -distance,-distance,-distance, timeOut);
    }
    private void left(double speed, double distance, double timeOut){
        distance *= SIDE_DIST_NORM;
        encoderDrive(speed, -distance, distance,distance,-distance, timeOut);
    }

  

    private void encoderDrive(double speed,
                             double leftFrontCm, double rightFrontCm, double leftRearCm, double rightRearCm,
                             double timeoutS) {
        
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            setDriveNewTargetPosition(leftFrontCm, rightFrontCm, leftRearCm, rightRearCm);
            turnOnRunToPosition();
            runToPosition(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy()))) {
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.update();
                if (sensorDistance.getDistance(DistanceUnit.CM) < 10) {
                    runToPosition(0);
                }
                else {
                        runToPosition(speed);
                }
            }
            // Stop all motion;
            stopAllMotion();
            if(runtime.seconds() > timeoutS){
                telemetry.addData("runtime", "******TIME OUT******");
                telemetry.update();
                sleep(sleepTime);   // optional pause after each move.
            }

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(sleepTime);   // optional pause after each move.
        }
    }
    private void sendSuccessfulMotorResetMessage() {
        telemetry.addData("Starting at",  "%7d :%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition());
        telemetry.update();
    }
    private void initializeMotors() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void setDriveNewTargetPosition(double leftFrontCm, double rightFrontCm, double leftRearCm, double rightRearCm) {
        newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftFrontCm * COUNTS_PER_CM);
        newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightFrontCm * COUNTS_PER_CM);
        newLeftRearTarget = leftRear.getCurrentPosition() + (int)(leftRearCm * COUNTS_PER_CM);
        newRightRearTarget = rightRear.getCurrentPosition() + (int)(rightRearCm * COUNTS_PER_CM);

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);
    }

    private void turnDegreesByTicks(double degrees, double speed) {
        double rotTarget = FULL_ROUND * (degrees/360);

        newLeftFrontTarget = leftFront.getCurrentPosition() - (int)rotTarget;
        newRightFrontTarget = rightFront.getCurrentPosition() + (int)rotTarget;
        newLeftRearTarget = leftRear.getCurrentPosition() - (int)rotTarget;
        newRightRearTarget = rightRear.getCurrentPosition() + (int)rotTarget;

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftRear.setPower(Math.abs(speed));
        rightRear.setPower(Math.abs(speed));

        while ((opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy()))) {
            // Display it for the driver.
            telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
            telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
            telemetry.update();
        }
        // Stop all motion;
        stopAllMotion();
        telemetry.addData("runtime", "******TIME OUT******");
        telemetry.update();
        sleep(sleepTime);   // optional pause after each move.
    }
    private void turnOnRunToPosition() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void runToPosition(double speed) {
        turnOnRunToPosition();
        runtime.reset();
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftRear.setPower(Math.abs(speed));
        rightRear.setPower(Math.abs(speed));
    }
    private void stopAllMotion() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    private void stopRobot() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }



    private double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
        boolean right_turn = true;

        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading <= -180) targetHeading += 360;

        // Determine the heading current error
        double heading = getHeading();

        if (heading >= targetHeading)  {
            headingError = heading - targetHeading;
            if (headingError > 180) {
                headingError = 360 - headingError;
                right_turn = false;
            }
        }
        else {
            headingError = targetHeading - heading;
            if (headingError < 180) {
                right_turn = false;
            }
            else {
                headingError = 360 -headingError;
            }
        }

        double speed =  calc_turn_speed(headingError);
        if (right_turn) return speed;
        else return -speed;
     }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private double getYaw() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    } public double getPitch() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getPitch(AngleUnit.DEGREES);
    }public double getRoll() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getRoll(AngleUnit.DEGREES);
    }

    private boolean isRed() {
        if ( colorSensor.red() > COLOR_THRESHOLD) return true;
        return false;
    }

    private boolean isBlue() {
        if ( colorSensor.blue() > COLOR_THRESHOLD) return true;
        return false;
    }

    private void MoveForwardToBlueLine() {
        // start to move the robot forward
        moveRobot( DRIVE_2BLUE_SPEED, 0);

        // while we are not on the blue line continue to move forward
        while (  !isBlue()  ) {
                 continue;
        }
        stopRobot();
        // stop the robot engine

    }
    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
//            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
//            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftDrive.getCurrentPosition(),
//                    rightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("imu", String.valueOf(getHeading()));
        telemetry.addData("yaw", String.valueOf(getYaw()));
        telemetry.addData("pitch", String.valueOf(getPitch()));
        telemetry.addData("roll", String.valueOf(getRoll()));


//        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(targetHeading - getHeading()) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
            telemetry.addData("Heading", "%5.2f : %5.2f : %2.2f", targetHeading, getHeading(), turnSpeed);
            telemetry.update();
        }
        moveRobot(0, 0);
        telemetry.addData("Heading- error ", "%5.2f : %5.2f", getHeading(), targetHeading );
        telemetry.update();
        // Stop all motion;
        moveRobot(0, 0);
    }

    private double calc_turn_speed (double heading_error) {

        if (Math.abs(heading_error) > 10) return TURN_SPEED;
        return MIN_TURN_SPEED;
    }

    private void mysleep (int sec) {
        sleep(1000*sec);

    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double linear_speed, double turn_speed) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(linear_speed+turn_speed);
        leftRear.setPower(linear_speed+turn_speed);
        rightFront.setPower(linear_speed-turn_speed);
        rightRear.setPower(linear_speed-turn_speed);
   }

    private void check_color_distance_sensors() {

        while (opModeIsActive()){
            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.update();
        }
    }

    private void turnExerc(){
        turnToHeading(TURN_SPEED, 45.0);
        mysleep(1);
        turnToHeading( TURN_SPEED, -45.0);               // Turn  CW to -45 Degrees
        mysleep(1);
        turnToHeading( TURN_SPEED, 180.0);               // Turn  CW to -45 Degrees
        mysleep(1);


    }

    private void stopOnBlue(){
        MoveForwardToBlueLine();
    }

}
