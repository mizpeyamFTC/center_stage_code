package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;



@TeleOp(name = "OffSeasonTeleop")
public class OffSeasonTeleop  extends LinearOpMode {

    private DcMotor
            leftFront,
            leftRear,
            rightFront,
            rightRear,
            elevatorRight,
            elevatorLeft,
            leftArmMotor,
            rightArmMotor;

    private Servo
            finalArmJoint,
            leftClawServo,
            rightClawServo,
            planeServo;



    private boolean leftClawOpen = true;
    private boolean rightClawOpen = true;

    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private IMU controlHubIMU;
    private DistanceSensor leftClawSensor;
    private DistanceSensor rightClawSensor;
    private double DIST_NORM, SIDE_DIST_NORM, WHEEL_DIAMETER_CM, WHEEL_BASE_DISTANCE, FULL_ROUND;
    private double COUNTS_PER_CM;
    private ElapsedTime runtime;
    private ElapsedTime matchRuntime;
    private double motorMax = 0.5;
    private int newLeftFrontTarget;
    private int newRightFrontTarget;
    private int newLeftRearTarget;
    private int newRightRearTarget;
    private int sleepTime = 30;
    private double targetHeading = 0;
    private double headingError = 0;
    private double turnSpeed = 0.6;


    private final double CORE_HEX_COUNTS_PER_REVOLUTION = 288;
    private final double CORE_HEX_DEGREES_PER_COUNT = 360 / CORE_HEX_COUNTS_PER_REVOLUTION;
    private final double COUNTS_PER_MOTOR_REV = 28;    //כמות צעדים עבור סיבוב מנוע
    private final double DRIVE_GEAR_REDUCTION = 20.0;     // יחס גירים
    private final double GOB_WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    private final double REV_WHEEL_DIAMETER_CM = 7.5;     // For figuring circumference
    private final double GOB_WHEEL_BASE_DISTANCE = 39;     // For figuring circumference
    private final double REV_WHEEL_BASE_DISTANCE = 37;     // For figuring circumference
    private final double TURN_SPEED = 0.3;
    private final double MIN_TURN_SPEED = 0.2;
    private final double HEADING_THRESHOLD = 0.5;    // How close must the heading get to the target before moving to next step.
    private final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    private final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    private final int COLOR_THRESHOLD = 400; // blue
    private final double DRIVE_MAX = 0.9;
    private final double TURN_MAX = 0.5;
    private final boolean MAIN_ROBOT = true; // false for ROBOT_B

    private final long SHORT_SIDE_WAIT = 10000;

    private int LEFT_ARM_HOME_POSITION;
    private int RIGHT_ARM_HOME_POSITION;
    private int LEFT_ARM_UP_POSITION;
    private int RIGHT_ARM_UP_POSITION;
    private int LEFT_ARM_SCORE_POSITION;
    private int RIGHT_ARM_SCORE_POSITION;
    private int LEFT_ARM_INTAKE_POSITION;
    private int RIGHT_ARM_INTAKE_POSITION;
    private int LEFT_HOME_TO_UP_OFFSET = 60;
    private int RIGHT_HOME_TO_UP_OFFSET = 60;
    private int LEFT_HOME_TO_SCORE_OFFSET = 130;
    private int RIGHT_HOME_TO_SCORE_OFFSET = 130;
    //private int LEFT_HOME_TO_INTAKE_OFFSET = 195;
    //private int RIGHT_HOME_TO_INTAKE_OFFSET = 195;
    private int LEFT_HOME_TO_INTAKE_OFFSET = 192;
    private int RIGHT_HOME_TO_INTAKE_OFFSET = 192;



    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;


    private final double HOME_TO_UP_SPEED = 0.7;
    private final double UP_TO_HOME_SPEED = 0.4;
    private final double UP_TO_SCORE_SPEED = 0.4;
    private final double SCORE_TO_UP_SPEED = 0.7;
    private final double SCORE_TO_INTAKE_SPEED = 0.4;
    private final double INTAKE_TO_SCORE_SPEED = 0.8;

    private final double WRIST_INTAKE_POSITION = 0.55;
    //private final int LIFT_INTAKE_POSITION =-340;
    private final int LIFT_INTAKE_POSITION =-300;
    private final double LEFT_CLAW_CLOSED = 0;
    private final double LEFT_CLAW_OPENED = 1;
    private final double RIGHT_CLAW_CLOSED = 1;
    private final double RIGHT_CLAW_OPENED = 0;
    enum Position{
        home,
        up,
        score,
        intake;
    }

    Position currentPosition = Position.home;
    Thread clawDistanceSensorThread;
    Thread positionThread;
    Thread updateClawsStateThread;
    Thread updateTelemetryThread;
    Thread moveElevatorThread;
    Thread driveThread;

    Thread armControlThread;
    Thread elevatorMotorsModeThread;
    Thread robotControl;
    Thread updateLiftLockThread;


    /*TODO: create thread for claw angle*/
    /*TODO: create thread for claws state */
    /*TODO: create method to make open/close buttons for the claws */

    //*************************THREADS*************************
    // a synchronized method is a method that can work and lock to the hardware in a synchronized matter with other methods that try to lock in in parallel
    // because only 1 source can read/write data on to the hardware device thus locking access to it
    // we use synchronization to regulate and order the approach manner of the different sources so that the access wont be chaotic but organized(synchronized)

    private synchronized void updateCurrentArmPosition(int rightArmPosition){
        if(rightArmPosition<=RIGHT_ARM_UP_POSITION/2){
            currentPosition = Position.home;

        }
        else if (RIGHT_ARM_UP_POSITION/2<rightArmPosition
                &&rightArmPosition<=RIGHT_ARM_UP_POSITION+(RIGHT_ARM_SCORE_POSITION-RIGHT_ARM_UP_POSITION)/2) {
            currentPosition = Position.up;

        }
        else if (RIGHT_ARM_UP_POSITION+(RIGHT_ARM_SCORE_POSITION-RIGHT_ARM_UP_POSITION)/2<rightArmPosition
                &&rightArmPosition<=RIGHT_ARM_SCORE_POSITION+(RIGHT_ARM_INTAKE_POSITION-RIGHT_ARM_SCORE_POSITION)/2) {
            currentPosition = Position.score;

        }
        else{
            currentPosition = Position.intake;
        }
    }
    private int getCurrentArmPosition(){
        return rightArmMotor.getCurrentPosition();

    }
    private synchronized void closeClawsOnPixelDetection(){
        if(leftClawSensor.getDistance(DistanceUnit.CM)<5) closeLeftClaw();
        if(rightClawSensor.getDistance(DistanceUnit.CM)<5) closeRightClaw();

    }
    private synchronized void changeElevatorMotorsModeOnChange(){
        if(gamepad2.left_trigger>0.02||gamepad2.right_trigger>0.02){
            elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
       /*
        if(gamepad2.right_stick_y>0.02 || gamepad2.right_stick_y<-0.02){
            leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        */
    }
    private synchronized void armControl(){
        double rightArmPower, leftArmPower;
        double elevatorPower;

        if(gamepad2.left_bumper){
            //if (leftClawOpen) closeLeftClaw();
            //else openLeftClaw();
            closeLeftClaw();
        }
        if(gamepad2.right_bumper){
            //if (rightClawOpen) closeRightClaw();
            //else openRightClaw();//almost done
            closeRightClaw();
        }
        if(gamepad2.a){
            //if (leftClawOpen) closeLeftClaw();
            //else openLeftClaw();
            openRightClaw();
        }
        if(gamepad2.x){
            //if (rightClawOpen) closeRightClaw();
            //else openRightClaw();//almost done
            openLeftClaw();
        }

        if(gamepad1.a) resetIMU();
        if(gamepad2.ps){ openLeftClaw(); openRightClaw();}
        if(gamepad2.dpad_down) requestMoveToIntake();
        if(gamepad2.dpad_right) requestMoveToScore();
        if(gamepad2.dpad_up) requestMoveToUp();
        if(gamepad2.dpad_left) requestMoveToHome();


        double gp2LeftStickY, gp2LeftStickX, gp2RightStickY, gp2RightStickX;
        gp2LeftStickY = gamepad2.left_stick_y;
        gp2LeftStickX = gamepad2.left_stick_x;
        gp2RightStickY = gamepad2.right_stick_y;
        gp2RightStickX = gamepad2.right_stick_x;

        rightArmPower = gp2RightStickY*0.7;
        leftArmPower = gp2RightStickY*0.7;

        if(leftArmMotor.getMode()== DcMotor.RunMode.RUN_TO_POSITION && (leftArmPower>0.05 ||leftArmPower<-0.05)){
            leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArmMotor.setPower(rightArmPower);
            leftArmMotor.setPower(leftArmPower);

        }
        else if(leftArmMotor.getMode()!= DcMotor.RunMode.RUN_TO_POSITION){
            rightArmMotor.setPower(rightArmPower);
            leftArmMotor.setPower(leftArmPower);
        }


        elevatorPower = gamepad2.right_trigger-gamepad2.left_trigger;
        if(elevatorLeft.getMode()== DcMotor.RunMode.RUN_TO_POSITION && (elevatorPower>0.05 ||elevatorPower<-0.05)){
            elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevatorRight.setPower(elevatorPower);
            elevatorLeft.setPower(elevatorPower);

        }
        else if(elevatorLeft.getMode()!= DcMotor.RunMode.RUN_TO_POSITION){
            elevatorRight.setPower(elevatorPower);
            elevatorLeft.setPower(elevatorPower);
        }





    }

    private synchronized void robotControlThread(){
        driveByInput();
        armControl();

    }

    private void updateClawsState(){
        if(LEFT_CLAW_OPENED>LEFT_CLAW_CLOSED){
            if(leftClawServo.getPosition()>(Math.abs(LEFT_CLAW_OPENED-LEFT_CLAW_CLOSED)/2)){
                leftClawOpen = true;
            }
            else{
                leftClawOpen = false;
            }

        }
        else if(LEFT_CLAW_OPENED<LEFT_CLAW_CLOSED){
            if(leftClawServo.getPosition()<(Math.abs(LEFT_CLAW_OPENED-LEFT_CLAW_CLOSED)/2)){
                leftClawOpen = true;
            }
            else{
                leftClawOpen = false;
            }

        }
        if(RIGHT_CLAW_OPENED>RIGHT_CLAW_CLOSED){
            if(rightClawServo.getPosition()>(Math.abs(RIGHT_CLAW_OPENED-RIGHT_CLAW_CLOSED)/2)){
                rightClawOpen = true;
            }
            else{
                rightClawOpen = false;
            }

        }
        else if(RIGHT_CLAW_OPENED<RIGHT_CLAW_CLOSED){
            if(rightClawServo.getPosition()<(Math.abs(RIGHT_CLAW_OPENED-RIGHT_CLAW_CLOSED)/2)){
                rightClawOpen = true;
            }
            else{
                rightClawOpen = false;
            }

        }


    }



    private void updateTelemetry(){
        updateLiftTelemetry();
        updateArmTelemetry();
        telemetryAddIMUData();

    }
    //*************************THREADS*************************



    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        waitForStart();


        // a thread is a virtual processor that allows independent and parallel execution of code

        /*clawDistanceSensorThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()&&!isStopRequested()){
                    try {
                        closeClawsOnPixelDetection();

                    } catch (Exception e){
                        e.printStackTrace();

                    }
                }
            }
        });

         */
       /* moveElevatorThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()&&!isStopRequested()){
                    try {
                        elevatorToIntake();


                    } catch (Exception e){
                        e.printStackTrace();

                    }
                }
            }
        });
       */
        /*driveThread = new Thread(new Runnable() {
            @Override
            synchronized public void run() {
                while(opModeIsActive()&&!isStopRequested()){
                    try {
                        driveByInput();

                    } catch (Exception e){
                        e.printStackTrace();

                    }
                }
            }
        });

         */
        /*armControlThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()&&!isStopRequested()){
                    try {
                        armControl();

                    } catch (Exception e){
                        e.printStackTrace();

                    }
                }
            }
        });

         */
        positionThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()&&!isStopRequested()){
                    try {

                        updateCurrentArmPosition(getCurrentArmPosition());

                    } catch (Exception e){
                        e.printStackTrace();

                    }
                }
            }
        });
        updateClawsStateThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()&&!isStopRequested()){
                    try {
                        updateClawsState();

                    } catch (Exception e){
                        e.printStackTrace();
                    }
                }
            }
        });
        updateTelemetryThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()&&!isStopRequested()){
                    try {
                        updateTelemetry();

                    } catch (Exception e){
                        e.printStackTrace();

                    }
                }
            }
        });
        elevatorMotorsModeThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()&&!isStopRequested()){
                    try {
                        changeElevatorMotorsModeOnChange();





                    } catch (Exception e){
                        e.printStackTrace();

                    }
                }
            }
        });

        robotControl = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()&&!isStopRequested()){
                    try {
                        robotControlThread();

                    } catch (Exception e){
                        e.printStackTrace();

                    }
                }
            }
        });





        double rightArmPower, leftArmPower;
       /* int rightTargetPosition, leftTargetPosition;
        rightTargetPosition = rightArmMotor.getCurrentPosition();
        leftTargetPosition = leftArmMotor.getCurrentPosition();

        */

        positionThread.start();
        updateClawsStateThread.start();
        updateTelemetryThread.start();
        //elevatorMotorsModeThread.start();
        robotControl.start();



        while (opModeIsActive()){

            if(isStopRequested() ){
                positionThread.stop();
                updateClawsStateThread.stop();
                updateTelemetryThread.stop();
                elevatorMotorsModeThread.stop();
                robotControl.stop();
            }
            //driveByInput();
            /*
            if(gamepad2.left_bumper){
                if (leftClawOpen) closeLeftClaw();
                else openLeftClaw();
            }
            if(gamepad2.right_bumper){
                if (rightClawOpen) closeRightClaw();
                else openRightClaw();//almost done
            }
            if(gamepad1.a) resetIMU();
            if(gamepad2.ps){ openLeftClaw(); openRightClaw();}
            if(gamepad2.dpad_down) moveToIntake();
            if(gamepad2.dpad_right) moveToScore();
            if(gamepad2.dpad_up) moveToUp();
            if(gamepad2.dpad_left) moveToHome();

            elevatorLeft.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            elevatorRight.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            double gp2LeftStickY, gp2LeftStickX, gp2RightStickY, gp2RightStickX;
            gp2LeftStickY = gamepad2.left_stick_y;
            gp2LeftStickX = gamepad2.left_stick_x;
            gp2RightStickY = gamepad2.right_stick_y;
            gp2RightStickX = gamepad2.right_stick_x;

            rightArmPower = gp2RightStickY*0.7;
            leftArmPower = gp2RightStickY*0.7;
            rightArmMotor.setPower(rightArmPower);
            leftArmMotor.setPower(leftArmPower);

             */



            /*if(-0.01<gp2RightStickY&& gp2RightStickY<0.01){
                if(!breaking){
                    rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightTargetPosition = rightArmMotor.getCurrentPosition();
                    leftTargetPosition = leftArmMotor.getCurrentPosition();
                    rightArmMotor.setTargetPosition(rightTargetPosition);
                    leftArmMotor.setTargetPosition(leftTargetPosition);

                    breaking = true;
                }
                rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArmMotor.setPower(1);
                leftArmMotor.setPower(1);
                telemetry.addLine("breaking");

            }

             */
            /*else{
            rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightArmMotor.setPower(rightArmPower);
            leftArmMotor.setPower(leftArmPower);

            }

             */







            //clawDistanceSensorThread.start();

            //closeClawsOnPixelDetection();

        }
    }




    private synchronized void moveArmToSetPosition(Position currentPosition, Position targetPosition) {
        if(currentPosition == targetPosition){
            switch (currentPosition){
                case home:
                    moveArmToHome(1);
                    break;
                case up:
                    moveArmToUp(1);
                    break;
                case score:
                    moveArmToScore(1);
                    break;
                case intake:
                    moveArmToIntake(1);
                    break;
            }
        }
        else{
            if (currentPosition == Position.home){
                if (targetPosition == Position.up){
                    moveArmToUp(HOME_TO_UP_SPEED);
                }
                else if(targetPosition == Position.score||targetPosition == Position.intake){
                    moveArmToSetPosition(currentPosition,Position.up );
                    moveArmToSetPosition(Position.up,targetPosition );
                }

            }

            else if (currentPosition == Position.up) {
                if (targetPosition == Position.home){
                    moveArmToHome(UP_TO_HOME_SPEED);
                }
                else if (targetPosition == Position.score){
                    moveArmToScore(UP_TO_SCORE_SPEED);
                }
                else if (targetPosition == Position.intake) {
                    moveArmToSetPosition(currentPosition, Position.score);
                    moveArmToSetPosition(Position.score, targetPosition);
                }
            }

            else if (currentPosition == Position.score) {
                if(targetPosition == Position.up){
                    moveArmToUp(SCORE_TO_UP_SPEED);
                }
                else if(targetPosition == Position.intake){
                    moveArmToIntake(SCORE_TO_INTAKE_SPEED);
                }
                else if (targetPosition == Position.home) {
                    moveArmToSetPosition(currentPosition, Position.up);
                    moveArmToSetPosition( Position.up, targetPosition);
                }
            }

            else if (currentPosition == Position.intake) {
                if (targetPosition == Position.score){
                    moveArmToScore(INTAKE_TO_SCORE_SPEED);
                }
                else if(targetPosition == Position.up|| targetPosition==Position.home){
                    moveArmToSetPosition(currentPosition,Position.score );
                    moveArmToSetPosition(Position.score,targetPosition );
                }

            }
        }
    }



    private synchronized void moveArmToTargetPosition(int leftTargetPosition, int rightTargetPosition, double power){
        leftArmMotor.setTargetPosition(leftTargetPosition);
        rightArmMotor.setTargetPosition(rightTargetPosition);
            rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArmMotor.setPower(power);
            leftArmMotor.setPower(power);



    }

    private synchronized void elevatorToIntake(){
        elevatorLeft.setTargetPosition(LIFT_INTAKE_POSITION);
        elevatorRight.setTargetPosition(LIFT_INTAKE_POSITION);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setPower(1);
        elevatorRight.setPower(1);


    }
    private synchronized void elevatorToHome(){
        elevatorLeft.setTargetPosition(-10);
        elevatorRight.setTargetPosition(-10);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setPower(1);
        elevatorRight.setPower(1);


    }



    private synchronized void requestMoveToHome(){
        elevatorToHome();
        moveArmToSetPosition(currentPosition, Position.home);
    }
    private synchronized void requestMoveToUp(){

        moveArmToSetPosition(currentPosition, Position.up);
    }
    private synchronized void requestMoveToScore(){
        moveArmToSetPosition(currentPosition, Position.score);
    }
     private synchronized void requestMoveToIntake(){
        //elevator to -300
        elevatorToIntake();
        moveArmToSetPosition(currentPosition, Position.intake);
    }


    private synchronized void moveArmToHome(double power){
        finalArmJoint.setPosition(0);
        moveArmToTargetPosition(LEFT_ARM_HOME_POSITION, RIGHT_ARM_HOME_POSITION, power);
    }
    private synchronized void moveArmToUp(double power){
        finalArmJoint.setPosition(0);
        moveArmToTargetPosition(LEFT_ARM_UP_POSITION, LEFT_ARM_UP_POSITION, power);
    }
    private synchronized void moveArmToScore(double power){
        finalArmJoint.setPosition(0.5);
        moveArmToTargetPosition(LEFT_ARM_SCORE_POSITION, LEFT_ARM_SCORE_POSITION, power);
    }
    private synchronized void moveArmToIntake(double power){
        finalArmJoint.setPosition(WRIST_INTAKE_POSITION);
        moveArmToTargetPosition(LEFT_ARM_INTAKE_POSITION, RIGHT_ARM_INTAKE_POSITION, power);
    }


    private synchronized void driveByInput(){
        double denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
        double rotX, rotY;
        double y, x,rx;
        double botHeading;
        double maxOutputPower;

        y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;

        motorMax = 0.5+gamepad1.right_trigger*0.5;

        botHeading = controlHubIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //telemetryAddIMUData();
        // Rotate the movement direction counter to the bot's rotation
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        maxOutputPower = motorMax / denominator;
        frontLeftPower =maxOutputPower* (rotY + rotX + rx);
        backLeftPower = maxOutputPower*(rotY - rotX + rx);
        frontRightPower = maxOutputPower*(rotY - rotX - rx);
        backRightPower = maxOutputPower*(rotY + rotX - rx);

        powerDriveMotors(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

    }
    private synchronized void powerDriveMotors(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }



    private void positionToIntake(){}
    private void positionToScore(){
    }
    private void positionToRest(){}
    private synchronized void openLeftClaw(){ leftClawServo.setPosition(LEFT_CLAW_OPENED);}
    private synchronized void closeLeftClaw(){ leftClawServo.setPosition(LEFT_CLAW_CLOSED);}
    private synchronized void openRightClaw(){ rightClawServo.setPosition(RIGHT_CLAW_OPENED);}
    private synchronized void closeRightClaw(){ rightClawServo.setPosition(RIGHT_CLAW_CLOSED);}



    private synchronized void updateLiftTelemetry(){
        telemetry.addLine("left lift motor: " + elevatorLeft.getCurrentPosition());
        telemetry.addLine("right lift motor: " + elevatorRight.getCurrentPosition());
        telemetry.addLine("left lift state: " + elevatorLeft.getMode().toString());
        telemetry.addLine("right lift state: " + elevatorRight.getMode().toString());
    }
    private synchronized void updateArmTelemetry()
    {
        telemetry.addData("current position: ",  currentPosition.toString());
        telemetry.addLine("left motor pos:" + leftArmMotor.getCurrentPosition());
        telemetry.addLine("right motor pos:" + rightArmMotor.getCurrentPosition());
        telemetry.addLine("left motor state:" + leftArmMotor.getMode().toString());
        telemetry.addLine("right motor state:" + rightArmMotor.getMode().toString());
        telemetry.addLine("left claw state:" +(leftClawOpen? "opened": "closed") + leftClawServo.getPosition());
        telemetry.addLine("right claw state:" + (rightClawOpen? "opened": "closed") + rightClawServo.getPosition());




    }
    private synchronized void telemetryAddIMUData() {
        telemetry.addData("yaw:", controlHubIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) );
        telemetry.addData("pitch:", controlHubIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES) );
        telemetry.addData("roll:", controlHubIMU.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES) );
        telemetry.update();
    }

    //*************************INITIALIZE*************************

    private void initializeRobot() {
        //initCamera();
        initMotors();
        closeRightClaw();
        closeLeftClaw();
        initRobotParameters();
        initIMU();

    }
    private void initRobotParameters() {
        runtime = new ElapsedTime();
        matchRuntime = new ElapsedTime();
        initDriveConstants();
        initArmConstants();
    }

    private void initDriveConstants() {
        boolean bLedOn = true;
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds-1");
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
    private void initArmConstants(){
        LEFT_ARM_HOME_POSITION = leftArmMotor.getCurrentPosition();
        RIGHT_ARM_HOME_POSITION = rightArmMotor.getCurrentPosition();

        LEFT_ARM_UP_POSITION = LEFT_ARM_HOME_POSITION + LEFT_HOME_TO_UP_OFFSET;
        RIGHT_ARM_UP_POSITION = RIGHT_ARM_HOME_POSITION + RIGHT_HOME_TO_UP_OFFSET;

        LEFT_ARM_SCORE_POSITION = LEFT_ARM_HOME_POSITION + LEFT_HOME_TO_SCORE_OFFSET;
        RIGHT_ARM_SCORE_POSITION = RIGHT_ARM_HOME_POSITION + RIGHT_HOME_TO_SCORE_OFFSET;

        LEFT_ARM_INTAKE_POSITION = LEFT_ARM_HOME_POSITION + LEFT_HOME_TO_INTAKE_OFFSET;
        RIGHT_ARM_INTAKE_POSITION = RIGHT_ARM_HOME_POSITION + RIGHT_HOME_TO_INTAKE_OFFSET;
    }

    private void initIMU() {
        // Retrieve the IMU from the hardware map
        controlHubIMU = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        controlHubIMU.initialize(parameters);
        controlHubIMU.resetYaw();


    }
    private void initMotors(){
        assignHardware();
        //intakeMotor = hardwareMap.dcMotor.get("intake");
        // liftMotor = hardwareMap.dcMotor.get("lift");
        initMotorsDirection();
        resetAllMotorsEncoders();
        setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    private void assignHardware() {


        leftFront = hardwareMap.dcMotor.get("leftFront");//color: blank, ExpansionHub, port:2
        rightFront = hardwareMap.dcMotor.get("rightFront");//color: blank, ControlHub, port:2
        leftRear = hardwareMap.dcMotor.get("leftRear");//color: blank, ExpansionHub, port:3
        rightRear = hardwareMap.dcMotor.get("rightRear");//color: blank, ControlHub, port:3
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");//color: RED, ExpansionHub, port:1
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");//color: BLUE, ControlHub, port:1
        finalArmJoint = hardwareMap.get(Servo.class,"finalArmJoint" );//color: RED-BLUE, ControlHub, port:0
        leftClawServo = hardwareMap.get(Servo.class,"leftClawServo" );//color: RED, ExpansionHub, port:0
        rightClawServo = hardwareMap.get(Servo.class,"rightClawServo" );//color: BLUE, ControlHub, port:0
        planeServo = hardwareMap.get(Servo.class, "planeServo");//color: blank, ControlHub, port:0
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds-1");//color: blank, ControlHub, port:0
        //leftClawSensor = hardwareMap.get(DistanceSensor.class, "leftClawSensor");//color: blank, ControlHub, port:0
        //rightClawSensor = hardwareMap.get(DistanceSensor.class, "rightClawSensor");//color: blank, ControlHub, port:0
        elevatorRight = hardwareMap.dcMotor.get("elevatorRight");//color: BLUE, ControlHub, port:0
        elevatorLeft = hardwareMap.dcMotor.get("elevatorLeft");//color: RED, ExpansionHub, port:0

    }

    private void resetAllMotorsEncoders() {
        setAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void setAllMotorsMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);
        elevatorRight.setMode(runMode);
        elevatorLeft.setMode(runMode);
        rightArmMotor.setMode(runMode);
        leftArmMotor.setMode(runMode);
    }
    private void initMotorsDirection() {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftArmMotor.setDirection(DcMotor.Direction.FORWARD);
        rightArmMotor.setDirection(DcMotor.Direction.REVERSE);
        finalArmJoint.setDirection(Servo.Direction.FORWARD);
        elevatorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private void turnOnRunToPosition() {
        setAllMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void resetIMU(){
        initIMU();
    }


    //*************************INITIALIZE*************************
}

