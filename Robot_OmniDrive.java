package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDriveByGyro_Linear;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class defines all the specific hardware for a three wheel omni-bot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left drive"
 * Motor channel:  Right drive motor:        "right drive"
 * Motor channel:  Rear  drive motor:        "back drive"
 *
 * These motors correspond to three drive locations spaced 120 degrees around a circular robot.
 * Each motor is attached to an omni-wheel. Two wheels are in front, and one is at the rear of the robot.
 *
 * Robot motion is defined in three different axis motions:
 * - Axial    Forward/Backwards      +ve = Forward
 * - Lateral  Side to Side strafing  +ve = Right
 * - Yaw      Rotating               +ve = CCW
 */


public class Robot_OmniDrive
{
    // Private Members
    private LinearOpMode myOpMode;

    private DcMotor  leftDrive      = null;
    private DcMotor  rightDrive     = null;


   // private DcMotor  backDrive      = null;

    private double  driveAxial      = 0 ;   // Positive is forward
    private double  driveLateral    = 0 ;   // Positive is right
    private double  driveYaw        = 0 ;   // Positive is CCW

    //Define Core Hex Motor 76:1
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: Core Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     IN_RANGE                = 1;

    static final double     P_TURN_COEFF           = 0.15;
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private ElapsedTime     runtime = new ElapsedTime();

    BNO055IMU      gyro ;


    /* Constructor */
    public Robot_OmniDrive(){

    }


    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {
        // Save reference to Hardware map
        myOpMode = opMode;


        //set Gyro sensor parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "gyro";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Define and Initialize Motors
        leftDrive        = myOpMode.hardwareMap.get(DcMotor.class, "left drive");
        rightDrive       = myOpMode.hardwareMap.get(DcMotor.class, "right drive");
        gyro = opMode.hardwareMap.get(BNO055IMU.class,"gyro");


       // backDrive        = myOpMode.hardwareMap.get(DcMotor.class, "back drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
      //  backDrive.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise

        //use RUN_USING_ENCODERS because encoders are installed.
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop all robot motion by setting each axis value to zero
        moveRobot(0,0,0) ;
        gyro.initialize(parameters);
    }

    public void manualDrive()  {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        //  (note: The joystick goes negative when pushed forwards, so negate it)
        setAxial(-myOpMode.gamepad1.left_stick_y);
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(-myOpMode.gamepad1.right_stick_x);
    }


    /***
     * void moveRobot(double axial, double lateral, double yaw)
     * Set speed levels to motors based on axes requests
     * @param axial     Speed in Fwd Direction
     * @param lateral   Speed in lateral direction (+ve to right)
     * @param yaw       Speed of Yaw rotation.  (+ve is CCW)
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();
    }

    /***
     * void moveRobot()
     * This method will calculate the motor speeds required to move the robot according to the
     * speeds that are stored in the three Axis variables: driveAxial, driveLateral, driveYaw.
     * This code is setup for a three wheeled OMNI-drive but it could be modified for any sort of omni drive.
     *
     * The code assumes the following conventions.
     * 1) Positive speed on the Axial axis means move FORWARD.
     * 2) Positive speed on the Lateral axis means move RIGHT.
     * 3) Positive speed on the Yaw axis means rotate COUNTER CLOCKWISE.
     *
     * This convention should NOT be changed.  Any new drive system should be configured to react accordingly.
     */
    public void moveRobot() {
        // calculate required motor speeds to acheive axis motions
       // double back = driveYaw + driveLateral;
        double left = driveYaw - driveAxial - (driveLateral * 0.5);
        double right = driveYaw + driveAxial - (driveLateral * 0.5);

        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(left), Math.abs(right));
        //max = Math.max(max, Math.abs(left));
        if (max > 1.0)
        {
           // back /= max;
            right /= max;
            left /= max;
        }

        // Set drive motor power levels.
      //  backDrive.setPower(back);
        leftDrive.setPower(left/10);
        rightDrive.setPower(right/10);

        // Display Telemetry
        myOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        myOpMode.telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f]]", left, right);
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;



        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ( (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                myOpMode.telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }
            // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

    }

    //Gyro drive straight method

    public void gyroDrive(double speed, double distance, double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        //get current left motor pos and right motor pos of robot then allow the robot move to target
        moveCounts = (int) (distance * COUNTS_PER_INCH);
        newLeftTarget = leftDrive.getCurrentPosition() + moveCounts;
        newRightTarget = rightDrive.getCurrentPosition() + moveCounts;

        //set Target and run to pos
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Range.clip(Math.abs(speed), 0.0,1.0);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while (leftDrive.isBusy() && rightDrive.isBusy()) {
             // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);
            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0) steer *= -1.0;
            leftSpeed = speed - steer;
            rightSpeed = speed + steer;
            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {leftSpeed /= max; rightSpeed /= max;}
            leftDrive.setPower(leftSpeed);
            rightDrive.setPower(rightSpeed);
            // Display drive status for the driver.
            myOpMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            myOpMode.telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            myOpMode.telemetry.addData("Actual",  "%7d:%7d",      leftDrive.getCurrentPosition(),
                                                                rightDrive.getCurrentPosition());
            myOpMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            myOpMode.telemetry.update();

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }


    }

    //getError determines the error between the target angle and the robot's current heading




    public double getError(double targetAngle) {

        double robotError;
        // calculate error in -179 to +180 range
        robotError = targetAngle - gyro.getAngularOrientation().thirdAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
            return Range.clip(error * PCoeff, -1, 1);
    }

    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setLateral(double lateral)  {driveLateral = Range.clip(lateral, -1, 1); }
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
      //  backDrive.setMode(mode);
    }
    //create a gyro turn method for our robot
    public void gyroTurn(double speed, double angle, double pCoeff) {
        double error = getError(angle);
        double steer;
        boolean heading = false;
        double leftSpeed;
        double rightSpeed;
        while (!heading) {

            if (Math.abs(error) <= IN_RANGE) {
                steer = 0.0;
                leftSpeed = 0.0;
                rightSpeed = 0.0;
                heading = true;

            } else {
                steer = getSteer(error,pCoeff);
                rightSpeed = speed * steer;
                leftSpeed = -rightSpeed;
            }

            leftDrive.setPower(leftSpeed);
            rightDrive.setPower(rightSpeed);
        }
    }
}

