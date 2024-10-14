package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Autonomous.Auto_Struct.RobotDirection;

public class MecanumRobotUtilities {
    //https://docs.revrobotics.com/duo-control/sensors/encoders/motor-based-encoders
    public final static double COUNTS_PER_MOTOR_REV =  28;            // rev REV-41-1291

    // 1:4 gear ( ratio 3.61) - https://www.revrobotics.com/rev-41-1602/
    // 1:5 gear ( ratio 5.23) - https://www.revrobotics.com/rev-41-1603/
    public final static double DRIVE_GEAR_REDUCTION = 18.88;// 3.61*5.23

    // Wheel Spec ( 75 mm = 2.95276 inches)(https://www.revrobotics.com/rev-45-1655/
    public final static double MECANUM_WHEEL_DIAMETER_INCHES = 2.95276;
    public final static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (MECANUM_WHEEL_DIAMETER_INCHES * 3.1413);

    public static DcMotor FrontLeftMotor;
    public static DcMotor FrontRightMotor;
    public static DcMotor BackRightMotor;
    public static DcMotor BackLeftMotor;
    //public static DcMotor LinearArm;

    public static double FrontLeftmotorPower = 0;
    public static double FrontRightmotorPower = 0;
    public static double BackLeftmotorPower = 0;
    public static double BackRightmotorPower = 0;


    /****************************************************************************************************************/
    /*Initialization function that sets up the hardware interfaces*/
    public static void InitializeHardware(LinearOpMode opMode)
    {
        /* Initialize standard Hardware interfaces */
        // Define and Initialize Motors
        MecanumRobotUtilities.FrontLeftMotor = opMode.hardwareMap.get(DcMotor.class, "frontLeft");     //Mapped to Motor port#0 REV HUB#2
        MecanumRobotUtilities.FrontRightMotor = opMode.hardwareMap.get(DcMotor.class, "frontRight");   //Mapped to Motor port#1 REV HUB#1
        MecanumRobotUtilities.BackRightMotor = opMode.hardwareMap.get(DcMotor.class, "backRight");     //Mapped to Motor port#2 REV HUB#1
        MecanumRobotUtilities.BackLeftMotor = opMode.hardwareMap.get(DcMotor.class, "backLeft");     //Mapped to Motor port#3 REV HUB#1

       MecanumRobotUtilities.FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
       MecanumRobotUtilities.FrontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //set the Zero Power to be Brake so that the motors are not floating.
        MecanumRobotUtilities.FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MecanumRobotUtilities.FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MecanumRobotUtilities.BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MecanumRobotUtilities.BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

       /****************************************************************************************************************/
    //Function that can drive the robot in any direction in inches.
    //Robot direction
    public static void encoderDrive(RobotDirection direction, double motorPower,
                                    double distanceInInches, double timeoutS, LinearOpMode opMode) {

        ElapsedTime runtime = new ElapsedTime();

        int FrontLeftMotorNewPosition = 0;
        int FrontRightMotorNewPosition = 0;
        int BackLeftMotorNewPosition = 0;
        int BackRightMotorNewPosition = 0;
       // int LinearArmMotorNewPosition = 0;

        // reset the motorticks..
        MecanumRobotUtilities.FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MecanumRobotUtilities.FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MecanumRobotUtilities.BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MecanumRobotUtilities.BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int FLM_InitialPoss = MecanumRobotUtilities.FrontLeftMotor.getCurrentPosition();
        int FRM_InitialPoss = MecanumRobotUtilities.FrontRightMotor.getCurrentPosition();
        int BLM_InitialPoss = MecanumRobotUtilities.BackLeftMotor.getCurrentPosition();
        int BRM_InitialPoss = MecanumRobotUtilities.BackRightMotor.getCurrentPosition();

        // Display it for the driver.
        String status = String.format("Current Position before FLeft:%d  FRight: %d BLeft:%d  BRight: %d",
                MecanumRobotUtilities.FrontLeftMotor.getCurrentPosition(),
                MecanumRobotUtilities.FrontRightMotor.getCurrentPosition(),
                MecanumRobotUtilities.BackLeftMotor.getCurrentPosition(),
                MecanumRobotUtilities.BackRightMotor.getCurrentPosition());
        //MecanumRobotUtilities.LinearArm.getCurrentPosition();
        opMode.telemetry.addData(status, "");
        opMode.telemetry.update();
       
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //based on the direction, set the motor power for all motors
            switch (direction) {
                case FORWARD:
                    MecanumRobotUtilities.FrontLeftmotorPower = motorPower;
                    MecanumRobotUtilities.FrontRightmotorPower = motorPower;
                    MecanumRobotUtilities.BackLeftmotorPower = motorPower;
                    MecanumRobotUtilities.BackRightmotorPower = motorPower;

                    // Determine new target position and pass to the motor controller for each of the motors
                    FrontLeftMotorNewPosition = FLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontRightMotorNewPosition = FRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = BLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = BRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to forward motion", "");
                    //opMode.telemetry.update();
                    break;
                case BACKWARD:
                    MecanumRobotUtilities.FrontLeftmotorPower = -motorPower;
                    MecanumRobotUtilities.FrontRightmotorPower = -motorPower;
                    MecanumRobotUtilities.BackLeftmotorPower = -motorPower;
                    MecanumRobotUtilities.BackRightmotorPower = -motorPower;
                    // Determine new target position and pass to the motor controller for each of the motors
                    FrontLeftMotorNewPosition = FLM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontRightMotorNewPosition = FRM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = BLM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = BRM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to backward motion", "");
                    //opMode.telemetry.update();
                    break;
                case SPINCLOCKWISE:
                    MecanumRobotUtilities.FrontLeftmotorPower = motorPower;
                    MecanumRobotUtilities.FrontRightmotorPower = -motorPower;
                    MecanumRobotUtilities.BackLeftmotorPower = motorPower;
                    MecanumRobotUtilities.BackRightmotorPower = -motorPower;
                    FrontLeftMotorNewPosition = FLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontRightMotorNewPosition = FRM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = BLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = BRM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to clock wise spin motion", "");
                    //opMode.telemetry.update();
                    break;
                case SPINANTICLOCKWISE:
                    MecanumRobotUtilities.FrontLeftmotorPower = -motorPower;
                    MecanumRobotUtilities.FrontRightmotorPower = motorPower;
                    MecanumRobotUtilities.BackLeftmotorPower = -motorPower;
                    MecanumRobotUtilities.BackRightmotorPower = motorPower;
                    FrontLeftMotorNewPosition = FLM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontRightMotorNewPosition = FRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = BLM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = BRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to spin anti clockwise motion", "");
                    //opMode.telemetry.update();
                    break;
                case RIGHT:
                    /*FrontLeftmotorPower = motorPower;
                    FrontRightmotorPower = -motorPower;
                    BackLeftmotorPower = -motorPower;
                    BackRightmotorPower = motorPower;*/
                    FrontRightmotorPower = motorPower;
                    BackRightmotorPower = -motorPower;
                    FrontLeftmotorPower = -motorPower;
                    BackLeftmotorPower = motorPower;
                    // Determine new target position and pass to the motor controller for each of the motors

                    FrontRightMotorNewPosition = FRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = BRM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontLeftMotorNewPosition = FLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = BLM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to right motion", "");
                    opMode.telemetry.update();
                    break;
                case LEFT:
                    /*MecanumRobotUtilities.FrontLeftmotorPower = -motorPower;
                    MecanumRobotUtilities.FrontRightmotorPower = motorPower;
                    MecanumRobotUtilities.BackLeftmotorPower = motorPower;
                    MecanumRobotUtilities.BackRightmotorPower = -motorPower;*/

                    MecanumRobotUtilities.FrontRightmotorPower = -motorPower;
                    MecanumRobotUtilities.BackRightmotorPower = motorPower;
                    MecanumRobotUtilities.FrontLeftmotorPower = motorPower;
                    MecanumRobotUtilities.BackLeftmotorPower = -motorPower;

                    FrontRightMotorNewPosition = FRM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = BRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    FrontLeftMotorNewPosition = FLM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = BLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to left motion", "");
                    opMode.telemetry.update();
                    break;
                case LEFTFRONTDIAGONAL:
                    MecanumRobotUtilities.FrontLeftmotorPower = 0;
                    MecanumRobotUtilities.FrontRightmotorPower = 0;
                    MecanumRobotUtilities.BackLeftmotorPower = motorPower;
                    MecanumRobotUtilities.BackRightmotorPower = motorPower;

                    // Determine new target position and pass to the motor controller for each of the motors
                    FrontLeftMotorNewPosition = MecanumRobotUtilities.FrontLeftMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontRightMotorNewPosition = MecanumRobotUtilities.FrontRightMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = MecanumRobotUtilities.BackLeftMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = MecanumRobotUtilities.BackRightMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to Left Diagnal fwd motion", "");
                    opMode.telemetry.update();
                    break;
                case RIGHTFRONTDIAGONAL:
                    MecanumRobotUtilities.FrontLeftmotorPower = motorPower;
                    MecanumRobotUtilities.FrontRightmotorPower = motorPower;
                    MecanumRobotUtilities.BackLeftmotorPower = 0;
                    MecanumRobotUtilities.BackRightmotorPower = 0;
                    // Determine new target position and pass to the motor controller for each of the motors
                    FrontLeftMotorNewPosition = MecanumRobotUtilities.FrontLeftMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontRightMotorNewPosition = MecanumRobotUtilities.FrontRightMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = MecanumRobotUtilities.BackLeftMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = MecanumRobotUtilities.BackRightMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to right front diag motion", "");
                    opMode.telemetry.update();
                    break;
                case RIGHTBACKWARDDIAGONAL:
                    MecanumRobotUtilities.FrontLeftmotorPower = 0;
                    MecanumRobotUtilities.FrontRightmotorPower = 0;
                    MecanumRobotUtilities.BackLeftmotorPower = -motorPower;
                    MecanumRobotUtilities.BackRightmotorPower = -motorPower;
                    // Determine new target position and pass to the motor controller for each of the motors
                    FrontLeftMotorNewPosition = MecanumRobotUtilities.FrontLeftMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontRightMotorNewPosition = MecanumRobotUtilities.FrontRightMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = MecanumRobotUtilities.BackLeftMotor.getCurrentPosition() + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = MecanumRobotUtilities.BackRightMotor.getCurrentPosition() + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to left back dia motion", "");
                    opMode.telemetry.update();
                    break;
                case LEFTBACKWARDDIAGONAL:
                    MecanumRobotUtilities.FrontLeftmotorPower = -motorPower;
                    MecanumRobotUtilities.FrontRightmotorPower = -motorPower;
                    MecanumRobotUtilities.BackLeftmotorPower = 0;
                    MecanumRobotUtilities.BackRightmotorPower = 0;
                    // Determine new target position and pass to the motor controller for each of the motors
                    FrontLeftMotorNewPosition = MecanumRobotUtilities.FrontLeftMotor.getCurrentPosition() + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontRightMotorNewPosition = MecanumRobotUtilities.FrontRightMotor.getCurrentPosition() + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = MecanumRobotUtilities.BackLeftMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = MecanumRobotUtilities.BackRightMotor.getCurrentPosition() + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to right back dia motion", "");
                    opMode.telemetry.update();
                case ROTATERIGHTBACKPIVOT_CLOCK:
                    MecanumRobotUtilities.FrontLeftmotorPower = motorPower;
                    MecanumRobotUtilities.FrontRightmotorPower = 0;
                    MecanumRobotUtilities.BackLeftmotorPower = motorPower;
                    MecanumRobotUtilities.BackRightmotorPower = 0;

                    // Determine new target position and pass to the motor controller for each of the motors
                    FrontLeftMotorNewPosition = FLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontRightMotorNewPosition = FRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = BLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = BRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to forward motion", "");
                    //opMode.telemetry.update();
                    break;
                case    ROTATELEFTBACKPIVOT_ANTICLOCK:
                    MecanumRobotUtilities.FrontLeftmotorPower = 0;
                    MecanumRobotUtilities.FrontRightmotorPower = motorPower;
                    MecanumRobotUtilities.BackLeftmotorPower = 0;
                    MecanumRobotUtilities.BackRightmotorPower = motorPower;

                    // Determine new target position and pass to the motor controller for each of the motors
                    FrontLeftMotorNewPosition = FLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    FrontRightMotorNewPosition = FRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = BLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = BRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to forward motion", "");
                    //opMode.telemetry.update();
                    break;
                case ROTATEBACKCENTERPIVOT_ANTICLOCK:

                    MecanumRobotUtilities.FrontRightmotorPower = 0;
                    MecanumRobotUtilities.BackRightmotorPower = motorPower;
                    MecanumRobotUtilities.FrontLeftmotorPower = motorPower;
                    MecanumRobotUtilities.BackLeftmotorPower = 0;

                    FrontRightMotorNewPosition = FRM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackRightMotorNewPosition = BRM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    FrontLeftMotorNewPosition = FLM_InitialPoss + (int) (-distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);
                    BackLeftMotorNewPosition = BLM_InitialPoss + (int) (distanceInInches * MecanumRobotUtilities.COUNTS_PER_INCH);

                    opMode.telemetry.addData("Set to left motion", "");
                    opMode.telemetry.update();
                    break;

                default:
                    //nothing to do -- should not reach here.
                    break;
            }

            status = String.format("Target Position FLeft:%d  FRight: %d BLeft:%d  BRight: %d",
                    FrontLeftMotorNewPosition, FrontRightMotorNewPosition, BackLeftMotorNewPosition, BackRightMotorNewPosition);
            opMode.telemetry.addData(status, "");
            opMode.telemetry.update();
       
            //set the motors to coast... so the robot does not jerk
            FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            //set the target for each motor
            MecanumRobotUtilities.FrontLeftMotor.setTargetPosition(FrontLeftMotorNewPosition);
            MecanumRobotUtilities.FrontRightMotor.setTargetPosition(FrontRightMotorNewPosition);
            MecanumRobotUtilities.BackRightMotor.setTargetPosition(BackRightMotorNewPosition);
            MecanumRobotUtilities.BackLeftMotor.setTargetPosition(BackLeftMotorNewPosition);

            MecanumRobotUtilities.FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Turn On RUN_TO_POSITION
            MecanumRobotUtilities.FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Turn On RUN_TO_POSITION
            MecanumRobotUtilities.BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Turn On RUN_TO_POSITION
            MecanumRobotUtilities.BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Turn On RUN_TO_POSITION

            // reset the timeout time and start motion.
            runtime.reset();

            MecanumRobotUtilities.FrontLeftMotor.setPower(MecanumRobotUtilities.FrontLeftmotorPower);
            MecanumRobotUtilities.FrontRightMotor.setPower(MecanumRobotUtilities.FrontRightmotorPower);
            MecanumRobotUtilities.BackRightMotor.setPower(MecanumRobotUtilities.BackRightmotorPower);
            MecanumRobotUtilities.BackLeftMotor.setPower(MecanumRobotUtilities.BackLeftmotorPower);

            // keep looping while we are still active, and there is time left, and if any of the motor is running.
            while (opMode.opModeIsActive() && !opMode.isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (MecanumRobotUtilities.FrontLeftMotor.isBusy() &&
                            MecanumRobotUtilities.FrontRightMotor.isBusy()
                            && MecanumRobotUtilities.BackRightMotor.isBusy()
                            && MecanumRobotUtilities.BackLeftMotor.isBusy()
                    )) {
                ; //just wait
            }//End While loop


            // Stop all motion;
        
            MecanumRobotUtilities.FrontLeftMotor.setPower(0);
            MecanumRobotUtilities.BackLeftMotor.setPower(0);
            MecanumRobotUtilities.FrontRightMotor.setPower(0);
            MecanumRobotUtilities.BackRightMotor.setPower(0);

            FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            MecanumRobotUtilities.FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MecanumRobotUtilities.FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MecanumRobotUtilities.BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MecanumRobotUtilities.BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Display it for the driver.
            status = String.format("Current Position after FLeft:%d  FRight: %d BLeft:%d  BRight: %d",
                    MecanumRobotUtilities.FrontLeftMotor.getCurrentPosition(),
                    MecanumRobotUtilities.FrontRightMotor.getCurrentPosition(),
                    MecanumRobotUtilities.BackLeftMotor.getCurrentPosition(),
                    MecanumRobotUtilities.BackRightMotor.getCurrentPosition());
            opMode.telemetry.addData(status, "");
            opMode.telemetry.update();
        }
    }

}



