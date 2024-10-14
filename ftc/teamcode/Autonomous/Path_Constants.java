package org.firstinspires.ftc.teamcode.Autonomous;
import java.util.List;

// Declaring all of our path constant arrays:
public class Path_Constants
{
    public static List<Double> motors;
    // BSB Center Park & BSB Corner Park:
    public static double[] BSB_WP_Center = new double[10];
    public static double[] BSB_WP_Left = new double[11];
    public static double[] BSB_WP_Right = new double[11];

    // BSR Center Park & BSR Corner Park: ******
    public static double [] BSR_WP_Center = new double[15];
    public static double [] BSR_WP_Left = new double[15];
    public static double [] BSR_WP_Right = new double[15];

    // FSB Center Park & FSB Corner Park:
    public static double[] FSB_WP_Center = new double[16];
    public static double [] FSB_WP_Left = new double[12];
    public static double [] FSB_WP_Right = new double[12];


    // FBR Center Park & FBR Corner Park:
    public static double [] FSR_WP_Center = new double[15];
    public static double [] FSR_WP_Left = new double[15];
    public static double [] FSR_WP_Right = new double[15];
    // Setting the values to the path constants:
    public static void SetData()
    {
// BLUE:
        // BSB-Center Spike:
        BSB_WP_Center[0]=16.5; // Move Forward by 16.5
        BSB_WP_Center[1]=8.0; // Move Forward by 8.0
        BSB_WP_Center[2]=3.0; // Move Backward by 3.0
        BSB_WP_Center[3]=1.0; // Move Backward by 1.0
        BSB_WP_Center[4]=23.0; // Spin Clockwise by 23.0
        BSB_WP_Center[5]=30.0; // Move Backward by 30.0
        // Corner Park Constants:
        BSB_WP_Center[6]=22.0; // Strafe Right by 22.0
        // Center Park Constants:
        BSB_WP_Center[8]=24.0; // Strafe Left by 24.0
        BSB_WP_Center[9]=10.0; // Move Backward by 10.0

        // BSB-Left Spike:
        BSB_WP_Left[0] = 9.5; // Strafe Left by 9.5
        BSB_WP_Left[1] = 10.0; // Move Forward by 10.0
        BSB_WP_Left[2] = 13.0; // Move Forward by 13.0
        BSB_WP_Left[3] = 3.0; // Move Backward by 3.0
        BSB_WP_Left[4] = 1.0; // Move Backward 1.0
        BSB_WP_Left[5] = 21.5; // Spin Clockwise by 21.5
        BSB_WP_Left[6] = 22.0; // Move Backward by 22.0
        // Corner Park Constants:
        BSB_WP_Left[7] = 16.0; // Strafe Right by 16.0
        // Center Park Constants:
        BSB_WP_Left[9] = 32.0; // Strafe Left by 32.0
        BSB_WP_Left[10] = 10.0; // Move Backward by 10.0

        // BSB-Right Spike:
        BSB_WP_Right[0] = 7.0; // Strafe Left by 7.0
        BSB_WP_Right[1] = 27.0; // Move Forward by 27.0
        BSB_WP_Right[2] = 22.0; // Spin Clockwise by 22.0
        BSB_WP_Right[3] = 6.5; // Move Forward by 6.5
        BSB_WP_Right[4] = 3.0; // Move Backward by 3.0
        BSB_WP_Right[5] = 27.0; // Move Backward by 27.0
        BSB_WP_Right[6] = 3.0; // Strafe Left by 3.0
        // Corner Park Constants:
        BSB_WP_Right[7] = 29.0; // Strafe Right by 29.0
        // Center Park Constants:
        BSB_WP_Right[9] = 14.0; // Strafe Left by 14.0
        BSB_WP_Right[10] = 10.0; // Move Backward by 10.0

        // FSB-Center Spike:
        FSB_WP_Center[0] = 16.5; // Move Forward by 16.5
        FSB_WP_Center[1] = 8.5; // Move Forward by 8.0
        FSB_WP_Center[2] = 3.0; // Move Backward by 3.25
        FSB_WP_Center[3] = 16.0; // strafe right by 16.0
        FSB_WP_Center[4] = 30.25; // Move forward by 30.25
        FSB_WP_Center[5] = 20.0; // strafe left by 20.0
        FSB_WP_Center[6] = 21.50; // Spin Clockwise by 21.50
        FSB_WP_Center[7] = 45.0; // Move Backward by 67 (pt. 1)
        FSB_WP_Center[8] = 22.0; // Move Backward by 67 (pt. 2)
        FSB_WP_Center[9] = 24.0; // strafe right by 24
        FSB_WP_Center[10] = 7.5; // Backward by 7 inches
        FSB_WP_Center[11] = 23.5; // Strafe left by 22.5
        FSB_WP_Center[13] = 24.5; // Strafe Left by 24.5

        // FSB-Left Spike:
        FSB_WP_Left[0] = 8.0; // Move Forward by 8.0
        FSB_WP_Left[1] = 7.0; // Strafe Right by 7.0
        FSB_WP_Left[2] = 22.0; // Spin Anticlockwise by 22.0
        FSB_WP_Left[3] = 24.0; // Strafe Right by 24.0
        FSB_WP_Left[4] = 6.0; // Move Forward by 5.5
        FSB_WP_Left[5] = 3.25; // Move Backward by 3.25
        FSB_WP_Left[6] = 7.0; // Move Backward by 7.0
        FSB_WP_Left[7] = 20.0; // Strafe Right 20.0
        FSB_WP_Left[8] = 43.0; // Spin Anticlockwise by 43.0
        FSB_WP_Left[9] = 52.0; // Move Backward by 52.0
        FSB_WP_Left[10] = 38.0; // Move Backward by 36.0
        FSB_WP_Left[11] = 33.5; // Move right by 34.5

        // FSB-Right Spike:
        FSB_WP_Right[0] = 10.0; // Move Forward by 10.0
        FSB_WP_Right[1] = 10.25; // Strafe Right by 10.25
        FSB_WP_Right[2] = 7.0; // Move Forward by 7.0
        FSB_WP_Right[3] = 3.0; // Move Backward by 3.0
        FSB_WP_Right[4] = 11.5; // Strafe Left by 11.5
        FSB_WP_Right[5] = 33.0; // Move Forward by 33.0
        FSB_WP_Right[6] = 22.0; // Spin Clockwise by 22.0
        FSB_WP_Right[7] = 56.2; // Move Backward by 56.2
        FSB_WP_Right[8] = 22.0; // Move Backward by 21.0
        FSB_WP_Right[9] = 18.0; // Move right by 20.0

// RED:
        // BSR-Center Spike:
        BSR_WP_Center[0]=16.5; // Move Forward by 16.5
        BSR_WP_Center[1]=8.0; // Move Forward by 8.0
        BSR_WP_Center[2]=3.0; // Move Backward by 3.0
        BSR_WP_Center[3]=3.0; // Move Backward by 3.0
        BSR_WP_Center[4]=24.0; // Spin Clockwise by 24.0
        BSR_WP_Center[5]=30.0; // Move Backward by 30.0
        // Corner Park Constants:
        BSR_WP_Center[6]=22.0; // Strafe Right by 22.0
        // Center Park Constants:
        BSR_WP_Center[8]=24.0; // Strafe Left by 24.0
        BSR_WP_Center[9] = 10.0; // Move Backward by 10.0

        // BSR-Left Spike:
        BSR_WP_Left[0] = 7.0; // Strafe Left by 7.0
        BSR_WP_Left[1] = 27.0; // Move Forward by 27.0
        BSR_WP_Left[2] = 22.0; // Spin Clockwise by 22.0
        BSR_WP_Left[3] = 6.5; // Move Forward by 6.5
        BSR_WP_Left[4] = 3.0; // Move Backward by 3.0
        BSR_WP_Left[5] = 27.75; // Move Backward by 27.0
        BSR_WP_Left[6] = 3.0; // Strafe Left by 3.0
        // Corner Park Constants:
        BSR_WP_Left[7] = 29.0; // Strafe Right by 29.0
        // Center Park Constants:
        BSR_WP_Left[9] = 14.0; // Strafe Left by 14.0
        BSR_WP_Left[10] = 10.0; // Move Backward by 10.0

        // BSR-Right Spike:
        BSR_WP_Right[0] = 10.0; // Strafe Left by 9.5
        BSR_WP_Right[1] = 10.0; // Move Forward by 10.0
        BSR_WP_Right[2] = 13.0; // Move Forward by 13.0
        BSR_WP_Right[3] = 3.0; // Move Backward by 3.0
        BSR_WP_Right[4] = 1.0; // Move Backward 1.0
        BSR_WP_Right[5] = 21.5; // Spin Clockwise by 21.5
        BSR_WP_Right[6] = 23.0; // Move Backward by 21.0
        // Corner Park Constants:
        BSR_WP_Right[7] = 16.0; // Strafe Right by 16.0
        // Center Park Constants:
        BSR_WP_Right[9] = 32.0; // Strafe Left by 32.0
        BSR_WP_Right[10] = 10.0; // Move Backward by 10.0

        // FSR-Center Spike:
        FSR_WP_Center[0] = 16.5; // Move Forward by 16.5
        FSR_WP_Center[1] = 8.0; // Move Forward by 8.0
        FSR_WP_Center[2] = 3.25; // Move Backward by 3.25
        FSR_WP_Center[3] = 16.0; // strafe right by 16.0
        FSR_WP_Center[4] = 30.25; // Move forward by 30.25
        FSR_WP_Center[5] = 20.0; // strafe left by 20.0
        FSR_WP_Center[6] = 21.50; // Spin Clockwise by 21.5
        FSR_WP_Center[7] = 45.0; // Move Backward by 67 (pt. 1)
        FSR_WP_Center[8] = 22.0; // Move Backward by 67 (pt. 2)
        FSR_WP_Center[9] = 24.0; // strafe right by 24
        FSR_WP_Center[10] = 7.0; // Backward by 6.0 inches
        FSR_WP_Center[11] = 24.5; // Strafe left by 22.5
        FSR_WP_Center[13] = 22.5; // Strafe Left by 22.5

        // FSR-Left Spike:
        FSR_WP_Left[0] = 10.0; // Move Forward by 10.0
        FSR_WP_Left[1] = 10.5; // Strafe Right by 10.25
        FSR_WP_Left[2] = 7.0; // Move Forward by 7.0
        FSR_WP_Left[3] = 3.0; // Move Backward by 3.0
        FSR_WP_Left[4] = 11.5; // Strafe Left by 11.5
        FSR_WP_Left[5] = 33.0; // Move Forward by 33.0
        FSR_WP_Left[6] = 22.0; // Spin Clockwise by 22.0
        FSR_WP_Left[7] = 56.2; // Move Backward by 56.2
        FSR_WP_Left[8] = 22.0; // Move Backward by 22
        FSR_WP_Left[9] = 22.0; // Move right by 20.0

        // FSR-Right Spike:
        FSR_WP_Right[0] = 8.0; // Move Forward by 8.0
        FSR_WP_Right[1] = 7.0; // Strafe Left by 7.0
        FSR_WP_Right[2] = 21.5; // Spin Clockwise by 22.0
        FSR_WP_Right[3] = 26.0; // Strafe Left by 24.0
        FSR_WP_Right[4] = 8.25; // Move Forward by 9.0
        FSR_WP_Right[5] = 3.25; // Move Backward by 3.25
        FSR_WP_Right[6] = 9.0; // Move Backward by 9.0
        FSR_WP_Right[7] = 20.0; // Strafe Left 20.0
        FSR_WP_Right[8] = 44.5; // Spin Clockwise by 43.0
        FSR_WP_Right[9] = 51.0; // Move Backward by 50.0
        FSR_WP_Right[10] = 42.0; // Move Backward by 35.0
        FSR_WP_Right[11] = 33.0; // Strafe Left by 37.0 //*CHANGE*
    }


}

