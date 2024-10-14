package org.firstinspires.ftc.teamcode.Autonomous;
import java.util.List;
// Declaring all of our path constant arrays:
public class Path_Constants_Feb29
{
    public static List<Double> motors;
    // BSB Center Park & BSB Corner Park:
    public static double[] BSB_WP_Center = new double[15];
    public static double[] BSB_WP_Left = new double[15];
    public static double[] BSB_WP_Right = new double[15];

    // BSR Center Park & BSR Corner Park:
    public static double [] BSR_WP_Center = new double[15];
    public static double [] BSR_WP_Left = new double[15];
    public static double [] BSR_WP_Right = new double[15];

    // FSB:
    public static double[] FSB_WP_Center = new double[15];
    public static double [] FSB_WP_Left = new double[15];
    public static double [] FSB_WP_Right = new double[15];


    // FBR:
    public static double [] FSR_WP_Center = new double[15];
    public static double [] FSR_WP_Left = new double[15];
    public static double [] FSR_WP_Right = new double[15];
    // Setting the values to the path constants:
    public static void SetData()
    {
// BLUE:
        // BSB-Center Spike:
        BSB_WP_Center[0] = 16.5; // Move Forward by 16.5
        BSB_WP_Center[1] = 8.0; // Move Forward by 8.0
        BSB_WP_Center[2] = 3.0; // Move Backward by 3.0
        BSB_WP_Center[3] = 1.0; // Move Backward by 1.0
        BSB_WP_Center[4]=23.0; // Spin Clockwise by 23.0
        BSB_WP_Center[5]=30.0; // Move Backward by 30.0
        // Corner Park Constants:
        BSB_WP_Center[6]=22.0; // Strafe Right by 22.0
        // BSB_WP_Center[7] = 5.0; // Move Backward by 5.0
        // Center Park Constants:
        BSB_WP_Center[8]=24.0; // Strafe Left by 24.0
        BSB_WP_Center[9]=10.0; // Move Backward by 10.0

        // BSB-Left Spike:
        BSB_WP_Left[0] = 9.5; // Strafe Left by 9.5
        BSB_WP_Left[1] = 10.0; // Move Forward by 10.0
        BSB_WP_Left[2] = 13.0; // Move Forward by 14.0
        BSB_WP_Left[3] = 3.0; // Move Backward by 3.0
        BSB_WP_Left[4] = 1.0; // Move Backward 1.0
        BSB_WP_Left[5] = 21.5; // Spin Clockwise by 21.5
        BSB_WP_Left[6] = 22.0; // Move Backward by 21.0
        // Corner Park Constants:
        BSB_WP_Left[7] = 16.0; // Strafe Right by 16.0
        // Center Park Constants:
        BSB_WP_Left[9] = 32.0; // Strafe Left by 32.0
        BSB_WP_Left[10] = 10.0;

        // BSB-Right Spike:
        BSB_WP_Right[0] = 7.0; // Strafe Left by 7.0
        BSB_WP_Right[1] = 27.0; // Move Forward by 27.0
        BSB_WP_Right[2] = 22.0; // Spin Clockwise by 22.0
        BSB_WP_Right[3] = 6.5; // Move Forward by 6.5
        BSB_WP_Right[4] = 3.0; // Move Backward by 3.0
        BSB_WP_Right[5] = 27.0; // Move Backward by 27.0
        BSB_WP_Right[6] = 3.0; // Strafe Left by 2.0
        // Corner Park Constants:
        BSB_WP_Right[7] = 29.0; // Strafe Right by 29.0
        // Center Park Constants:
        BSB_WP_Right[9] = 14.0; // Strafe Left by 14.0
        BSB_WP_Right[10] = 10.0;

        // FSB-Center Spike:
        FSB_WP_Center[0] = 16.5; // Move Forward by 16.5
        FSB_WP_Center[1] = 8.0; // Move Forward by 8.0
        FSB_WP_Center[2] = 3.25; // Move Backward by 4.0
        FSB_WP_Center[3] = 16.0; // strafe right by 12.0
        FSB_WP_Center[4] = 30.25; // Move forward by 30.0
        FSB_WP_Center[5] = 20.0; // strafe left by 18.0
        FSB_WP_Center[6] = 21.50; // Spin Clockwise by 21.5
        FSB_WP_Center[7] = 45.0; // Move Backward by 67
        FSB_WP_Center[8] = 22.0; // Move Backward by 67
        FSB_WP_Center[9] = 24.0; // strafe right by 20 //28.5
        FSB_WP_Center[10] = 7.0; // Backward by 8.5 inches
        FSB_WP_Center[11] = 22.5; // Strafe left by 29.0
        FSB_WP_Center[13] = 24.5; // Strafe Left by 20.0

        // FSB-Left Spike:
        FSB_WP_Left[0] = 7.0; // Strafe Right by 5
        FSB_WP_Left[1] = 8.0; // Move Forward by 6.0
        FSB_WP_Left[2] = 22.0; // Spin Anticlockwise by 22.0
        FSB_WP_Left[3] = 24.0; // Strafe Right by 24.0
        FSB_WP_Left[4] = 5.5; // Move Forward by 4.5
        FSB_WP_Left[5] = 3.25; // Move Backward by 3.0
        FSB_WP_Left[6] = 7.0; // Move Backward by 7.0
        FSB_WP_Left[7] = 20.0; // Strafe Right 18.0
        FSB_WP_Left[8] = 43.0; // Spin Anticlockwise by 44.0
        FSB_WP_Left[9] = 52.0; // Move Backward by 50
        FSB_WP_Left[10] = 36.0; // Move Backward by 36
        FSB_WP_Left[11] = 34.5; // Move right by 36

        // FSB-Right Spike:
        FSB_WP_Right[0] = 10.25; // Strafe Right by 10.25
        FSB_WP_Right[1] = 10.0; // Move Forward by 10.0
        FSB_WP_Right[2] = 7.0; // Move Forward by 7.0
        FSB_WP_Right[3] = 3.0; // Move Backward by 3.0
        FSB_WP_Right[4] = 11.5; // Strafe Left by 9.5
        FSB_WP_Right[5] = 33.0; // Move Forward by 33.0
        FSB_WP_Right[6] = 22.0; // Spin Clockwise by 22.0
        FSB_WP_Right[7] = 56.2; // Move Backward by 50
        FSB_WP_Right[8] = 21.0; // Move Backward by 36
        FSB_WP_Right[9] = 20.0; // Move right by 6

// RED:
        // BSR-Center Spike:
        BSR_WP_Center[0] = 16.5; // Move Forward by 16.5
        BSR_WP_Center[1] = 8.0; // Move Forward by 8.0
        BSR_WP_Center[2] = 3.0; // Move Backward by 3.0
        BSR_WP_Center[3] = 3.0; // Move Backward by 3.0
        BSR_WP_Center[4] = 24.0; // Spin Anticlockwise by 24.0
        BSR_WP_Center[5] = 30.0; // Move Backward by 30.0
        // Corner Park Constants:
        BSR_WP_Center[6] = 22.0; // Strafe Left by 22.0
        // BSR_WP_Center[7] = 5.0; // Move Backward by 5.0
        // Center Park Constants:
        BSR_WP_Center[8] = 24.0; // Strafe Right by 24.0
        // BSR_WP_Center[9] = 10.0; // Move Backward by 10.0

        // BSR-Left Spike:
        BSR_WP_Left[0] = 7.0; // Strafe Right by 7.0
        BSR_WP_Left[1] = 27.0; // Move Forward by 27.0
        BSR_WP_Left[2] = 22.0; // Spin Anticlockwise by 22.0
        BSR_WP_Left[3] = 6.5; // Move Forward by 6.5
        BSR_WP_Left[4] = 3.0; // Move Backward by 3.0
        BSR_WP_Left[5] = 27.0; // Move Backward by 27.0
        BSR_WP_Left[6] = 3.0; // Strafe Right by 2.0
        // Corner Park Constants:
        BSR_WP_Left[7] = 29.0; // Strafe Left by 29.0
        // BSR_WP_Left[8] = 5.0; // Move Backward by 5.0
        // Center Park Constants:
        BSR_WP_Left[9] = 15.0; // Strafe Right by 14.0
        BSR_WP_Left[10] = 10.0; // Move Backward by 10.0

        // BSR-Right Spike:
        BSR_WP_Right[0] = 9.5; // Strafe Right by 9.5
        BSR_WP_Right[1] = 10.0; // Move Forward by 10.0
        BSR_WP_Right[2] = 13.0; // Move Forward by 13.0
        BSR_WP_Right[3] = 3.0; // Move Backward by 3.0
        BSR_WP_Right[4] = 1.0; // Move Backward 1.0
        BSR_WP_Right[5] = 21.5; // Spin Anticlockwise by 21.5
        BSR_WP_Right[6] = 21.0; // Move Backward by 21.0
        // Corner Park Constants:
        BSR_WP_Right[7] = 16.0; // Strafe Left by 16.0
        // BSR_WP_Right[8] = 5.0; // Move Backward by 5.0
        // Center Park Constants:
        BSR_WP_Right[9] = 32.0; // Strafe Right by 32.0
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
        FSR_WP_Center[9] = 27.0; // strafe Left by 24 //28.5
        FSR_WP_Center[10] = 7.5; // Backward by 6.0
        FSR_WP_Center[11] = 20.5; // Strafe left by 22.5

        // FSR-Left Spike:
        FSR_WP_Left[0] = 10.0; // Move Forward by 10.0
        FSR_WP_Left[1] = 10.0; // Strafe Left by 10.25
        FSR_WP_Left[2] = 7.0; // Move Forward by 7.0
        FSR_WP_Left[3] = 3.0; // Move Backward by 3.0
        FSR_WP_Left[4] = 11.5; // Strafe Right by 11.5
        FSR_WP_Left[5] = 33.0; // Move Forward by 33.0
        FSR_WP_Left[6] = 22.0; // Spin Anticlockwise by 22.0
        FSR_WP_Left[7] = 56.2; // Move Backward by 56.2
        FSR_WP_Left[8] = 20.0; // Move Backward by 22.0
        FSR_WP_Left[9] = 17.0; // Strafe Left by 20.5

        // FSR-Right Spike:
        FSR_WP_Right[0] = 8.0; // Move Forward by 8.0
        FSR_WP_Right[1] = 7.0; // Strafe Left by 7.0
        FSR_WP_Right[2] = 21.5; // Spin Clockwise by 22.0
        FSR_WP_Right[3] = 26.0; // Strafe Left by 24.0
        FSR_WP_Right[4] = 8.0; // Move Forward by 9.0
        FSR_WP_Right[5] = 3.25; // Move Backward by 3.25
        FSR_WP_Right[6] = 9.0; // Move Backward by 9.0
        FSR_WP_Right[7] = 20.0; // Strafe Left 20.0
        FSR_WP_Right[8] = 44.5; // Spin Clockwise by 43.0
        FSR_WP_Right[9] = 51.0; // Move Backward by 50.0
        FSR_WP_Right[10] = 39.5; // Move Backward by 35.0
        FSR_WP_Right[11] = 33.5; // Strafe Left by 37.0
    }


}

