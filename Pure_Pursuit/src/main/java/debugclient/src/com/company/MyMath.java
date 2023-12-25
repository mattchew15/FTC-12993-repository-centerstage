package debugclient.src.com.company;

public class MyMath {
    /**
     * Makes sure an angle is in the range of -180 to 180
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle){
        while (angle<-java.lang.Math.PI){
            angle += 2.0* java.lang.Math.PI;
        }
        while (angle> java.lang.Math.PI){
            angle -= 2.0* java.lang.Math.PI;
        }
        return angle;
    }


}
