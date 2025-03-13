package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.vision.VisionManager;

public class BannerLight {
    // Instantiated DigitalOutput objects
    private static DigitalOutput white;
    private static DigitalOutput tan;
    private static DigitalOutput gray;
    private static DigitalOutput black;

    private static BannerState bannerState;

    /** Enum managing color on banner light with each boolean  
     *  representing power in each colored output wire */
    public enum BannerState {
        NONE(false, false, false, false),
        GREEN(true, false, false, false),
        RED(false, false, false, true), //Red
        ORANGE(true, true, false, true), //Orange
        PINK_GREEN(false, true, true, true), //Worst Color Combo Ever (Pink w/ moving Green)
        FLASHING_GREEN(true, true, true, true); //Flashing Green
    
        public final boolean white;
        public final boolean tan;
        public final boolean gray;
        public final boolean black;
    
        private BannerState(boolean white, boolean tan, boolean gray, boolean black) {
            this.white = white;
            this.tan = tan;
            this.gray = gray;
            this.black = black;
        }
    }

    /** Initilizes the Banner Light by setting digital output channels */
    public static void init() {
        white = new DigitalOutput(1);
        tan = new DigitalOutput(2);
        gray = new DigitalOutput(3);  
        black = new DigitalOutput(4);
    }

    /**
     * Sets the state of the banner light
     *
     * @param state the desired state of the banner light
     */
    public static void setState(BannerState state){
        bannerState = state;
        white.set(state.white);
        tan.set(state.tan);
        gray.set(state.gray);
        black.set(state.black);
    }

    /** Sets the banner light color to green if an april  
     *  tag is seen and red if no april tags are seen. */
    public static void update(){
        if(VisionManager.hasTag()){
            setState(BannerState.GREEN);
        } else {
            setState(BannerState.RED);
        }
    }

}