package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The CoralVisualizer provides a visual representation of the reef for the operator to select a coral
 */
public class CoralVisualizer {
    private static Mechanism2d mechanism2d = new Mechanism2d(5, 13);
    private static MechanismLigament2d[][] ligamentMatrix;
    private static int level = 0;
    private static boolean isRight = true;

    /**
     * Gets the Mechanism2d instance representing the visual coral structure.
     *
     * @return the Mechanism2d instance
     */
    public static Mechanism2d getMechanism2d(){
        return mechanism2d;
    }

    /**
     * Initializes the coral visualization by setting the background color and creating
     * a matrix of MechanismLigament2d objects representing reefs.
     */
    public static void init(){
        mechanism2d.setBackgroundColor(new Color8Bit(Color.kWhiteSmoke));
        ligamentMatrix = new MechanismLigament2d[4][2];
        double angle = 80;

        for(int row = 0; row < ligamentMatrix.length; row++){
            for(int col = 0; col < ligamentMatrix[0].length; col++){
                MechanismRoot2d root = mechanism2d.getRoot(row+", "+col+" root", 1.5+col*2, 1+row*3.5);
                
                ligamentMatrix[row][col] = root.append(new MechanismLigament2d(row+", "+col+" lig", 1, col == 1 ?  angle  : 180-angle));
                ligamentMatrix[row][col].setColor(new Color8Bit(Color.kDarkGray));
            }
        }
    }

    /**
     * Selects a coral segment based on the current isRight value and level.
     *
     * @param isRight whether the right coral segment should be selected
     */
    public static void setSelectedCoral(boolean isRight){
        setSelectedCoral(isRight, level);
    }

    /**
     * Selects a coral segment based on the given level while keeping the current isRight value.
     *
     * @param level the level of the coral to be selected
     */
    public static void setSelectedCoral(int level){
        setSelectedCoral(isRight, level);
    }

    /**
     * Selects a coral segment based on the specified level and position.
     *
     * @param isRight whether the right coral segment should be selected
     * @param level   the level of the coral to be selected
     * @throws IllegalArgumentException if the level is less than 1 or greater than 4
     */
    public static void setSelectedCoral(boolean isRight, int level){
        if(level < 1) throw new IllegalArgumentException("Coral Level cannot be less than 1");
        if(level > 4) throw new IllegalArgumentException("Coral Level cannot be greater than 4");
        
        // Reset all ligaments to default color
        for(int row = 0; row < ligamentMatrix.length; row++){
            for(int col = 0; col < ligamentMatrix[0].length; col++){
                MechanismLigament2d ligament2d = ligamentMatrix[row][col];
                ligament2d.setColor(new Color8Bit(Color.kDarkGray));
            }
        }

        // Highlight the selected coral segment
        ligamentMatrix[level-1][isRight ? 1 : 0].setColor(new Color8Bit(Color.kDarkGreen));
        CoralVisualizer.isRight = isRight;
        CoralVisualizer.level = level;
    }
}
