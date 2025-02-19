package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.subsystems.ScoringManager.TransitoryState;

/**
 * The CoralVisualizer provides a visual representation of the reef for the operator to select a coral
 */
public class CoralVisualizer {
    private static Mechanism2d mechanism2d = new Mechanism2d(5/Constants.METERSTOINCHES, 13/Constants.METERSTOINCHES);
    private static MechanismLigament2d[][] ligamentMatrix;

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
            for(int col = 0; col < ligamentMatrix[row].length; col++){
                MechanismRoot2d root = mechanism2d.getRoot("root_" + row + "_" + col, 1.5+col*2, 1+row*3.5);
                
                ligamentMatrix[row][col] = root.append(new MechanismLigament2d("ligament_" + row + "_" + col, 1, col == 1 ?  angle  : 180-angle));
                ligamentMatrix[row][col].setColor(new Color8Bit(Color.kDarkGray));
            }
        }

        ScoringPosition position = ScoringManager.getScoringLevel();
        setSelectedCoral(ScoringManager.isPickingRightCoral(), getLevelId(position), true);
    }

    /**
     * Gets the coral level from a scoringPosition
     * @param scoringPosition The current socring position
     * @return An int of the coral level
     */
    private static int getLevelId(ScoringPosition scoringPosition){
        if(scoringPosition == ScoringPosition.L1) return 1;
        if(scoringPosition == ScoringPosition.L2) return 2;
        if(scoringPosition == ScoringPosition.L3) return 3;
        if(scoringPosition == ScoringPosition.L4) return 4;
        return -1;
    }

    /**
     * Selects a coral segment based on the specified level and position. If level is less then 1, no level is selected
     *
     * @param isRight whether the right coral segment should be selected
     * @param level the level of the coral to be selected
     * @param transitory whether the elvator is moving to the correct position
     * @throws IllegalArgumentException if the level is greater than 4
     */
    private static void setSelectedCoral(boolean isRight, int level, boolean transitory){
        if(level > 4) throw new IllegalArgumentException("Coral Level cannot be greater than 4");
        
        // Reset all ligaments to default color
        for(int row = 0; row < ligamentMatrix.length; row++){
            for(int col = 0; col < ligamentMatrix[0].length; col++){
                MechanismLigament2d ligament2d = ligamentMatrix[row][col];
                ligament2d.setColor(new Color8Bit(Color.kDarkGray));
            }
        }

        // No Coral Selected
        if(level < 1) return;

        // Highlight the selected coral segment
        ligamentMatrix[level-1][isRight ? 1 : 0].setColor(transitory ? new Color8Bit(Color.kGold) : new Color8Bit(Color.kDarkGreen));
    }

    /**
     * Updates the Coral Visualizer view based of the state of ScoringManager.
     */
    public static void updateView(){
        setSelectedCoral(
            ScoringManager.isPickingRightCoral(), 
            getLevelId(ScoringManager.getScoringLevel()), 
            ScoringManager.transitoryState != TransitoryState.FINISHED
        );
    }
}
