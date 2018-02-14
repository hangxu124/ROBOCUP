/**
* @file LibCodeRelease.h
*/

class LibCodeRelease : public LibraryBase
{
public:
  /** Constructor for initializing all members*/
  int maxDeviation;

  float minValidity;
  
  LibCodeRelease();

  void preProcess() override;

  void postProcess() override;
  
  
  bool between(float value, float min, float max);
    
  int timeSinceBallWasSeen();
  
 
  float angleToGoal;
  
  float angleToGoal_postl;
  float angleToGoal_postr;
  std::string strAng;
  
  float angleToOwnGoal;
    // often used functions, to keep the roles clean
  float distanceBetween(Vector2f vector1, Vector2f vector2);

  Vector2f robotToField(Vector2f vector); //converts coordinates from relative to the robot to absolute field coordinates
    
  Vector2f fieldToRobot(Vector2f vector);
    //function to sum vectors
    Vector2f vectorAdd(Vector2f vector1,Vector2f vector2);
    //function to avoid obstacles
    Vector2f AvoidObstacles(Vector2f target);
    
    // often used variables, to keep the code clean
    Vector2f centerOpponentGroundLine;    //the center of the opponents ground line
	Vector2f centerOwnGoalGroundLine; //the center of your own goal ground line
    Vector2f centerPenaltyAreaFieldCoordinates;
    Vector2f centerField ;
    Vector2f centerDefenseAreaFieldCoordinates;

	
    float goalWidth;
    float fieldWidth;
    float defenseRadius;
};