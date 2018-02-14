/**
* @file LibCodeRelease.cpp
*/

#include "../LibraryBase.h"

namespace Behavior2015
{
  #include "LibCodeRelease.h"
  
  LibCodeRelease::LibCodeRelease():
    angleToGoal(0.f)
  //angleToCenter(0.f)
  {}
  
  void LibCodeRelease::preProcess()
  {
    angleToGoal = (theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    //Vector2f goalPos = (theFieldDimensions.xPosOpponentGoal,theFieldDimensions.yPosCenterGoal);
    strAng=angleToGoal;
    
    angleToGoal_postr = (theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentGroundline, 700.f)).angle();
	//Generate angles to goal post of rival for use in striker kick
	angleToGoal_postl = (theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentGroundline, -700.f)).angle();
    
    // own variables
    angleToOwnGoal = (theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();

	//angleToCenter = (theRobotPose.inverse() * Vector2f(0.f, 0.f)).angle();
	
    centerOpponentGroundLine = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
	centerOwnGoalGroundLine = Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f);
    centerPenaltyAreaFieldCoordinates = Vector2f((theFieldDimensions.xPosOwnGroundline + theFieldDimensions.xPosOwnPenaltyArea)/2, 0);
	centerField = Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosCenterGoal);
    goalWidth = (int)std::abs(theFieldDimensions.yPosLeftGoal - theFieldDimensions.yPosRightGoal)*(float)0.9;
    fieldWidth = (int)std::abs(theFieldDimensions.yPosLeftSideline - theFieldDimensions.yPosRightSideline);
	defenseRadius = 1500.f;
	centerDefenseAreaFieldCoordinates = Vector2f(theFieldDimensions.xPosOwnGroundline + defenseRadius, 0.f); //own goal is always at negative coordinates, so plus
    
    maxDeviation = 90;
    minValidity = 0.5f;
  }

  void LibCodeRelease::postProcess()
  {
  }
  
  int LibCodeRelease::timeSinceBallWasSeen()
  {
    return theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  }
  
  bool LibCodeRelease::between(float value, float min, float max)
  {
    return value >= min && value <= max;
  } 
  float LibCodeRelease::distanceBetween(Vector2f vector1, Vector2f vector2){
    Vector2f distanceVector = vector1 - vector2;
    
    return distanceVector.norm();
  }
  
  Vector2f LibCodeRelease::robotToField(Vector2f vector){
      return Transformation::robotToField(theRobotPose, vector);
  }
  
 
  
  Vector2f LibCodeRelease::fieldToRobot(Vector2f vector){
      return Transformation::fieldToRobot(theRobotPose, vector);
  }
  
   Vector2f LibCodeRelease::vectorAdd(Vector2f vector1,Vector2f vector2)
  {
      Vector2f res;
      res=vector1 + vector2;
      return res;
      
  }
  
  Vector2f LibCodeRelease::AvoidObstacles(Vector2f target) //returns a target position with some offset that makes the robot avoid obstacles (potential field method)
  {
      float max=1000;
      float magnitude=2;
      Vector2f nextpos=Vector2f::Zero();
      Vector2f move;
      int nbObstacles=theObstacleModel.obstacles.size();
      
      for(int i=0;i<nbObstacles;i++)
      {
          if(theObstacleModel.obstacles[i].type==Obstacle::someRobot || theObstacleModel.obstacles[i].type==Obstacle::opponent  //obstacles are referenced relative to the robot
          || theObstacleModel.obstacles[i].type==Obstacle::teammate)
          {
              /*ANNOTATION("obstacleX",theObstacleModel.obstacles[i].center.x()); 
            ANNOTATION("obstacleY",theObstacleModel.obstacles[i].center.y());*/
              move=-theObstacleModel.obstacles[i].center; //get vector that points away from obstacle
              float distance=move.norm();         
              move=move/distance; //unit vector         
              if(distance<max) // only evade if object too close
              {
                  float scale=max-distance; //size of evasive manouvre depends on distance to object (closer = larger evade)
                  move=move*scale*magnitude;
                  nextpos+=move;
                  }
          }
        }
        return target + nextpos;
    }
 
}