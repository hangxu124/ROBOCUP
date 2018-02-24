
option(Striker)
{
    ANNOTATION("robotPose deviation: ", theRobotPose.deviation);
    ANNOTATION("robotPose validity: ", theRobotPose.validity);
    
    common_transition{
        if (theRobotPose.deviation > libCodeRelease.maxDeviation || theRobotPose.validity < libCodeRelease.minValidity)
            goto localizeSelf;
    }
    
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
        goto walkToBall;
    }
    action
    {
      Vector2f target_2f = theBallModel.estimate.position;
      Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
      SetHeadTargetOnGround(target_3f);
      //theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }

  state(walkToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) > 10_deg)
        goto turnToBall;
      if(theBallModel.estimate.position.norm() < 700.f)
        goto alignToPen;
    }
    action
    {
      Vector2f target_2f = theBallModel.estimate.position;
      Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
      SetHeadTargetOnGround(target_3f);
      //theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
    }
  }

  state(alignToPen)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(std::abs(libCodeRelease.angleToPen) < 8_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBall;
    }
    action
    {
      Vector2f target_2f = theBallModel.estimate.position;
      Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
      SetHeadTargetOnGround(target_3f);
      //theHeadControlMode = HeadControl::lookForward;
      //Annotation(libCodeRelease.strAng);
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), 
                   Pose2f(libCodeRelease.angleToPen, theBallModel.estimate.position.x() - 600.f, theBallModel.estimate.position.y()));
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(libCodeRelease.between(theBallModel.estimate.position.y(), 0.f, 50.f)
          && libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
          && std::abs(libCodeRelease.angleToPen) < 3_deg)
        goto walkToPen;
    }
    action
    {  // Annotation(libCodeRelease.strAng);
      //theHeadControlMode = HeadControl::lookForward;
      //Ball position estimate as 2d vector		
      Vector2f target_2f = theBallModel.estimate.position;
      //Add z component = 0
      Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		//track ball with head
      SetHeadTargetOnGround(target_3f);
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), 
                   Pose2f(libCodeRelease.angleToPen, 
                          theBallModel.estimate.position.x() - 150.f, 
                          theBallModel.estimate.position.y() -30.f));
    }
  }

  state(walkToPen)
  {
      transition
      {
      if(libCodeRelease.timeSinceBallWasSeen() > 400)
        goto searchForBall;
      if (std::abs(theBallModel.estimate.position.angle())>15_deg)
        goto alignBehindBall;
      if (libCodeRelease.distanceBetween(Vector2f(theFieldDimensions.xPosOpponentGoal,theFieldDimensions.yPosCenterGoal),theRobotPose.translation)<1200)
        goto kick;
          
      }
      action
      {
      //Annotation("Ano:",libCodeRelease.distanceBetween(Vector2f(theFieldDimensions.xPosOpponentGoal,theFieldDimensions.yPosCenterGoal),theRobotPose.translation));
      Vector2f target_2f = theBallModel.estimate.position;
      Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
      SetHeadTargetOnGround(target_3f); 
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), 
                   Pose2f(libCodeRelease.angleToPen, 
                          -theFieldDimensions.xPosOwnPenaltyMark , 
                          0.f));      
      }
  }
  state(kick)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      Annotation("Alive and Kickin'");
        Vector2f target_2f = theBallModel.estimate.position;
		//Add z component = 0
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		//track ball with head
		SetHeadTargetOnGround(target_3f);
      //theHeadControlMode = HeadControl::lookForward;
      InWalkKick(WalkRequest::left, 
                 Pose2f(libCodeRelease.angleToGoal, 
                 theBallModel.estimate.position.x() - 160.f, 
                 theBallModel.estimate.position.y() - 55.f));
    }
  }
  
  state(searchForBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() < 300)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight; 
      headControlTurnAngle = float(0.5f);   
      WalkAtSpeedPercentage(Pose2f(0.3f, 0.05f, 0.05f));
    }
  }
  
  state(localizeSelf)
    {
        transition
        {
            if(theRobotPose.validity < libCodeRelease.maxDeviation && theRobotPose.validity > libCodeRelease.minValidity)
                goto turnToBall;
            
        }
        action
        {
			
            if (state_time < 5000){				//look left right
				Stand();
                theHeadControlMode = HeadControl::lookLeftAndRight; 
				headControlTurnAngle = float(0.5f);
				headControlTiltAngle = float(0.0f);
            }
		    else if (state_time< 12000){			//look up down
		        theHeadControlMode = HeadControl::lookLeftAndRight; 
				headControlTurnAngle = float(0.5f);
				headControlTiltAngle = float(0.0f);
		        WalkAtSpeedPercentage(Pose2f(0.3f, 0.05f, 0.05f));
		    }
			else if(state_time < 20000){
				Stand();
 				theHeadControlMode = HeadControl::lookLeftAndRight; 
				headControlTurnAngle = float(0.5f);
				headControlTiltAngle = float(0.0f);
            }
			else {
		            WalkAtSpeedPercentage(Pose2f(0.3f, 0.05f, 0.05f));
		            theHeadControlMode = HeadControl::lookLeftAndRight; 
					headControlTurnAngle = float(0.5f);
					headControlTiltAngle = float(0.0f);
	        }
        }
    }
}