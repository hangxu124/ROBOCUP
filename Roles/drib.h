 
Vector2f target = Vector2f(3500.f, 0.f);
float dribblingMinBallToTargetDist = 165.f;
float dribblingMaxBallToTargetDist = 650.f;
float dribblingMinBallOffset = 100.f;
float dribblingMaxBallOffset = 100.f;
float dribblingTargetAccuracy =100.f;
	
option(Striker)
{
    // Debug
    Annotation("robotPose deviation: ", theRobotPose.deviation);
    Annotation("robotPose validity: ", theRobotPose.validity);

    common_transition{
        if (theRobotPose.deviation > libCodeRelease.maxDeviation || theRobotPose.validity < libCodeRelease.minValidity)
            goto localizeSelf;
    }
    initial_state(start)
    {
        transition
        {
            if(state_time > 10000)
                goto turnToBall;
        }
        action
        { // try to find your position
            theHeadControlMode = HeadControl::lookLeftAndRight; headControlTurnAngle = (float) 0.3;
            //WalkAtSpeedPercentage(Pose2f(0.5f, 1.f, 1.f)); headControlTurnAngle = float(0.5);// turn in circles to locate yourself
            //Stand();
        }
    }

	state(turnToBall)
		{
			transition
			{
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
			if(std::abs(theBallModel.estimate.position.angle()) > 10_deg)
                goto turnToBall;
            if(theBallModel.estimate.position.norm() < 700.f)
                goto alignToGoal;
        }
        action
        {
			//face ball
			Vector2f target_2f = theBallModel.estimate.position;
			Vector3f target_3f(target_2f[0], target_2f[1], 0.f); 
			SetHeadTargetOnGround(target_3f);
            //theHeadControlMode = HeadControl::lookLeftAndRight; 
			//headControlTurnAngle = float(0.5);
			//WalkAtSpeed(Pose2f(100.f, 100.f, 0.f));
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position); //50.f, 50.f, 50.f
        }
			
		
	}
	
	state(alignToGoal)
    {
        transition
        {
			
            if(std::abs(libCodeRelease.angleToGoal) < 5_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
                goto alignBehindBall;
        }
        action
        {
			
			// Judge whether the Robot between Ball and Goal, if yes, we should set another proper behavior to 'alignToGoal' to avoid back kicking to own goal. Otherwise, just use normal 'alignToGoal'
			Vector2f target_2f = theBallModel.estimate.position;
			Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
			SetHeadTargetOnGround(target_3f);
			
			Vector2f BallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);
			Vector2f RobotFieldCoordinates = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
			
			// the symmetric position in reference to Ball in field coordinate the robot needs to walk to
			Vector2f PointWalkToVertical= Vector2f(theFieldDimensions.xPosOwnGroundline,RobotFieldCoordinates.y());
			Vector2f PointWalkToHorizon1= Vector2f(RobotFieldCoordinates.x(),theFieldDimensions.yPosRightSideline);
			Vector2f PointWalkToHorizon2= Vector2f(RobotFieldCoordinates.x(),theFieldDimensions.yPosLeftSideline);
	

            // Vector2f(RobotFieldCoordinates.x()-2*(RobotFieldCoordinates.x() - BallFieldCoordinates.x()), RobotFieldCoordinates.y());
			// in Robot coordinate
			Vector2f PointWalkToRobotCoordinate=libCodeRelease.fieldToRobot(PointWalkToVertical);
			//Vector2f PointWalkToRobotCoordinateHorizon1=libCodeRelease.fieldToRobot(PointWalkToHorizon1);
			//Vector2f PointWalkToRobotCoordinateHorizon2=libCodeRelease.fieldToRobot(PointWalkToHorizon2);
			
			// Rotate angle until facing own goal side
			float angleToRotationVertical=(theRobotPose.inverse()*PointWalkToVertical).angle();
			float angleToRotationHorizon1=(theRobotPose.inverse()*PointWalkToHorizon1).angle();
			float angleToRotationHorizon2=(theRobotPose.inverse()*PointWalkToHorizon2).angle();

			if(libCodeRelease.between(RobotFieldCoordinates.x(), BallFieldCoordinates.x()-150.f,theFieldDimensions.xPosOpponentGroundline))
			{
				if (abs(RobotFieldCoordinates.y()- BallFieldCoordinates.y())< 100.f && (RobotFieldCoordinates.y() > BallFieldCoordinates.y()))
				{
					WalkToTarget(Pose2f(50.f, 50.f, 50.f),
							 Pose2f(angleToRotationHorizon1,
									PointWalkToHorizon1.x(),
									PointWalkToHorizon1.y()));
				}
				
			    else if (abs(RobotFieldCoordinates.y()-BallFieldCoordinates.y())< 100.f && (RobotFieldCoordinates.y() < BallFieldCoordinates.y()))
				{
					WalkToTarget(Pose2f(50.f, 50.f, 50.f),
							 Pose2f(angleToRotationHorizon2,
									PointWalkToHorizon2.x(),
									PointWalkToHorizon2.y()));
				}
				
				else
				{
					WalkToTarget(Pose2f(50.f, 50.f, 50.f),
							 Pose2f(angleToRotationVertical,
									PointWalkToRobotCoordinate.x(),
									PointWalkToRobotCoordinate.y()));
				}
				
			}
				
			else
				{
					//theHeadControlMode = HeadControl::lookLeftAndRight;
					//headControlTurnAngle = float(0.5);
					WalkToTarget(Pose2f(100.f, 100.f, 100.f),
									Pose2f(libCodeRelease.angleToGoal,
									theBallModel.estimate.position.x() - 400.f,
									theBallModel.estimate.position.y()));
				}
		}
		
	}
            

    state(alignBehindBall)
    {
        transition
        {
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f) &&
               libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f) &&
               (std::abs(libCodeRelease.angleToGoal) < 3_deg ||
			   std::abs(libCodeRelease.angleToGoal_postr) < 3_deg||
			   std::abs(libCodeRelease.angleToGoal_postl) < 3_deg))
                goto dribbleBall;
        }
        action
        {
		//Ball position estimate as 2d vector		
			Vector2f target_2f = theBallModel.estimate.position;
			//Add z component = 0
			Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
			//track ball with head
			SetHeadTargetOnGround(target_3f);

			//Compute own position in field coordinates
			Vector2f ownPosFieldCoordinates = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
			//Compute distance between own position in the field and center point of rival goal
			float distance = libCodeRelease.distanceBetween(ownPosFieldCoordinates,
																libCodeRelease.centerOpponentGroundLine);

					//if distance to goal center is less than 1600 mm and robot is on the left side of the field
			if(distance<1600.f && distance >800.f && ownPosFieldCoordinates.y()> 20.f){
				WalkToTarget(Pose2f(80.f, 80.f, 80.f),Pose2f(libCodeRelease.angleToGoal_postl,theBallModel.estimate.position.x() - 150.f,theBallModel.estimate.position.y() - 30.f));
			}
			//if distance to goal center is less than 1600 mm and robot is on the right side of the field		
			else if(distance<1600.f && distance >800.f && ownPosFieldCoordinates.y()< -20.f) 
				{
					WalkToTarget(Pose2f(80.f, 80.f, 80.f),Pose2f(libCodeRelease.angleToGoal_postr,theBallModel.estimate.position.x() - 150.f,theBallModel.estimate.position.y() - 30.f));
				}
			else  
			{
				WalkToTarget(Pose2f(80.f, 80.f, 80.f),Pose2f(libCodeRelease.angleToGoal,theBallModel.estimate.position.x() - 150.f,theBallModel.estimate.position.y() - 30.f));
			}
        }
    }
	
	
	state(dribbleBall)
	{
		transition
		{
			if(libCodeRelease.timeSinceBallWasSeen() > 7000)
				goto searchForBall;
				
			/*if(std::abs(theBallModel.estimate.position.angle()) > 20_deg)
                goto turnToBall;*/
			//if(libCodeRelease.between(libCodeRelease.robotToField(Vector2f(0.f, 0.f)).x(), libCodeRelease.robotToField(theBallModel.estimate.position).x()-100.f,theFieldDimensions.xPosOpponentGroundline))	
				//goto alignToGoal;
				
			if(state_time > 10 && action_done)
				{  Annotation("dribAccuracy:",dribblingTargetAccuracy);
					if((target- libCodeRelease.robotToField(theBallModel.estimate.position)).norm() > dribblingTargetAccuracy) 
						goto dribbleBall;
					else
						goto kick;	
				}
		}
		action
		{
			Vector2f target_2f = theBallModel.estimate.position;
			Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
			SetHeadTargetOnGround(target_3f);
			
						// calculate the target shifted position for dribbling
			Vector2f BallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);
			
			float angleToTarget=(theRobotPose.inverse()*target).angle();

			float dribblingOffset;
			
			//target, dribblingMinBallToTargetDist, dribblingMaxBallToTargetDist, dribblingMinBallOffset, dribblingMaxBallOffset, dribblingTargetAccuracy are defined in DribblingParameters.h and set in file DribblingParameters.cfg 
			float distanceToTargetDribble = libCodeRelease.distanceBetween(target,BallFieldCoordinates); //distanceToTargetDribble will also defined in DribblingParameters.h
			
			if (distanceToTargetDribble < dribblingMinBallToTargetDist)
				{
					dribblingOffset = dribblingMinBallOffset;
				}
			else if (dribblingMinBallToTargetDist< distanceToTargetDribble < dribblingMaxBallToTargetDist)
				{
					dribblingOffset = (dribblingMaxBallOffset-dribblingMinBallOffset)/(dribblingMaxBallToTargetDist-dribblingMinBallToTargetDist)*distanceToTargetDribble + dribblingMinBallOffset - (dribblingMaxBallOffset-dribblingMinBallOffset)/(dribblingMaxBallToTargetDist-dribblingMinBallToTargetDist)*dribblingMinBallToTargetDist;
				}				
			else
				{
					dribblingOffset = dribblingMaxBallOffset;
				}
			
			if (target.y() < BallFieldCoordinates.y())				
				{
					
					WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(angleToTarget, (target.x()- BallFieldCoordinates.x())*dribblingOffset/distanceToTargetDribble + BallFieldCoordinates.x(), BallFieldCoordinates.y()-(BallFieldCoordinates.y()- target.y())*dribblingOffset/distanceToTargetDribble));
					
				}
			else
				{
					WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(angleToTarget,(target.x()- BallFieldCoordinates.x())*dribblingOffset/distanceToTargetDribble + BallFieldCoordinates.x(), (target.y()- BallFieldCoordinates.y())*dribblingOffset/distanceToTargetDribble + BallFieldCoordinates.y()));
				
				}
			
		}	
	}	
		
	state(kick)
    {
        transition
        {
            if(state_time > 3000 || (state_time > 10 && action_done))
                goto turnToBall;
        }
        action
        {
            Annotation("Alive and Kickin'");
            //theHeadControlMode = HeadControl::lookForward;
			Vector2f target_2f = theBallModel.estimate.position;
			Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
			SetHeadTargetOnGround(target_3f);
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
            WalkAtSpeedPercentage(Pose2f(0.3f, 0.05f, 0.05f));
            theHeadControlMode = HeadControl::lookLeftAndRight;
	    	headControlTurnAngle = float(0.5f);
			//headControlTiltAngle = float(0.5f);
			/*if(state_time > 1500)
				{
				Vector2f centerPenaltyAreaRobotCoordinates = libCodeRelease.fieldToRobot(
                libCodeRelease.centerPenaltyAreaFieldCoordinates);          // convert to relative coordinates
            	Pose2f target = Pose2f(0.0f, centerPenaltyAreaRobotCoordinates); 
           		WalkToTarget(Pose2f(100.f, 100.f, 100.f), target);	
				
				}*/
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