


namespace teleop_assist
{

    
    //Project pose to a given time
    projecetd_pose = projectPose(pose, time);

    //Modify velocity command to prevent a future collision in n time 
    vel = computeCollisionFreeVelocityCmd(vel);

    //Checks to if teleop is ready to take over 
    startTeleopAssist(msg);


    //Checks to cancel teleop assist, make sure you can exit in a valid location
    cancelTeleopAssist(joymsg);
}