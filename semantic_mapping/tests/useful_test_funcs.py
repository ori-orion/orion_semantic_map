import orion_actions.srv;
import rospy;

def create_obs_instance(class_, x=0, y=0, z=0, batch_num=0, category="") -> orion_actions.srv.SOMAddObservationRequest:
    output = orion_actions.srv.SOMAddObservationRequest();
    output.adding.class_ = class_;
    output.adding.obj_position.position.x = x;
    output.adding.obj_position.position.y = y;
    output.adding.obj_position.position.z = z;
    output.adding.observation_batch_num = batch_num;
    output.adding.category = category;
    output.adding.observed_at = rospy.Time.now();
    
    return output;