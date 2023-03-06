import orion_actions.srv as act_srv
import rospy;

import time
import unittest

def create_obs_instance(class_, x=0, y=0, z=0, batch_num=0, category="") -> act_srv.SOMAddObservationRequest:
    output = act_srv.SOMAddObservationRequest();
    output.adding.class_ = class_;
    output.adding.obj_position.position.x = x;
    output.adding.obj_position.position.y = y;
    output.adding.obj_position.position.z = z;
    output.adding.observation_batch_num = batch_num;
    output.adding.category = category;
    output.adding.observed_at = rospy.Time.from_sec(time.time())

    return output;

def skipTest(reason):
    def decorator(test):
        print(f'SKIPPED: [{test.__name__}] {reason}')
        return unittest.skip(reason)(test)
    return decorator