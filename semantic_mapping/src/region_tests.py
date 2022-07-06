#!/usr/bin/env python3

import rospy

import geometry_msgs.msg;

import orion_actions.srv
import orion_actions.msg

import std_srvs.srv;

import math;

import random;

def create_obj_instance(class_, x=0, y=0, z=0) -> orion_actions.srv.SOMAddObservationRequest:
    output = orion_actions.srv.SOMAddObjectRequest();
    output.adding.class_ = class_;
    output.adding.obj_position.position.x = x;
    output.adding.obj_position.position.y = y;
    output.adding.obj_position.position.z = z;
    output.adding.num_observations = 100;

    output.adding.size.x = 0.1;
    output.adding.size.y = 0.1;
    output.adding.size.z = 0.1;
    
    return output;



def test_regions():
    # Adding regions (if applicable).
    region_input = rospy.ServiceProxy('/som/object_regions/input', orion_actions.srv.SOMAddRegion);
    region_basic_query = rospy.ServiceProxy('/som/object_regions/basic_query', orion_actions.srv.SOMQueryRegions);
    region_region_query = rospy.ServiceProxy('/som/object_regions/region_query', orion_actions.srv.SOMRegionQuery);
    object_input = rospy.ServiceProxy('/som/objects/input', orion_actions.srv.SOMAddObject);



    print("Deleting all again...");
    region_delete_srv = rospy.ServiceProxy('/som/object_regions/delete_entries', std_srvs.srv.Empty);
    region_delete_srv(std_srvs.srv.EmptyRequest());
    region_query_2 = orion_actions.srv.SOMQueryRegionsRequest();
    region_query_2_response = region_basic_query(region_query_2);
    # assert(len(region_query_2_response.returns) == 0);


    # Adding an origin marker.
    object_input(create_obj_instance("origin"));


    region_name_1 = "test_reg_1";
    region_name_2 = "test_reg_2";
    region_name_3 = "test_reg_3";

    region_query_1 = orion_actions.srv.SOMQueryRegionsRequest();
    region_query_1.query.name = region_name_1;
    region_query_1_response = region_basic_query(region_query_1);
    if len(region_query_1_response.returns) == 0:
        region_to_add = orion_actions.msg.SOMBoxRegion();
        region_to_add.name = region_name_1;
        region_to_add.dimension.x = 1;
        region_to_add.dimension.y = 1;
        region_to_add.dimension.z = 1;
        region_to_add.corner_loc.translation.x = 0;
        region_to_add.corner_loc.translation.y = 1;
        region_to_add.corner_loc.translation.z = 0;
        region_to_add.corner_loc.rotation.x = 0.0998;
        region_to_add.corner_loc.rotation.y = 0;
        region_to_add.corner_loc.rotation.z = 0;
        region_to_add.corner_loc.rotation.w = 0.995;
        region_input(orion_actions.srv.SOMAddRegionRequest(region_to_add));

        region_to_add.name = region_name_2;
        region_to_add.corner_loc.translation.x = 1.2;
        region_to_add.corner_loc.translation.z = 1;
        region_to_add.corner_loc.rotation.x = 0;
        region_to_add.corner_loc.rotation.z = 0.955;
        region_to_add.corner_loc.rotation.w = 0.296;
        region_to_add.dimension.x = 0.5;
        region_input(orion_actions.srv.SOMAddRegionRequest(region_to_add));

        region_to_add.name = region_name_3;
        region_to_add.corner_loc.translation.x = 4;
        region_to_add.corner_loc.rotation.x = 1;
        region_to_add.corner_loc.rotation.z = 0;
        region_to_add.corner_loc.rotation.w = 0;
        region_to_add.dimension.z = 0.2;
        region_input(orion_actions.srv.SOMAddRegionRequest(region_to_add));

        region_query_1_response:orion_actions.srv.SOMQueryRegionsResponse = region_basic_query(region_query_1);
        print(region_query_1_response);

        # assert(len(region_query_1_response.returns) == 1);
    region_q1_ret:orion_actions.msg.SOMBoxRegion = region_query_1_response.returns[0];
    # assert(region_q1_ret.SESSION_NUM == -1);        # We want this to be a prior.
    # assert(region_q1_ret.name == region_name_1);


    adding_request = create_obj_instance("Test_1", 0.5, 1.5, 0.5);
    adding_request.adding.size.x = 0.1;
    adding_request.adding.size.y = 0.1;
    adding_request.adding.size.z = 0.1;
    
    object_input(adding_request);
    

    # name_prefix:str = "obj_";
    # for i in range(1000):
    #     rand_x:float = random.uniform(-1,2);
    #     rand_y:float = random.uniform(0, 3);
    #     rand_z:float = random.uniform(-1, 2);
    #     adding_request = create_obj_instance(name_prefix + str(i), rand_x, rand_y, rand_z);
    #     adding_request.adding.size.x = 0.1;
    #     adding_request.adding.size.y = 0.1;
    #     adding_request.adding.size.z = 0.1;

    #     object_input(adding_request);
        
    region_request = orion_actions.srv.SOMRegionQueryRequest();
    region_request.region_name = region_name_1;
    response:orion_actions.srv.SOMRegionQueryResponse = region_region_query(region_request);
    print("Length of output =", len(response.returns));


if __name__ == '__main__':
    test_regions();