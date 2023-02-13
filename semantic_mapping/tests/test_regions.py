#!/usr/bin/env python3

import rospy

import orion_actions.srv
import orion_actions.msg

import std_srvs.srv;

import unittest
from typing import List


def create_region(name: str, position: List[int], size: List[int] = None, rotation: List[int]=None) -> orion_actions.msg.SOMBoxRegion:
    """Create a SOMBoxRegion with the given attributes.
    
    INPUT:
    - name -- the name of the region
    - position -- a list containing position coordinates (x, y, z)
    - size -- a list containing the size of the region (x, y, z). If None, will be set to (1, 1, 1)
    - rotation -- a list containing the rotation of the region (x, y, z, w). If None, will be set to (0, 0, 0, 1)
    """
    if size is None:
        size = [1, 1, 1]
    
    if rotation is None:
        rotation = [0, 0, 0, 1]

    region_to_add = orion_actions.msg.SOMBoxRegion()
    region_to_add.name = name
    region_to_add.dimension.x = size[0]
    region_to_add.dimension.y = size[1]
    region_to_add.dimension.z = size[2]
    region_to_add.corner_loc.translation.x = position[0]
    region_to_add.corner_loc.translation.y = position[1]
    region_to_add.corner_loc.translation.z = position[2]
    region_to_add.corner_loc.rotation.x = rotation[0]
    region_to_add.corner_loc.rotation.y = rotation[1]
    region_to_add.corner_loc.rotation.z = rotation[2]
    region_to_add.corner_loc.rotation.w = rotation[3]

    return region_to_add


def create_object_instance(class_: str, x: float=0, y: float=0, z: float=0, category: str=None) -> orion_actions.srv.SOMAddObservationRequest:
    """Creeate a SOMAddObservationRequest with the given attributes. The size is always (0.1, 0.1, 0.1) 
    and the number of observations is set to 100.
    
    INPUT:
    - class_ -- the class_ attribute of the object
    - x, y, z -- the position of the object, they are all zero by default
    - category -- teh category attribute of the object, it is None by default
    """
    output = orion_actions.srv.SOMAddObjectRequest()
    output.adding.category = category
    output.adding.class_ = class_
    output.adding.obj_position.position.x = x
    output.adding.obj_position.position.y = y
    output.adding.obj_position.position.z = z
    output.adding.num_observations = 100
    output.adding.size.x = 0.1
    output.adding.size.y = 0.1
    output.adding.size.z = 0.1
    
    return output


class TestRegionManager(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.region_input = rospy.ServiceProxy('/som/object_regions/input', orion_actions.srv.SOMAddRegion)
        cls.region_basic_query = rospy.ServiceProxy('/som/object_regions/basic_query', orion_actions.srv.SOMQueryRegions)
        cls.region_region_query = rospy.ServiceProxy('/som/object_regions/region_query', orion_actions.srv.SOMRegionQuery)
        cls.object_input = rospy.ServiceProxy('/som/objects/input', orion_actions.srv.SOMAddObject)
        cls.region_delete_srv = rospy.ServiceProxy('/som/object_regions/delete_entries', std_srvs.srv.Empty)

        # Get the regions that are currently in the database, so they can be restored
        response: orion_actions.srv.SOMQueryRegionsResponse = cls.region_basic_query(orion_actions.srv.SOMQueryRegionsRequest())
        cls.stored_regions: List[orion_actions.msg.SOMBoxRegion] = response.returns

    @classmethod
    def tearDownClass(cls):
        """After all tests have run, restore the regions that were present before running the tests."""
        cls.region_delete_srv(std_srvs.srv.EmptyRequest())
        for region in cls.stored_regions:
            cls.region_input(orion_actions.srv.SOMAddRegionRequest(region))

    def setUp(self):
        """Before running each test, delete all regions that may have been created previously."""
        self.region_delete_srv(std_srvs.srv.EmptyRequest())


    def test_querying_regions_return_only_objects_inside(self):
        """When querying in a region, objects outside of it should not be returned."""
        region_to_add = create_region('Test_Region1', [1, 1, 0])
        self.region_input(orion_actions.srv.SOMAddRegionRequest(region_to_add))

        obj_category = 'correct_objs'
        obj_inside = create_object_instance('obj_inside', 1.5, 1.5, 0.5, obj_category)
        self.object_input(obj_inside)
        self.object_input(create_object_instance('obj_outside', category=obj_category)) # Add object outside the region

        region_request = orion_actions.srv.SOMRegionQueryRequest()
        region_request.region_name = region_to_add.name
        region_request.query.category = obj_category
        response: orion_actions.srv.SOMRegionQueryResponse = self.region_region_query(region_request)

        self.assertEqual(len(response.returns), 1, 'Only one object should be inside the region.')

        resp_obj = response.returns[0]
        self.assertEqual(resp_obj.class_, obj_inside.adding.class_, 'Object returned is not the one positioned inside the region.')


    def test_querying_multiple_regions_returns_all_objects_inside(self):
        """If a region is composed of multiple separate subregions, objects inside all of the
            subregions should be returned.
        """
        region_name = 'Test_Multiple_Regions'
        region_to_add_1 = create_region(region_name, [1, 1, 0])
        region_to_add_2 = create_region(region_name, [-1, -1, 0])
        self.region_input(orion_actions.srv.SOMAddRegionRequest(region_to_add_1))
        self.region_input(orion_actions.srv.SOMAddRegionRequest(region_to_add_2))

        obj_category = 'multiple_regions_objs'
        inside_objs = [('obj_inside_1', [1.5, 1.5, 0.5]), 
                       ('obj_inside_2', [-0.5, -0.5, 0.5]), 
                       ('obj_inside_3', [1.75, 1.75, 0.25])]
        outside_objs = [('obj_outside_1', [0, 0.1, 0]), 
                        ('obj_outside_2', [10, 5, 5]), 
                        ('obj_outside_3', [-1.1, -1.1, 1])]
        for (name, pos) in inside_objs + outside_objs:
            obj = create_object_instance(name, pos[0], pos[1], pos[2], obj_category)
            self.object_input(obj)

        region_request = orion_actions.srv.SOMRegionQueryRequest()
        region_request.region_name = region_name
        region_request.query.category = obj_category
        response: orion_actions.srv.SOMRegionQueryResponse = self.region_region_query(region_request)

        self.assertEqual(len(response.returns), len(inside_objs), 'Number of objects returned is not correct.')

        inside_obj_names = {name for (name, _) in inside_objs}
        response_names = {obj.class_ for obj in response.returns}
        self.assertEqual(response_names, inside_obj_names, 'Objects returned are not the ones inside the regions.')


    def test_only_objects_inside_queried_region_are_returned(self):
        """If multiple different regions are present, the query should only return objects inside
            the region that was queried.
        """
        region1 = create_region('Test_Region_Queried_1', [1, 1, 0])
        self.region_input(orion_actions.srv.SOMAddRegionRequest(region1))
        region2 = create_region('Test_Region_Queried_2', [-1, -1, 0])
        self.region_input(orion_actions.srv.SOMAddRegionRequest(region2))

        obj_category = 'only_selected_region_objs'
        # This object is inside the region that will be queried
        obj_inside = create_object_instance('FirstObj', 1.5, 1.5, 0.5, obj_category)
        self.object_input(obj_inside)
        # This object is inside the region that will NOT be queried
        self.object_input(create_object_instance('SecondObj', -0.5, -0.5, 0.5, obj_category))

        region_request = orion_actions.srv.SOMRegionQueryRequest()
        region_request.region_name = region1.name
        region_request.query.category = obj_category
        response: orion_actions.srv.SOMRegionQueryResponse = self.region_region_query(region_request)

        self.assertEqual(len(response.returns), 1, 'Only one object should be inside the region.')

        resp_obj = response.returns[0]
        self.assertEqual(resp_obj.class_, obj_inside.adding.class_, 'Object returned is not the one positioned inside the queried region.')


    def test_object_in_multiple_subregions_is_only_returned_once(self):
        """If a region is composed of multiple separate subregions, objects inside all of the
            subregions should be returned.
        """
        # Create two subregions that have an intersection
        region_name = 'Test_Regions_Intersection'
        region1 = create_region(region_name, [0, 0, 0], [2, 2, 1])
        region2 = create_region(region_name, [1, 1, 0], [1, 1, 1])
        self.region_input(orion_actions.srv.SOMAddRegionRequest(region1))
        self.region_input(orion_actions.srv.SOMAddRegionRequest(region2))

        # The object is located in the intersection of the two regions
        obj_category = 'intersection_obj'
        obj_inside = create_object_instance('test_intersection_obj', 1.5, 1.5, 0.5, obj_category)
        self.object_input(obj_inside)

        region_request = orion_actions.srv.SOMRegionQueryRequest()
        region_request.region_name = region_name
        region_request.query.category = obj_category
        response:orion_actions.srv.SOMRegionQueryResponse = self.region_region_query(region_request)

        self.assertEqual(len(response.returns), 1, 'The object has been returned more than once.')


if __name__ == '__main__':    
    import rostest
    rostest.rosrun('semantic_mapping', 'test_regions', 'test_regions.TestRegionManager')