# Testing
How to test the Semantic Mapping package.

## Running Tests
Test nodes can either be run directly from the command line using `rosrun`, or they can be run from a `.test` file using `rostest`.

With `rosrun`, the SOM node must be running, then a test node can be executed as any other node. For example:
```
rosrun semantic_mapping tests.py
```

If a `.test` file is used (such as `alltests.test`), it will not require to run the SOM node beforehand.
You can execute the file as follows:
```
rostest semantic_mapping alltests.test
```

However, if the `.test` file is used, it will not write whether some tests were skipped (they will be considered successful).

## Writing Tests
Tests are written using the `unittest` library.

Test cases are defined as classes that inherit from the `TestCase` unittest class. Each class can contain multiple tests, which will check different aspects of the same feature. Each test is defined as a method of the class,
which name starts with 'test'.

For example, the class `TestRetrievingObservationByBatchNumber` inside `tests.py` is used to test retrieving observations using their batch number (as the name says). The class contains multiple tests,
which check specific elements of this feature: `test_query_positive_batch_returns_only_that_batch` and `test_query_interval_of_batches_returns_correct_value`.

If a file contains multiple testcases, they must be grouped into a single `TestSuite` class:
``` python
class GeneralTestsuite(unittest.TestSuite):
    def __init__(self):
        super(GeneralTestsuite, self).__init__()
        self.addTest(unittest.makeSuite(TestObservationInputDistanceUpdate))
        self.addTest(unittest.makeSuite(TestHumanObservationInput))
```

The tests can then be run using using `rostest.rosrun(<package name>, <test name>, <testcase class>)`, where:
- `package name` is the name of the package which contains the test (`semantic_mapping`)
- `test name` is the name that will be used when reporting the result of the test
- `testcase class` is the class that contains the tests, passed as a string: `<filename>.<classname>`. It will be the `TestCase` if the file contains only one, or the `TestSuite` otherwise.

For example, the `test_similarity.py` file contains:
``` python
rostest.rosrun('semantic_mapping', 'test_ontology_similarity', 'test_similarity.TestOntologySimilarity')
```

### Skipping Tests
If a test should not be run, it will be skipped if the `skipTest` decorator (from `useful_test_func.py`) is used. This is a wrapper around the `unittest.skip` method, but using it will also 
print on the terminal that the test was skipped.
This can be applied to either an entire `TestCase`, or to a single test function.

## Writing Rostest Files
They are similar to `roslaunch` files, but can contain `<test>` elements. These will be written as follows:
```
<test test-name="test_similarity" pkg="semantic_mapping" type="test_similarity.py"/>
```

Here `test-name` will be used when reporting test results, `pkg` abd `type` are the same as in `roslaunch` files.

However, if the SOM main node is also run from the same file, you may want to delay the start of the test in order to run it only after the SOM node is ready. To do so, add to each test element: 
```
launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' " 
````

In addition, you will need to add the element:
```
<arg name="node_start_delay" default="1.0" />  
```