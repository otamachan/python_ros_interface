
.. _examples:

Examples
--------

ROS Interface
*************

ROS Service
+++++++++++

.. code-block:: python

   from ros_interface import ROSService

   set_logger_level = ROSService('/namespace/node/set_logger_level')
   set_logger_level('node.class', 'DEBUG')
   print set_logger_level.request()

ROS Action
+++++++++++

.. code-block:: python

   from ros_interface import ROSAction

   fibonacci = ROSAction('/fibonacci')
   print fibonacci(4)

.. code-block:: python

   from ros_interface import ROSAction

   fibonacci = ROSAction('/fibonacci')
   goal = fibonacci.goal(4)
   # access SimpleActionClient properties
   fibonacci.set_goal(goal)
   fibonacci.wait_for_result()
   print fibonacci.get_result()

ROS Topic
+++++++++

.. code-block:: python

   from ros_interface import ROSTopic

   image_raw = ROSTopic('/camera/image_raw')
   print image_raw.get()

.. code-block:: python

   from ros_interface import ROSTopic

   image_raw = ROSTopic('/camera/image_raw')
   image_raw.subscribe()
   print image_raw.get()

.. code-block:: python

   from ros_interface import ROSTopic

   def callback(data):
       print data
   image_raw = ROSTopic('/camera/image_raw')
   image_raw.subscribe(callback=callback)

.. code-block:: python

   from ros_interface import ROSTopic

   chatter = ROSTopic('/chatter')
   chatter.put('Hello World')

.. code-block:: python

   from ros_interface import ROSTopic

   chatter = ROSTopic('/chatter', wait_clients=False,
                      data_class=std_msgs.msg.String, queue_size=1)
   chatter.put('Hello World')

ROS Parameter
+++++++++++++

.. code-block:: python

   from ros_interface import ROSParam

   dgain = ROSParam('/gains/dgain')
   x = dgain.get()
   dgain.set(4.0)

   igain = ROSParam('/gains/igain')
   y = param.get(0.0) # return 0.0 if the param doesn't exist

   gains = ROSParam('/gains')
   pgain = gains.get(suffix='pgain')
   pgain2 = gains.get()['pgain'] # same above
   param.set(2.1, suffix='pgain') # with default_value

ROS Interface
+++++++++++++

.. code-block:: python

   from ros_interface import ROSInterface
   from ros_interface import ROSActionProp, ROSServiceProp, ROSTopicProp, ROSParamProp

   class Foo(ROSInterface):
       _properties = {'fibonacci': ROSActionProp(),
                      'set_logger_level': ROSServiceProp(),
                      'chatter': ROSTopicProp(),
                      'some_topic': ROSTopicProp(),
                      'param': ROSParamProp()}
       foo = Foo('/namespace/hoge')
       print foo.fibonacci(3)
       print foo.set_logger_level('node', 'INFO')
       foo.chatter.put('hello world')
       print foo.some_topic.get()
       print foo.param
       foo.param = 2
       print foo.params['foo'].clear_cache()

Message Factory
***************
