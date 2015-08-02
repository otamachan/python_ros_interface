#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import rospy
import genpy
import rosservice
import rostopic
import actionlib
from rospy import ROSException
from .tf2 import Tf2Wrapper
from .exceptions import *

_TIMEOUT = 5.0
_POLLING = 20

def _wait_until(condition, timeout=None, polling=_POLLING):
    u"""
    Wait until condition returns true
    """
    if timeout is None:
        timeout_time = None
    else:
        timeout_time = rospy.get_rostime() + rospy.Duration(timeout)
    rate = rospy.Rate(polling)
    while not condition():
        if rospy.is_shutdown():
            break
        if timeout_time and rospy.get_rostime() > timeout_time:
            raise TimeoutException()
        rate.sleep()
    return condition()

class ROSService(object):
    u"""
    Provide ROS Service wrapper

    No need to specify a ROS service type beforehand
    because this wrapper resolves its python class from the service name at runtime.
    You cann call a service like a function call,
    as this class overrides :meth:`__call__` method.

    Creation of a ``ServiceProxy`` is delayed until it's needed.

    Args:
        service_name: ROS service name
        timeout: Timeout to wait for the service[s] (default: :const:`_TIMEOUT`)

    Attributes:
        name: ROS service name
    """
    def __init__(self, service_name, timeout=_TIMEOUT):
        self.name = service_name
        self._timeout = timeout
        self._service_proxy = None
        self._proxy_class = None
    def _resolve(self):
        u"""
        Create ``ServiceProxy`` if it is the first time
        """
        if self._service_proxy is None:
            rospy.logdebug("Createing ServiceProxy to %s", self.name)
            try:
                rospy.wait_for_service(self.name, self._timeout)
            except ROSException as ex:
                rospy.logerr("Failed to connect to %s / %s",
                             self.name, ex)
                raise
            self._proxy_class = rosservice.get_service_class_by_name(self.name)
            self._service_proxy = rospy.ServiceProxy(self.name, self._proxy_class)
    def __call__(self, *args, **kwargs):
        u"""
        Call the service

        Args:
            *args: Used to call the service
            **kwargs: Used to call the service

        Return:
            Return value from the ServiceProxy
        """
        self._resolve()
        try:
            rospy.logdebug("Calling service %s", self.name)
            ret = self._service_proxy(*args, **kwargs)
            rospy.logdebug("Returned from service %s", self.name)
            return ret
        except rospy.ServiceException as ex:
            rospy.logerr("Service call failed %s / %s", self.name, ex)
            raise
    @property
    def request(self):
        u"""
        Return the serivce request class

        Return:
            The service request class
        """
        self._resolve()
        return self._proxy_class._request_class

class ROSAction(object):
    u"""
    Provide ROS Service wrapper

    No need to specify a ROS action type beforehand
    because this wrapper resolves its python class from the action name at runtime.
    You cann call a action like a function call,
    as this class overrides :meth:`__call__` method.
    (Wait for the completion of the action)

    Creation of a ``SimpleACionClient`` is delayed until it's needed.

    Args:
        action_name: ROS action name
        timeout: Timeout to wait for the action server[s] (default: :const:`_TIMEOUT`)

    Attributes:
        name: ROS action name

    """
    _action_class_cache = {}
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9
    GOALIDS = {0: 'PENDING',
               1: 'ACTIVE',
               2: 'PREEMPTED',
               3: 'SUCCEEDED',
               4: 'ABORTED',
               5: 'REJECTED',
               6: 'PREEMPTING',
               7: 'RECALLING',
               8: 'RECALLED',
               9: 'LOST'}
    TERMINAL = (2, 3, 4, 5, 7, 8)
    def __init__(self, action_name, timeout=_TIMEOUT):
        self.name = action_name
        self._action_client = None
        self._timeout = timeout
        self._action_class = None
        self._goal_class = None
    def _resolve(self):
        u"""
        Create ``SimpleActionClient`` if it is the first time
        """
        if self._action_client is None:
            action_goal_topic_type = rostopic.get_topic_type(self.name + '/goal')[0]
            if not action_goal_topic_type:
                raise NotResolvableException("cannot find action %s" % self.name)
            # get the action class
            action_type = action_goal_topic_type[:-4]
            if action_type in self._action_class_cache:
                self._action_class = self._action_class_cache[action_type]
            else:
                self._action_class = genpy.message.get_message_class(action_type)
                self._action_class_cache[action_type] = self._action_class
            # get the goal class
            goal_type = action_type[:-6]+'Goal'
            self._goal_class = genpy.message.get_message_class(goal_type)
            if not self._goal_class:
                raise NotResolvableException("cannot find action goal %s" % goal_type)
            # wait
            self._action_client = actionlib.SimpleActionClient(self.name, self._action_class)
            self._action_client.wait_for_server(timeout=rospy.Duration(self._timeout))
    def __call__(self, *args, **kwargs):
        u"""
        Call the action

        Args:
            *args:
            **kwargs:
        Return:
            Return value from the action
        Raises:
            NotResolvableException:

        """
        self._resolve()
        if 'timeout' in kwargs:
            timeout = kwargs['timeout']
            del kwargs['timeout']
        else:
            timeout = 0.0
        goal = self.goal(*args, **kwargs)
        rospy.logdebug("Calling action %s timeout=%f", self.name, timeout)
        state = self._action_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(timeout))
        rospy.logdebug("Returned from action %s state[%s]", self.name, state)
        return self._action_client.get_result()

    def goal(self, *args, **kwargs):
        u"""
        Create a goal object

        Args:
            *args:
            **kwargs:
        Return:
            Goal object
        Raises:
            NotResolvableException:
        """
        self._resolve()
        return self._goal_class(*args, **kwargs)

    def __getattr__(self, attr):
        u"""
        Get access to unkown attribute

        Args:
            attr:
        Raises:
            AttributeError:
            NotResolvableException:
        """
        self._resolve()
        if self._action_client is not None:
            if hasattr(self._action_client, attr):
                return getattr(self._action_client, attr)
        raise AttributeError("%r object has no attribute %r" %
                             (self.__class__, attr))

class ROSTopic(object):
    u"""
    Provide ROS Service wrapper

    Args:
        topic_name:
        publisher_wait_subscribers:
        publisher_timeout:
        **kwargs:

    Attributes:
        name:

    """
    def __init__(self, topic_name,
                 data_class = None,
                 wait_for_subscribers=True,
                 publisher_timeout=_TIMEOUT,
                 **kwargs):
        self.name = topic_name
        self._publisher_timeout = publisher_timeout
        self._wait_for_subscribers=wait_for_subscribers
        self._data_class = data_class
        self._subscriber = None
        self._publisher = None
        self._data = None
        self._kwargs = kwargs
        self._updated = False # This flag is not threadsafe
    def subscribe(self, wait_first=True, timeout=_TIMEOUT, **kwargs):
        u"""
        Start subscription

        Args:
            wait_first:
            timeout:
            **kwargs:
        Raises:
            TimeoutException
        Raises:
            NotResolvableException:
        """
        rospy.logdebug("Start subscrining %s", self.name)
        if self._subscriber is not None:
            self.unsubscribe()
        self._updated = False
        self._subscriber = self._get_subscriber(**kwargs)
        if wait_first and 'callback' not in kwargs:
            self._wait_update(timeout=timeout)
    def unsubscribe(self):
        u"""
        Stop subscription
        """
        if self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None
    def get(self, wait_update=False, timeout=_TIMEOUT, **kwargs):
        u"""
        Return the last value

        Args:
            wait_update:
            timeout:
            **kwargs:
        Return:
            The last value
        Raises:
            TimeoutException:
            NotResolvableException:
        """
        if self._subscriber is None:
            # subscribe once
            self._updated = False
            if 'callback' in kwargs:
                del kwargs['callback']
            sub = self._get_subscriber(**kwargs)
            self._wait_update(timeout=timeout)
            sub.unregister()
        else:
            # wait_update or not updated yet
            if wait_update or not self._updated:
                self._updated = False
                self._wait_update(timeout=timeout)
        return self._data
    def put(self, *args, **kwargs):
        u"""
        Publish one data

        Args:
            *args:
            **kwargs:
        Raises:
            TimeoutException
            NotResolvableException:
        """
        if self._publisher is None:
            self._publisher = self._get_publisher(**self._kwargs)
            if self._wait_for_subscribers:
                rospy.logdebug("Start publishing. Wait for subscribers %s", self.name)
                try:
                    _wait_until(lambda: self._publisher.get_num_connections() > 0,
                                timeout=self._publisher_timeout)
                except TimeoutException:
                    rospy.logerr("No subscriber is subscribing for %s", self.name)
                    raise
                rospy.logdebug("Connected %s", self.name)
        self._publisher.publish(*args, **kwargs)
    @property
    def data_class(self):
        u"""
        Return the topic data class

        Return:
            Topic data class
        """
        if self._data_class is None:
            self._resolve()
        return self._data_class

    def _resolve(self):
        u"""
        Resolve Topic type from topic name
        """
        if self._data_class is None:
            topic_type = rostopic.get_topic_type(self.name)[0]
            if topic_type:
                self._data_class = genpy.message.get_message_class(topic_type)
            else:
                raise NotResolvableException("cannot find topic: %s" % self.name)
    def _get_subscriber(self, **kwargs):
        u"""
        Create a subscriber
        """
        self._resolve()
        if 'callback' not in kwargs:
            kwargs['callback'] = self._callback
        return rospy.Subscriber(self.name, self._data_class, **kwargs)
    def _get_publisher(self, **kwargs):
        u"""
        Create a publisher
        """
        if 'queue_size' not in kwargs:
            kwargs['queue_size'] = 1
        return rospy.Publisher(self.name, self.data_class, **kwargs)
    def _callback(self, data):
        u"""
        Callback from Subscriber
        """
        self._data = data
        self._updated = True
    def _wait_update(self, timeout=None):
        u"""
        Wait until data is updated
        """
        try:
            _wait_until(lambda: self._updated,
                        timeout=timeout)
        except TimeoutException:
            rospy.logerr("Failed to recieve from %s", self.name)
            raise

class _Unspecified():
    def __repr__(self): return 'UNSPECIFIED'
_UNSPECIFIED = _Unspecified()

class ROSParam(object):
    u"""
    Provide ROS Parameter wrapper

    Args:
        param_name:
        cache:

    Attributes:
        name:
    """
    def __init__(self, param_name, cache=True):
        self.name = param_name
        self._cache = cache
        self._cached_value = _UNSPECIFIED
    def get(self, default_value=_UNSPECIFIED, suffix=''):
        u"""
        Get the ROS parameter

        Args:
            default_value:
            suffix:
        Return:
            Parameter value
        """
        if suffix:
            # do not cache
            name = self.name + '/' + suffix
            if default_value is _UNSPECIFIED:
                value = rospy.get_param(name)
            else:
                value = rospy.get_param(name, default_value)
        else:
            if self._cached_value is not _UNSPECIFIED:
                value = self._cached_value
            elif default_value is _UNSPECIFIED:
                value = rospy.get_param(self.name)
                if self._cache:
                    self._cached_value = value
            else:
                value = rospy.get_param(self.name, default_value)
        return value
    def set(self, value, suffix=''):
        u"""
        Set the ROS parameter

            value:
            suffix:
        """
        if suffix:
            name = self.name + '/' + suffix
        else:
            name = self.name
            if self._cache:
                self._cached_value = value
        rospy.set_param(name, value)
    def clear_cache(self):
        u"""
        Clear the cache
        """
        self._cached_value = _UNSPECIFIED
class ROSWrapperFactory(object):
    u"""
    ROS wrapper factory

    Args:
        name:
        **kwargs:
    """
    _wrapper = None
    _is_param = False
    def __init__(self, name='', **kwargs):
        self._name = name
        self._kwargs = kwargs
    def create(self, name, prefix):
        u"""生成する"""
        if self._name:
            name = prefix + '/' + self._name
        else:
            name = prefix + '/' + name
        return self._wrapper(name, **self._kwargs)

class ROSTopicProp(ROSWrapperFactory):
    u"""
    :class:`ros.ROSTopic` factory

    Args:
        name:
        **kwargs:
    """
    _wrapper = ROSTopic

class ROSServiceProp(ROSWrapperFactory):
    u"""
    :class:`ros.ROSService` factory

    Args:
        name:
        **kwargs:
    """
    _wrapper = ROSService

class ROSActionProp(ROSWrapperFactory):
    u"""
    :class:`ros.ROSAction` factory

    Args:
        name:
        **kwargs:
    """
    _wrapper = ROSAction

class ROSParamProp(ROSWrapperFactory):
    u"""
    :class:`ros.ROSParam` factory

    Args:
        name:
        **kwargs:
    """
    _wrapper = ROSParam
    _is_param = True

class ROSInterface(object):
    u"""
    Provide ROS interface wrapper

    Args:
         prefix:
         properties:

    Attributes:
        _properties:
    """
    _properties = {}
    def __init__(self, prefix='', properties=None):
        self.params = {}
        if properties is None:
            properties = self._properties
        for prop_name, prop in properties.items():
            wrapper = prop.create(prop_name, prefix)
            if prop._is_param:
                self.params[prop_name] = wrapper
            else:
                setattr(self, prop_name, wrapper)


    def __getattr__(self, attr):
        u"""
        Get access to unknown attribute

        Args:
            attr:
        Raises:
             AttributeError:
        """
        if hasattr(self, 'params') and attr in self.params:
            return self.params[attr].get()
        raise AttributeError("%r object has no attribute %r" %
                             (self.__class__, attr))

    def __setattr__(self, attr, value):
        u"""
        Set access to unknown attribute

        Args:
            attr:
            value:
        Raises:
             AttributeError:
        """
        if hasattr(self, 'params') and attr in self.params:
            self.params[attr].set(value)
        else:
            super(ROSInterface, self).__setattr__(attr, value)

class ROSConnection(object):
    u"""
    Singletone class to manage connection with ROS Network
    """
    __instance = None
    def __new__(cls, **kwargs):
        if ROSConnection.__instance is None:
            ROSConnection.__instance = object.__new__(cls, **kwargs)
        return ROSConnection.__instance
    def __init__(self):
        if not rospy.core.is_initialized():
            name = self.__class__.__name__
            rospy.init_node(name.replace('.', '_'), anonymous=True)
        #while not rospy.Time.now():
        #    time.sleep(0.1) #TODO: timeout
        self.tf2 = Tf2Wrapper()

class SubscribeManager(object):
    u"""
    Manager class for subscribers

    Args:
        subscribe_tf:

    Attributes:
        tf2:
    """
    def __init__(self, subscribe_tf=True):
        self._connection = ROSConnection()
        self._subscribers = []
        self.tf2 = self._connection.tf2
        self._subscribe_tf = subscribe_tf

    def add_subscriber(self, topic_wrapper, wait_first=True, timeout=5.0, **kwargs):
        u"""
        Register a subscriber in :attr:`_subscribers`

        Args:
            topic_wrapper:
            wait_first:
            timeout:
            **kwargs:
        """
        kwargs['wait_first'] = wait_first
        kwargs['timeout'] = timeout
        self._subscribers.append((topic_wrapper, kwargs))

    def subscribe(self):
        u"""
        Start subscription
        """
        if self._subscribe_tf:
            self.tf2.subscribe()
        for topic_wrapper, kwargs in self._subscribers:
            topic_wrapper.subscribe(**kwargs)

    def unsubscribe(self):
        u"""
        Stop subscription
        """
        # Not stop the tf2 because other clients may still use it
        for topic_wrapper, _ in self._subscribers:
            topic_wrapper.unsubscribe()
