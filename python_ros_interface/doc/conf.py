# -*- coding: utf-8 -*-

import os
import sys
import sphinx_rtd_theme
import mock

MOCK_MODULES = ['numpy', 'rospy', 'genpy', 'rosservice', 'rostopic', 'actionlib', 'tf2_ros', 'tf', 'tf.transformations']
for mod_name in MOCK_MODULES:
    sys.modules[mod_name] = mock.Mock()
MESSAGE_MODULES = ['tf2', 'geometry', 'trajectory', 'std', 'sensor']
for msg_name in MESSAGE_MODULES:
    sys.modules[msg_name+'_msgs'] = mock.Mock()
    sys.modules[msg_name+'_msgs.msg'] = mock.Mock()

html_theme = 'sphinx_rtd_theme'
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
extensions = ['sphinx.ext.autodoc', 'sphinxcontrib.napoleon']
master_doc = 'index'
project = u'python_ros_interface'
copyright = u'2015-, T. Nishino'
author = u'T. Nishino'
version = '0.1.0'
release = '0.1.0'
language = 'en'
exclude_patterns = ['_build']
pygments_style = 'sphinx'
autodoc_member_order='bysource'
