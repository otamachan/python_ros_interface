# -*- coding: utf-8 -*-
import os
import sys
import sphinx_rtd_theme
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
