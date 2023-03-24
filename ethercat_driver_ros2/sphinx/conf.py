# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
from pathlib import Path

# -- Project information -----------------------------------------------------

project = "ethercat_driver_ros2"
copyright = "2023, ICUBE Laboratory, University of Strasbourg"
author = "Maciej Bednarczyk, Philippe Zanne, Laurent Barbé"

# The full version, including alpha/beta/rc tags
release = "1.0.0"


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx_rtd_theme",
    "sphinx_mdinclude",
    "sphinx.ext.imgmath",
    "sphinx.ext.todo",
    "sphinx.ext.graphviz",
    "sphinxcontrib.plantuml",
    "breathe",
]

breathe_default_project = "ethercat_driver_ros2"


def get_package(package: str):
    path = Path(__file__).parent.parent.parent.joinpath(f"{package}/include/{package}")
    files_gen = path.glob("*.hpp")
    files = []
    for file in files_gen:
        files.append(file.name)
    return (path, files)


breathe_projects = {
    "ethercat_driver_ros2": "../doxygen/_build/xml/",
}


# Tell sphinx what the primary language being documented is.
primary_domain = "cpp"

# Tell sphinx what the pygments highlight language should be.
highlight_language = "cpp"


# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]
html_logo = "images/logo-icube.png"
html_css_files = ["css/custom.css"]
pygments_style = "sphinx"
