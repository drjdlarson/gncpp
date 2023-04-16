# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------
import subprocess
import os



# -- Project information -----------------------------------------------------

project = "GNCPy C++"
copyright = "2023, Laboratory for Autonomy, GNC, and Estimation Research (LAGER)"
author = "LAGER"



# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.autosectionlabel',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx_sitemap',
    'sphinx.ext.inheritance_diagram',
    "sphinxcontrib.bibtex",
    'sphinx_copybutton',
    'sphinx_rtd_theme'
]

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'

default_role = 'cpp:any'

# configure copy button for code snippets
copybutton_only_copy_prompt_lines = False


# Todo configuration
todo_include_todos = True
todo_link_only = True

# bibtex config
bibtex_bibfiles = ["refs.bib"]


# -- Options for HTML output -------------------------------------------------
# html_theme = "stanford_theme"
html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "display_version": True,
    'style_nav_header_background': '#9E1B32',
    # Toc options
    "collapse_navigation": False,
    "sticky_navigation": True,
    "navigation_depth": 4,
}
html_show_sourcelink = False
html_baseurl = "https://drjdlarson.github.io/gncpy_cpp/"
html_extra_path = ["api",]
html_logo = "logo.svg"
