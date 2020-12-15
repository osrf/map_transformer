# -- ReadTheDocs customisation -----------------------------------------------

import subprocess, os

def configureDoxyfile(input_dir, output_dir):
    with open('doxyfile.in', 'r') as f:
        lines = f.read()
    lines = lines.replace('@DOXYGEN_INPUT_DIRS@', input_dir)
    lines = lines.replace('@DOXYGEN_OUTPUT_DIR@', output_dir)

    with open('Doxyfile', 'w') as of:
        of.write(lines)

rtd_build = os.environ.get('READTHEDOCS', None) == 'True'

breathe_projects = {}

if rtd_build:
    input_dir = '../include ../src'
    output_dir = 'build'
    configureDoxyfile(input_dir, output_dir)
    subprocess.call('doxygen', shell=True)
    breathe_projects['MapTransformer'] = output_dir + '/xml'

# -- Project information -----------------------------------------------------

project = 'MapTransformer'
copyright = '2020, Geoffrey Biggs'
author = 'Geoffrey Biggs'

# The full version, including alpha/beta/rc tags
release = '1.0.0'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['breathe']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

breathe_default_project = 'MapTransformer'
