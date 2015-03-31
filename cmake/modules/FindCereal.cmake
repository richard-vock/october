###############################################################################
# Find Cereal
#
# This sets the following variables:
# CEREAL_FOUND - True if Cereal was found.
# CEREAL_INCLUDE_DIRS - Directories containing the Cereal include files.
# CEREAL_LIBRARY_DIRS - Directories containing the Cereal library.
# CEREAL_LIBRARIES - Cereal library files.

find_path(CEREAL_INCLUDE_DIR cereal
    HINTS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}")

set(CEREAL_INCLUDE_DIRS ${CEREAL_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Cereal DEFAULT_MSG CEREAL_INCLUDE_DIR)

mark_as_advanced(CEREAL_INCLUDE_DIR)
