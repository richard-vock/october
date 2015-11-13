###############################################################################
# Find October
#
# This sets the following variables:
# OCTOBER_FOUND - True if OCTOBER was found.
# OCTOBER_INCLUDE_DIRS - Directories containing the OCTOBER include files.
# OCTOBER_LIBRARY_DIRS - Directories containing the OCTOBER library.
# OCTOBER_LIBRARIES - OCTOBER library files.

if(WIN32)
    find_path(OCTOBER_INCLUDE_DIR october PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(OCTOBER_LIBRARY_PATH october PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${OCTOBER_LIBRARY_PATH})
        get_filename_component(OCTOBER_LIBRARY ${OCTOBER_LIBRARY_PATH} NAME)
        find_path(OCTOBER_LIBRARY_DIR ${OCTOBER_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(OCTOBER_INCLUDE_DIR october PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(OCTOBER_LIBRARY_PATH october PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${OCTOBER_LIBRARY_PATH})
        get_filename_component(OCTOBER_LIBRARY ${OCTOBER_LIBRARY_PATH} NAME)
        find_path(OCTOBER_LIBRARY_DIR ${OCTOBER_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(OCTOBER_INCLUDE_DIRS ${OCTOBER_INCLUDE_DIR})
set(OCTOBER_LIBRARY_DIRS ${OCTOBER_LIBRARY_DIR})
set(OCTOBER_LIBRARIES ${OCTOBER_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OCTOBER DEFAULT_MSG OCTOBER_INCLUDE_DIR OCTOBER_LIBRARY OCTOBER_LIBRARY_DIR)

mark_as_advanced(OCTOBER_INCLUDE_DIR)
mark_as_advanced(OCTOBER_LIBRARY_DIR)
mark_as_advanced(OCTOBER_LIBRARY)
mark_as_advanced(OCTOBER_LIBRARY_PATH)
