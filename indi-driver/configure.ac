AC_INIT([powerbox-driver], [0.1], [philipp.ph.weber@fau.de])

AC_PREFIX_DEFAULT([/usr])

AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AC_PROG_CC
AC_PROG_CXX
AM_PROG_AR

LT_PREREQ([2.4.6])
LT_INIT
AC_PROG_LIBTOOL
AM_PROG_LIBTOOL

AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_MACRO_DIRS([m4])

CFLAGS="$CFLAGS -g -pedantic -W -Wall -pthread"
CXXFLAGS="$CXXFLAGS -g -pedantic -W -Wall -pthread"
LDFLAGS="$LDFLAGS -g -W -Wall"


#### PUSH C++ ####
AC_LANG_PUSH([C++])
# FLag std=c++17
saved_cxxflags="$CXXFLAGS"
CXXFLAGS="-Werror -std=c++17"
AC_MSG_CHECKING([whether CXX supports -std=c++17])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([])], [AC_MSG_RESULT([yes]); CPPUTEST_CPP17FLAG="-std=c++17" ], [AC_MSG_ERROR([C++ compiler does not support C++17])])
CXXFLAGS="${saved_cxxflags} -std=c++17"

# FIND INDI
AC_ARG_WITH([indi],
            [AC_HELP_STRING([--with-indi=<indi path>], [path to indi installation])],
            [indiloc=$withval],
            [indiloc="/usr"])
AS_IF([ [[ "${indiloc}" != "notgiven" ]] ],
      [LDFLAGS="$LDFLAGS -L${indiloc}/lib -Wl,--rpath=${indiloc}/lib" && CXXFLAGS="$CXXFLAGS -cxx-isystem ${indiloc}/include/libindi" && AC_SUBST([INDILOC], [$indiloc])],
      [])

AC_SEARCH_LIBS(sin, m, [], [AC_MSG_ERROR([No math library found!])], [])
AC_SEARCH_LIBS(compress, z, [], [AC_MSG_ERROR([z library not found!])], [])
AC_SEARCH_LIBS(ln_deg_to_dms, nova, [], [AC_MSG_ERROR([nova library not found!])], [])
LIBS="-lindidriver $LIBS"


##### POP C++ ####
AC_LANG_POP()

AC_CONFIG_FILES([
		 Makefile
		 src/Makefile
		 xml/Makefile
		 ])

AC_OUTPUT
