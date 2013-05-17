set(ThirdParty_DIR ${PROJECT_SOURCE_DIR}/3rd-party)

set(CLAPACK_ROOT $ENV{CLAPACK_ROOT})
set(CLAPACK_INCLUDE_DIR ${CLAPACK_ROOT}/include)
set(CLAPACK_LIBRARY_DIR ${CLAPACK_ROOT}/lib)
set(BLAS_LIBRARY optimized blas.lib debug blasd.lib)
set(LIBF2C_LIBRARY optimized libf2c.lib debug libf2cd.lib)
set(LAPACK_LIBRARY optimized lapack.lib debug lapackd.lib)

include_directories(${CLAPACK_INCLUDE_DIR})
link_directories(${CLAPACK_LIBRARY_DIR})
set(ThirdParty_LIBS ${ThirdParty_LIBS} ${BLAS_LIBRARY} ${LIBF2C_LIBRARY} ${LAPACK_LIBRARY})