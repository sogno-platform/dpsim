#!/bin/bash
set -e -x

case $(uname -s) in
    Darwin)
        PLATFORM=macOS
        REQUIREMENTS="delocate"
        VERSIONS="cp37"
        CMAKE_OPTS_BASE="-DCMAKE_BUILD_PARALLEL_LEVEL=$(sysctl -n hw.ncpu)"
        ;;

    Linux)
        PLATFORM=Linux
        REQUIREMENTS="" # Are bundled in Manylinux image
        #PYDIRS=(/opt/python/{cp36,cp37}-*)
        PYDIRS=(/opt/python/cp37-*)
        CMAKE_OPTS_BASE="-DCMAKE_BUILD_PARALLEL_LEVEL=$(nproc) \
                         -DFETCH_EIGEN=ON \
                         -DFETCH_CIMPP=ON \
                         -DFETCH_SPDLOG=ON"
        ;;

    MINGW*)
        PLATFORM=Windows
        ;;
esac

# Compile wheels
for PYDIR in ${PYDIRS}; do
    LIB=(${PYDIR}/lib/python*)
    INC=(${PYDIR}/include/python*)
    EXC=(${PYDIR}/bin/python*)

    # CMAKE_OPTS is used by our setup.py script
    export CMAKE_OPTS="-DCMAKE_BUILD_PARALLEL_LEVEL=$(nproc) \
                       -DFETCH_EIGEN=ON \
                       -DFETCH_CIMPP=ON \
                       -DFETCH_SPDLOG=ON \
                       -DPYTHON_EXECUTABLE=${EXC} \
                       -DPYTHON_INCLUDE_DIRS=${INC} \
                       -DPYTHON_LIBRARY=${LIB}"

   "${PYDIR}/bin/pip" install .
   "${PYDIR}/bin/pip" wheel . --wheel-dir boilerhouse --no-deps
   #"${PYDIR}/bin/python" ./setup.py bdist_wheel
done

# Fixup wheels by including external libs
if [ ${PLATFORM} == "Linux" ]; then
    # Bundle external shared libraries into the wheels
    for WHEEL in boilerhouse/*.whl; do
        auditwheel show "${WHEEL}"
        auditwheel repair "${WHEEL}" --plat ${PLAT} --wheel-dir wheelhouse
    done
elif [ ${PLATFORM} == "macOS" ]; then
    for WHEEL in boilerhouse/*.whl; do
        delocate-listdeps --all --depending ${WHEEL}
        delocate-wheel -w wheelhouse -v ${WHEEL}
    done
elif [ ${PLATFORM} == "Windows" ]; then
    # Nothing to do here yet.
fi
